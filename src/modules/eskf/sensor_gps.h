//
// Created by Cain on 2023/3/2.
//

#ifndef ECL_SENSOR_GPS_H
#define ECL_SENSOR_GPS_H

#include "sensor.h"

namespace eskf {
    template<uint8_t DELAYS>
    class SensorGps : public Sensor<GpsSample, DELAYS> {
    public:
        SensorGps() = default;
        bool collect_gps(const GpsMessage &gps, bool in_air, bool at_rest, uint64_t time_current);

        bool checks_passed;
        bool rtk_available;

        Vector3f w_cross_offset_body {};
        Vector3f w_cross_offset_nav {};

        float hgt_imu {};
        Vector2f pos_horz_imu {};
        Vector3f vel_imu {};

        MapProjection pos_prev {}; // Contains WGS-84 position latitude and longitude of the previous GPS message
        float alt_prev {0.0f};	// height from the previous GPS message (m)

        float mag_declination {NAN};
        float mag_inclination {NAN};
        float mag_strength {NAN};

        float pos_horz_error_norm {1.f};
        float pos_vert_error_norm {1.f};
        float vel_error_norm {1.f};

        Vector3f pos_deriv_filt {};
        float pos_horz_deriv_filt_norm {0.f};
        float pos_vert_deriv_filt_norm {0.f};

        Vector2f vel_horz_filt {};
        float vel_horz_filt_norm {0.f};
        float vel_vert_diff_filt {0.f};

        gps_check_fail_status_u check_fail_status {0};

        uint64_t time_last_hgt_fuse {};
        uint64_t last_fail_us {};
        uint64_t last_pass_us {};
        uint64_t last_origin_us {0};

        static float origin_unc_pos_horz; // horizontal position uncertainty of the GPS origin
        static float origin_unc_pos_vert; // vertical position uncertainty of the GPS origin
        static MapProjection pos_ref; // Contains WGS-84 position latitude and longitude of the ESKF origin
        static float alf_ref;
        static float yaw_offset;	// Yaw offset angle for dual GPS antennas used for yaw estimation (radians).
        static float earth_rate_ned;

        static constexpr float req_hacc{5.0f};			///< maximum acceptable horizontal position error (m)
        static constexpr float req_vacc{8.0f};			///< maximum acceptable vertical position error (m)
        static constexpr float req_sacc{1.0f};			///< maximum acceptable speed error (m/s)
        static constexpr int32_t req_nsats{6};			///< minimum acceptable satellite count
        static constexpr float req_pdop{2.0f};			///< maximum acceptable position dilution of precision
        static constexpr float req_hdrift{0.3f};			///< maximum acceptable horizontal drift speed (m/s)
        static constexpr float req_vdrift{0.5f};			///< maximum acceptable vertical drift speed (m/s)

    protected:
        void reset_gps_drift_check_filters();
        bool gps_is_good(const GpsMessage &gps, bool in_air, bool at_rest, uint64_t time_current);
    };

    template<uint8_t DELAYS>
    float SensorGps<DELAYS>::origin_unc_pos_horz = 0.f;

    template<uint8_t DELAYS>
    float SensorGps<DELAYS>::origin_unc_pos_vert = 0.f;

    template<uint8_t DELAYS>
    MapProjection SensorGps<DELAYS>::pos_ref {};

    template<uint8_t DELAYS>
    float SensorGps<DELAYS>::alf_ref = 0.f;

    template<uint8_t DELAYS>
    float SensorGps<DELAYS>::yaw_offset = 0.f;

    template<uint8_t DELAYS>
    float SensorGps<DELAYS>::earth_rate_ned = 0.f;

    template<uint8_t DELAYS>
    void SensorGps<DELAYS>::reset_gps_drift_check_filters() {
        vel_horz_filt.setZero();
        pos_deriv_filt.setZero();

        pos_horz_deriv_filt_norm = NAN;
        pos_vert_deriv_filt_norm = NAN;
        vel_horz_filt_norm = NAN;
    }

    template<uint8_t DELAYS>
    bool SensorGps<DELAYS>::gps_is_good(const GpsMessage &gps, bool in_air, bool at_rest, uint64_t time_current) {
        // Check the fix type
        check_fail_status.flags.fix = (gps.fix_type < 3);

        // Check the number of satellites
        check_fail_status.flags.nsats = (gps.nsats < req_nsats);

        // Check the position dilution of precision
        check_fail_status.flags.pdop = (gps.pdop > req_pdop);

        // Check the reported horizontal and vertical position accuracy
        check_fail_status.flags.hacc = (gps.eph > req_hacc);
        check_fail_status.flags.vacc = (gps.epv > req_vacc);

        // Check the reported speed accuracy
        check_fail_status.flags.sacc = (gps.sacc > req_sacc);

        // check if GPS quality is degraded
        pos_horz_error_norm = gps.eph / req_hacc;
        pos_vert_error_norm = gps.epv / req_vacc;
        vel_error_norm = gps.sacc / req_sacc;

        // Calculate time lapsed since last update, limit to prevent numerical errors and calculate a lowpass filter coefficient
        constexpr float filt_time_const = 10.0f;
        const float dt = math::constrain(float(int64_t(time_current) - int64_t(pos_prev.getProjectionReferenceTimestamp())) * 1e-6f, 0.001f, filt_time_const);
        const float filter_coef = dt / filt_time_const;

        // The following checks are only valid when the vehicle is at rest
        const double lat = gps.lat * 1.0e-7;
        const double lon = gps.lon * 1.0e-7;

        if (!in_air && at_rest) {
            // Calculate position movement since last measurement
            float delta_pos_n = 0.0f;
            float delta_pos_e = 0.0f;

            // calculate position movement since last GPS fix
            if (pos_prev.getProjectionReferenceTimestamp() > 0) {
                pos_prev.project(lat, lon, delta_pos_n, delta_pos_e);

            } else {
                // no previous position has been set
                pos_prev.initReference(lat, lon, time_current);
                alt_prev = 1e-3f * (float)gps.alt;
            }

            // Calculate the horizontal and vertical drift velocity components and limit to 10x the threshold
            const Vector3f vel_limit(req_hdrift, req_hdrift, req_vdrift);
            Vector3f pos_derived(delta_pos_n, delta_pos_e, (alt_prev - 1e-3f * (float)gps.alt));
            pos_derived = matrix::constrain(pos_derived / dt, -10.0f * vel_limit, 10.0f * vel_limit);

            // Apply a low pass filter
            pos_deriv_filt = pos_derived * filter_coef + pos_deriv_filt * (1.0f - filter_coef);

            // Calculate the horizontal drift speed and fail if too high
            pos_horz_deriv_filt_norm = Vector2f(pos_deriv_filt.xy()).norm();
            check_fail_status.flags.hdrift = (pos_horz_deriv_filt_norm > req_hdrift);

            // Fail if the vertical drift speed is too high
            pos_vert_deriv_filt_norm = fabsf(pos_deriv_filt(2));
            check_fail_status.flags.vdrift = (pos_vert_deriv_filt_norm > req_vdrift);

            // Check the magnitude of the filtered horizontal GPS velocity
            const Vector2f gps_vel_horz = matrix::constrain(Vector2f(gps.vel_ned.xy()),
                                                            -10.0f * req_hdrift,
                                                            10.0f * req_hdrift);
            vel_horz_filt = gps_vel_horz * filter_coef + vel_horz_filt * (1.0f - filter_coef);
            vel_horz_filt_norm = vel_horz_filt.norm();
            check_fail_status.flags.hspeed = (vel_horz_filt_norm > req_hdrift);

        } else if (in_air) {
            // These checks are always declared as passed when flying
            // If on ground and moving, the last result before movement commenced is kept
            check_fail_status.flags.hdrift = false;
            check_fail_status.flags.vdrift = false;
            check_fail_status.flags.hspeed = false;

            reset_gps_drift_check_filters();

        } else {
            // This is the case where the vehicle is on ground and IMU movement is blocking the drift calculation
            reset_gps_drift_check_filters();
        }

        // save GPS fix for next time
        pos_prev.initReference(lat, lon, time_current);
        alt_prev = 1e-3f * (float)gps.alt;

        // assume failed first time through
        if (last_fail_us == 0) {
            last_fail_us = time_current;
        }

        // if any user selected checks have failed, record the fail time
        if (
                check_fail_status.flags.fix ||
                (check_fail_status.flags.nsats   && (_params.gps_check_mask & MASK_GPS_NSATS)) ||
                (check_fail_status.flags.pdop    && (_params.gps_check_mask & MASK_GPS_PDOP)) ||
                (check_fail_status.flags.hacc    && (_params.gps_check_mask & MASK_GPS_HACC)) ||
                (check_fail_status.flags.vacc    && (_params.gps_check_mask & MASK_GPS_VACC)) ||
                (check_fail_status.flags.sacc    && (_params.gps_check_mask & MASK_GPS_SACC)) ||
                (check_fail_status.flags.hdrift  && (_params.gps_check_mask & MASK_GPS_HDRIFT)) ||
                (check_fail_status.flags.vdrift  && (_params.gps_check_mask & MASK_GPS_VDRIFT)) ||
                (check_fail_status.flags.hspeed  && (_params.gps_check_mask & MASK_GPS_HSPD))
                ) {
            last_fail_us = time_current;

        } else {
            last_pass_us = time_current;
        }

        // continuous period without fail of x seconds required to return a healthy status
        return this->is_timeout(time_current, last_fail_us, RunnerParameters::MIN_GPS_HEALTH);
    }

    template<uint8_t DELAYS>
    bool SensorGps<DELAYS>::collect_gps(const GpsMessage &gps, bool in_air, bool at_rest, uint64_t time_current) {
        // Run GPS checks always
        checks_passed = gps_is_good(gps);

        if (!_ned_origin_initialised && checks_passed) {
            // If we have good GPS data set the origin's WGS-84 position to the last gps fix
            const double lat = gps.lat * 1.0e-7;
            const double lon = gps.lon * 1.0e-7;

            if (!pos_ref.isInitialized()) {
                pos_ref.initReference(lat, lon, time_current);

                // if we are already doing aiding, correct for the change in position since the EKF started navigating
                if (is_horizontal_aiding_active()) {
                    double est_lat;
                    double est_lon;
                    pos_ref.reproject(-_eskf._state.pos(0), -_eskf._state.pos(1), est_lat, est_lon);
                    pos_ref.initReference(est_lat, est_lon, _imu_sample_last.time_us);
                }
            }

            // Take the current GPS height and subtract the filter height above origin to estimate the GPS height of the origin
            alf_ref = 1e-3f * (float)gps.alt + _eskf._state.pos(2);
            _ned_origin_initialised = true;

            earth_rate_ned = calc_earth_rate_ned((float)math::radians(_gps_state.pos_ref.getProjectionReferenceLat()));
            last_origin_us = time_current;

            const bool declination_was_valid = PX4_ISFINITE(mag_declination);

            // set the magnetic field data returned by the geo library using the current GPS position
            mag_declination = get_mag_declination_radians((float)lat, (float)lon);
            mag_inclination = get_mag_inclination_radians((float)lat, (float)lon);
            mag_strength = get_mag_strength_gauss((float)lat, (float)lon);

//            // request a reset of the yaw using the new declination
//            if ((_params.mag_body_fusion_type != MAG_FUSE_TYPE_NONE)
//                && !declination_was_valid) {
//                _mag_yaw_reset_req = true;
//            }

            // save the horizontal and vertical position uncertainty of the origin
            origin_unc_pos_horz = gps.eph;
            origin_unc_pos_vert = gps.epv;

//            _information_events.flags.gps_checks_passed = true;
//            ECL_INFO("GPS checks passed");

        } else if (!_ned_origin_initialised) {
            // a rough 2D fix is still sufficient to lookup declination
            if ((gps.fix_type >= 2) && (gps.eph < 1000)) {

                const bool declination_was_valid = PX4_ISFINITE(mag_declination);

                // If we have good GPS data set the origin's WGS-84 position to the last gps fix
                const double lat = gps.lat * 1.0e-7;
                const double lon = gps.lon * 1.0e-7;

                // set the magnetic field data returned by the geo library using the current GPS position
                mag_declination = get_mag_declination_radians((float)lat, (float)lon);
                mag_inclination = get_mag_inclination_radians((float)lat, (float)lon);
                mag_strength = get_mag_strength_gauss((float)lat, (float)lon);

//                // request mag yaw reset if there's a mag declination for the first time
//                if (_params.mag_fusion_type != MAG_FUSE_TYPE_NONE) {
//                    if (!declination_was_valid && PX4_ISFINITE(_mag_declination_gps)) {
//                        _mag_yaw_reset_req = true;
//                    }
//                }

                earth_rate_ned = calc_earth_rate_ned((float)math::radians(lat));
            }
        }

        // start collecting GPS if there is a 3D fix and the NED origin has been set
        return _ned_origin_initialised && (gps.fix_type >= 3);
    }

}

#endif //ECL_SENSOR_GPS_H
