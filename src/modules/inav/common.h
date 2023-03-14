//
// Created by Cain on 2023/3/7.
//

#ifndef ECL_COMMON_H
#define ECL_COMMON_H

#include <matrix/math.hpp>

#define NUM_GPS 2

namespace inav {
    using matrix::AxisAnglef;
    using matrix::Dcmf;
    using matrix::Eulerf;
    using matrix::Matrix3f;
    using matrix::Quatf;
    using matrix::Vector2f;
    using matrix::Vector3f;
    using matrix::wrap_pi;

    union activated_sensor_u {
        struct {
            bool gps: 1;
            bool ex: 1;
            bool flow: 1;
            bool baro: 1;
            bool range: 1;
            bool mag: 1;
            bool airspeed: 1;
        } sensor;
        uint32_t value;
    };

    // eskf参数
    struct Parameters {
        int32_t eskf_update_interval_us {1000};    ///< eskf更新周期

        int32_t sensor_interval_max_ms{10}; ///< 除imu外的传感器的最小采样周期

//        activated_sensor_u activated_sensor {0xffff};   ///< 激活了那些传感器

        // 零漂时间常数
        float gyro_bias_tau_inv {0.0f};     ///< 陀螺仪偏移的时间常数
        float acc_bias_tau_inv {0.003f};	///< 加速度计偏移的时间常数
        float mag_bias_tau_inv {0.003f};   ///< 磁力计偏移的时间常数
        float wind_tau_inv {0.1f};          ///< 风速的时间常数
//        float gyro_bias_tau_inv {0.0f};     ///< 陀螺仪偏移的时间常数
//        float acc_bias_tau_inv {0.0f};	///< 加速度计偏移的时间常数
//        float mag_bias_tau_inv {0.0f};   ///< 磁力计偏移的时间常数
//        float wind_tau_inv {0.0f};          ///< 风速的时间常数

        // 过程噪声
        float pos_proc_noise {1e-7f};   ///< 位置过程噪声 (m/s)
        float vel_proc_noise {1e-7f};   ///< 速度过程噪声 (m/s^2)
        float ang_axis_proc_noise {1e-7f};  ///< 轴角过程噪声 (rad/s)
        float gyro_bias_proc_noise {1.0e-3f};   ///< 角速度偏移过程噪声 (rad/s^2)
        float acc_bias_proc_noise {1.0e-2f};    ///< 加速度偏移过程噪声 (m/s^3)
        float grav_proc_noise {0.005f};         ///< 重力加速度过程噪声 (m/s^3)
        float mag_norm_proc_noise {1.0e-3f};    ///< 磁场强度过程噪声 (gauss/s)
        float mag_ang_proc_noise {1.0e-3f};     ///< 磁场倾角与偏角过程噪声 (rad/s)
        float mag_bias_proc_noise {1.0e-4f};    ///< 磁力计偏移过程噪声 (gauss/s)
        float wind_proc_noise {1.0e-1f};        ///< 风速过程噪声 (m/s^2)
        float baro_bias_proc_noise {0.005f};    ///< 气压计偏移过程噪声 (m/s)
        float terrain_proc_noise{5.0f};         ///< 地形高度过程噪声 (m/s)

        // 传感器噪声
        float gyro_noise {1.5e-2f};        ///< 角速度量测噪声 (rad/s)
        float gyro_bad_noise {0.2f};       ///< 工况差时的角速度量测噪声 (rad/s)
        float acc_noise {3.5e-1f};         ///< 加速度量测噪声 (m/s^2)
        float acc_bad_noise {4.9f};        ///< 工况差时的加速度量测噪声 (m/s^2)
        float gps_pos_horz_noise {0.5f};   ///< gps水平位置量测噪声 (m)
        float gps_pos_vert_noise {0.5f};   ///< gps垂直位置量测噪声 (m)
        float gps_vel_horz_noise {0.3f};   ///< gps水平速度量测噪声 (m/s)
        float gps_vel_vert_noise {0.3f};   ///< gps垂直速度量测噪声 (m/s)
        float ev_pos_horz_noise {0.02f};   ///< 外部视觉水平位置量测噪声 (m)
        float ev_pos_vert_noise {0.02f};   ///< 外部视觉垂直位置量测噪声 (m)
        float ev_vel_horz_noise {0.02f};   ///< 外部视觉水平速度量测噪声 (m/s)
        float ev_vel_vert_noise {0.02f};   ///< 外部视觉垂直速度量测噪声(m/s)
        float flow_noise {0.15f};          ///< 光流量测噪声 (rad/s)
        float baro_noise {2.0f};           ///< 气压计量测噪声 (m)
        float range_noise{0.1f};           ///< 测距仪量测噪声 (m)
        float mag_body_noise{5.0e-2f};     ///< 磁力计量测噪声 (Gauss)
        float mag_norm_noise{1.0e-3f};     ///< 磁场强度量测噪声 (Gauss)
        float mag_ang_noise{1.0e-3f};      ///< 磁场角度量测噪声 (rad)

        // 误差因子上限
        float gps_pos_horz_innov_gate {50.0f};   ///< gps水平位置误差因子上限
        float gps_pos_vert_innov_gate {50.0f};   ///< gps垂直位置误差因子上限
        float gps_vel_horz_innov_gate{50.0f};    ///< gps水平速度误差因子上限
        float gps_vel_vert_innov_gate{50.0f};    ///< gps垂直速度误差因子上限
        float ev_pos_horz_innov_gate {5.0f};    ///< 外部视觉水平位置误差因子上限
        float ev_pos_vert_innov_gate {5.0f};    ///< 外部视觉垂直位置误差因子上限
        float ev_vel_horz_innov_gate {3.0f};    ///< 外部视觉水平速度误差因子上限
        float ev_vel_vert_innov_gate {3.0f};    ///< 外部视觉垂直速度误差因子上限
        float flow_innov_gate {3.f};          ///< 光流误差因子上限
        float baro_innov_gate {5.0f};           ///< 气压计误差因子上限
        float range_innov_gate {5.f};          ///< 测距仪误差因子上限
        float mag_body_innov_gate {3.f};    ///< 磁力计误差因子上限
        float mag_norm_innov_gate {3.f};    ///< 磁场强度误差因子上限
        float mag_ang_innov_gate {3.f};     ///< 磁场角度误差因子上限

        /* 常值状态的初值 */
//        float g; {}
        float mag_norm {0.45f};			///< 磁场强度 (Gauss)
        float mag_inclination {0.0f}; 	///< 磁倾角 (degrees)
        float mag_declination {0.0f};	///< 磁偏角 (degrees)

        /* 协方差矩阵参数 */
        float std_init_pos_horz {0.5f};
        float std_init_pos_vert {1.f};
        float std_init_vel_horz {0.5f};
        float std_init_vel_vert {1.f};
        float std_init_ang {0.1f};
        float std_init_gyro_bias {0.1f};
        float std_init_acc_bias {0.2f};
        float std_init_grav {0.2f};
        float std_init_mag_norm {5.0e-2f};
        float std_init_mag_ang {5.0e-2f};
        float std_init_mag_bias {5.0e-2f};
        float std_init_wind {1.0f};

        float var_pos_max {1e6f};               ///< 位置的最大方差
        float var_vel_max {1e6f};               ///< 速度的最大方差
        float var_angle_max {1.f};              ///< 轴角的最大方差
        float var_gyro_bias_max {1.f};          ///< 陀螺仪偏移的最大方差
        float var_acc_bias_max {1.f};           ///< 加速度计偏移的最大方差
        float var_grav_max {1.f};               ///< 重力加速度的最大方差
        float var_mag_norm_max {1.f};           ///< 磁场强度的最大方差
        float var_mag_ang_max {1.f};            ///< 磁场角度的最大方差
        float var_mag_bias_max {1.f};           ///< 磁力计偏移的最大方差
        float var_wind_max {1.f};               ///< 风速的最大方差
        float var_pos_min {1e-12f};              ///< 位置的最小方差
        float var_vel_min {1e-6f};              ///< 速度的最小方差
        float var_angle_min {1e-6f};            ///< 轴角的最小方差
        float var_gyro_bias_min {1e-12f};        ///< 陀螺仪偏移的最小方差
        float var_acc_bias_min {1e-12f};         ///< 加速度计偏移的最小方差
        float var_grav_min {1e-12f};               ///< 重力加速度的最小方差
        float var_mag_norm_min {1e-12f};         ///< 磁场强度的最小方差
        float var_mag_ang_min {1e-12f};          ///< 磁场角度的最小方差
        float var_mag_bias_min {1e-12f};         ///< 磁力计偏移的最小方差
        float var_wind_min {1e-6f};             ///< 风速的最小方差

        /* 气压计偏移融合参数 */
        float alpha_terrain {0.005f};            ///< 地形高度低通滤波系数

        /* 测距仪融合参数 */
        float alpha_baro_bias {0.001f};          ///< 气压计偏移低通滤波系数
        float rng_gnd_clearance {0.1f};         ///< 测距仪最小距离
    };

    struct RunnerParameters {
        // GPS pre-flight check bit locations
        static constexpr uint16_t CHECK_GPS_NSATS {1};
        static constexpr uint16_t CHECK_GPS_PDOP {2};
        static constexpr uint16_t CHECK_GPS_HACC {4};
        static constexpr uint16_t CHECK_GPS_VACC {8};
        static constexpr uint16_t CHECK_GPS_SACC {16};
        static constexpr uint16_t CHECK_GPS_HDRIFT {32};
        static constexpr uint16_t CHECK_GPS_VDRIFT {64};
        static constexpr uint16_t CHECK_GPS_HSPD {128};
        static constexpr uint16_t CHECK_GPS_VSPD {256};

        /* 磁场融合相关的独立常数 */
        static constexpr uint8_t MAG_FUSE_TYPE_AUTO {0};	///< The selection of either heading or 3D magnetometer fusion will be automatic
        static constexpr uint8_t MAG_FUSE_TYPE_HEADING {1};	///< Simple yaw angle fusion will always be used. This is less accurate, but less affected by earth field distortions. It should not be used for pitch angles outside the range from -60 to +60 deg
        static constexpr uint8_t MAG_FUSE_TYPE_3D {2};	    ///< Magnetometer 3-axis fusion will always be used. This is more accurate, but more affected by localised earth field distortions
        static constexpr uint8_t MAG_FUSE_TYPE_UNUSED {3};	///< Not implemented
        static constexpr uint8_t MAG_FUSE_TYPE_INDOOR {4};	///< The same as option 0, but magnetometer or yaw fusion will not be used unless earth frame external aiding (GPS or External Vision) is being used. This prevents inconsistent magnetic fields associated with indoor operation degrading state estimates.
        static constexpr uint8_t MAG_FUSE_TYPE_NONE	{5};	///< Do not use magnetometer under any circumstance. Other sources of yaw may be used if selected via the EKF2_AID_MASK parameter.

        static constexpr uint8_t MAG_EARTH_FUSE_TYPE_NORM {0};
        static constexpr uint8_t MAG_EARTH_FUSE_TYPE_ANG {1};
        static constexpr uint8_t MAG_EARTH_FUSE_TYPE_3D {2};
        static constexpr uint8_t MAG_EARTH_TYPE_TYPE_NONE {3};

        /* 高度融合相关的独立常数 */
        static constexpr uint8_t HGT_SENSOR_AUTO {0};
        static constexpr uint8_t HGT_SENSOR_BARO {1};
        static constexpr uint8_t HGT_SENSOR_GPS {2};
        static constexpr uint8_t HGT_SENSOR_RANGE {3};
        static constexpr uint8_t HGT_SENSOR_EV {4};

        /* 传感器相对于imu最新的采样数据的的最大时间间隔(us) */
        static constexpr uint64_t GPS_MAX_INTERVAL {500000};    ///< GPS量测的最大时间间隔 (us), 默认为0.5s
        static constexpr uint64_t BARO_MAX_INTERVAL {200000};   ///< 气压计的最大时间间隔 (us), 默认为0.2s
        static constexpr uint64_t RANGE_MAX_INTERVAL {200000};  ///< 测距仪的最大时间间隔 (us), 默认为0.2s
        static constexpr uint64_t FLOW_MAX_INTERVAL {200000};   ///< 光流的最大时间间隔 (us), 默认为0.2s
        static constexpr uint64_t EV_MAX_INTERVAL {200000};     ///< 外部视觉的最大时间间隔 (us), 默认为0.2s
        static constexpr uint64_t MAG_MAX_INTERVAL {200000};    ///< 磁力计的最大时间间隔 (us), 默认为0.2s

        static constexpr uint64_t BADACC_PROBATION {1000000};   ///< 持续多少时间内没有检测出过垂直加速度异常, 才能认为是垂直加速度正常 (us), 默认为1s
        static constexpr uint64_t HGT_FUSE_TIMEOUT {5000000};   ///< 超过多少时间没进行高度融合, 则需要重置eskf的高度状态, 默认为5s
        static constexpr uint64_t MAG_FUSE_TIMEOUT {5000000};   ///< 超过多少时间没进行磁场数据融合, 则需要重新对齐磁场并重置磁场相关状态, 默认为5s

        static constexpr uint64_t FLOW_FUSE_TIMEOUT {7000000};
        static constexpr uint64_t HORZ_FUSE_TIMEOUT {7000000};

        static constexpr uint64_t MIN_GPS_HEALTH {1000000};

        /* 传感器延迟(ms) */
        float gps_delay_ms {0.f};
        float ev_delay_ms {0.f};
        float flow_delay_ms {0.f};
        float baro_delay_ms {0.f};

//        float gps_delay_ms {110.f};
//        float ev_delay_ms {175.0f};
//        float flow_delay_ms {5.0f};
//        float baro_delay_ms {0.0f};
        float range_delay_ms {5.0f};
        float mag_delay_ms {0.0f};
        float airspeed_delay_ms {100.0f};

        /* 传感器安装距离(m) */
        Vector3f imu_pos_body {};			///< imu在机体系的坐标
        Vector3f gps_pos_body[NUM_GPS] {{-0.2f, -0.15f, -0.02f}, {-0.2f, 0.15f, -0.02f}}; ///< GPS天线在机体系的坐标
        Vector3f ev_pos_body;			///< 外部视觉在机体系的坐标
        Vector3f flow_pos_body;			///< 光流在机体系的坐标
        Vector3f range_pos_body {-0.01f, 0.f, 0.02f};		///< 测距仪在机体系的坐标
        Vector3f baro_pos_body {0.02f, 0.f, 0.01f};			///< 气压计在机体系的坐标

        /* 磁场融合相关的独立量 */
        bool check_mag_strength {true};
        uint8_t mag_body_fusion_type {MAG_FUSE_TYPE_3D};
        uint8_t mag_earth_fusion_type {MAG_EARTH_FUSE_TYPE_3D};

        /* 高度融合相关的独立量 */
        uint8_t hgt_sensor_type {RunnerParameters::HGT_SENSOR_AUTO};

        /* 测距仪 */
        float range_cos_max_tilt{0.7071f};
        float rng_gnd_clearance {0.4f};

        /* 光流 */
        float flow_noise{0.15f};
        float flow_noise_qual_min{0.5f};
        float flow_max_rate {2.f * M_PI};
        int32_t flow_qual_min{1};

        /* GPS */
        uint16_t gps_check_mask{21};		///< bitmask used to control which GPS quality checks are used
        float req_hacc{5.0f};			///< maximum acceptable horizontal position error (m)
        float req_vacc{8.0f};			///< maximum acceptable vertical position error (m)
        float req_sacc{1.0f};			///< maximum acceptable speed error (m/s)
        int32_t req_nsats{6};			///< minimum acceptable satellite count
        float req_pdop{2.0f};			///< maximum acceptable position dilution of precision
        float req_hdrift{0.3f};			///< maximum acceptable horizontal drift speed (m/s)
        float req_vdrift{0.5f};			///< maximum acceptable vertical drift speed (m/s)

        float acc_bias_lim{0.4f};		///< maximum accel bias magnitude (m/sec**2)
        float acc_bias_inhibit_acc_lim_max{25.0f};	///< learning is disabled if the magnitude of the IMU acceleration vector is greater than this (m/sec**2)
        float acc_bias_inhibit_gyr_lim_max{3.0f};
        float acc_bias_inhibit_acc_lim_min{0.5f};
        float acc_bias_inhibit_gyr_lim_min{0.06f};
        float acc_bias_inhibit_tc{0.5f};

        float vert_innov_test_lim {3.0f};	///< Number of standard deviations allowed before the combined vertical velocity and position test is declared as failed
        float mag_anomalies_max_hgt {1.5f}; ///< 离地多少米可用认为是脱离了磁场异常区域 (m)
        float gyro_orth_mag_gate {0.25f};   ///< 角速度垂直于磁场的分量的模值大于该数值时, 才能激活磁力计偏移
    };

    enum class velocity_frame_t : uint8_t {
        LOCAL_FRAME_FRD,
        BODY_FRAME_FRD
    };

    struct BaseSample {
        uint64_t time_us{0};	///< 量测时间 (us)
    };

    struct GpsMessage : BaseSample {
        int32_t lat;		///< Latitude in 1E-7 degrees
        int32_t lon;		///< Longitude in 1E-7 degrees
        int32_t alt;		///< Altitude in 1E-3 meters (millimeters) above MSL
        float yaw;		///< yaw angle. NaN if not set (used for dual antenna GPS), (rad, [-PI, PI])
        float yaw_offset;	///< Heading/Yaw offset for dual antenna GPS - refer to description for GPS_YAW_OFFSET
        uint8_t fix_type;	///< 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic
        float eph;		///< GPS horizontal position accuracy in m
        float epv;		///< GPS vertical position accuracy in m
        float sacc;		///< GPS speed accuracy in m/s
        float vel_m_s;		///< GPS ground speed (m/sec)
        Vector3f vel_ned;	///< GPS ground speed NED
        bool vel_ned_valid;	///< GPS ground speed is valid
        uint8_t nsats;		///< number of satellites used
        float pdop;		///< position dilution of precision
    };

    struct OutputSample : BaseSample {
        Vector3f    pos;	    ///< 位置 (m)
        Vector3f    vel;	    ///< 速度 (m/s)
        Quatf  quat_nominal;	///< 归一化后的四元数
    };

    struct ImuSample : BaseSample {
        Vector3f    delta_ang {};		///< delta angle in body frame (integrated gyro measurements) (rad)
        Vector3f    delta_vel {};		///< delta velocity in body frame (integrated accelerometer measurements) (m/sec)
        float       delta_ang_dt {0.f};	///< delta angle integration period (sec)
        float       delta_vel_dt {0.f};	///< delta velocity integration period (sec)
        bool        delta_ang_clipping[3] {}; ///< true (per axis) if this sample contained any accelerometer clipping
        bool        delta_vel_clipping[3] {}; ///< true (per axis) if this sample contained any accelerometer clipping
    };

    struct GpsSample : BaseSample {
        Vector2f 	pos_horz;	///< 水平位置(相对于home点的北东向为正)
        float		hgt{};		///< 海拔高度(向上为正)
        Vector3f	vel;		///< gps速度(北东地)
        float       yaw{};      ///< 板卡融合出的航向
        float		hacc{};		///< 水平位置标准差
        float		vacc{};		///< 海拔高度标准差
        float		sacc{};		///< 速度标准差
        uint8_t     fix_type{}; ///< 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic
    };

    struct MagSample : BaseSample {
        Vector3f    mag;	///< 磁力计的量测值 (Gauss)
    };

    struct BaroSample : BaseSample {
        float       hgt{};	///< 气压计的量测高度 (m)
    };

    struct RangeSample : BaseSample {
        float       rng{};	    ///< 测距仪测地距离 (m)
        int8_t	    quality{};    ///< 信号质量的百分比 (0...100%), where 0 = 无效信号, 100 = 最高质量, and -1 = 质量未知.
    };

    struct AirspeedSample : BaseSample {
        float       true_airspeed{};	///< true airspeed measurement (m/sec)
        float       eas2tas{};		///< equivalent to true airspeed factor
    };

    struct FlowSample : BaseSample {
        Vector2f flow_xy_rad;	///< measured delta angle of the image about the X and Y body axes (rad), RH rotation is positive
        Vector3f gyro_xyz;	///< measured delta angle of the inertial frame about the body axes obtained from rate gyro measurements (rad), RH rotation is positive
        float    dt{};		///< amount of integration time (sec)
        uint8_t  quality{};	///< quality indicator between 0 and 255
    };

    struct ExtVisionSample : BaseSample {
        Vector3f pos;	///< XYZ position in external vision's local reference frame (m) - Z must be aligned with down axis
        Vector3f vel;	///< FRD velocity in reference frame defined in vel_frame variable (m/sec) - Z must be aligned with down axis
        Quatf quat;		///< quaternion defining rotation from body to earth frame
        Vector3f posVar;	///< XYZ position variances (m**2)
        Matrix3f velCov;	///< XYZ velocity covariances ((m/sec)**2)
        float angVar{};		///< angular heading variance (rad**2)
        velocity_frame_t vel_frame = velocity_frame_t::BODY_FRAME_FRD;
        uint8_t reset_counter{0};
    };

    struct StateSample : BaseSample {
        Vector3f    pos;                ///< 位置
        Vector3f    vel;                ///< 速度
        Quatf       quat_nominal;	    ///< 四元数(机体系到地球系)
        Vector3f    delta_ang_bias;	    ///< 角度增量偏移
        Vector3f    delta_vel_bias;	    ///< 速度增量偏移
        float 		grav{};	            ///< 重力加速度
        float       mag_norm{};           ///< 磁场强度
        Vector2f    mag_ang;			///< 磁场偏角
        Vector3f    mag_bias;	        ///< 磁力计量测偏移
        Vector2f    wind;	        	///< 风速
    };

    struct PreIntegralSample : BaseSample {
        float dt {0.f};
        Dcmf dr;
        Vector3f dv;
        Vector3f dp;
    };

    struct ErrorState {
        Vector3f pos;				///< 位置
        Vector3f vel;				///< 速度
        Vector3f ang;				///< 旋转向量
        Vector3f delta_ang_bias;	///< 角度增量偏移
        Vector3f delta_vel_bias;	///< 速度增量偏移
        float 	 grav{};				///< 重力加速度
        float 	 mag_norm{};			///< 磁场强度
        Vector2f mag_ang;			///< 磁场偏角
        Vector3f mag_bias;			///< 磁力计量测偏移
        Vector2f wind;				///< 风速
    };

    struct VarAccum {
        Vector3f pos;				///< 位置
        Vector3f vel;				///< 速度
        Vector3f ang;				///< 旋转向量
        Vector3f delta_ang_bias;	///< 角度增量偏移
        Vector3f delta_vel_bias;	///< 速度增量偏移
        float 	 grav{};				///< 重力加速度
        float 	 mag_norm{};			///< 磁场强度
        Vector2f mag_ang;			///< 磁场偏角
        Vector3f mag_bias;			///< 磁力计量测偏移
        Vector2f wind;				///< 风速
    };

    template<uint8_t N>
    struct FuseData {
        matrix::Vector<float, N> innov;         ///< 融合误差
        matrix::Vector<float, N> innov_var;     ///< 后验方差
        matrix::Vector<float, N> innov_ratio;   ///< 融合误差与后验标准差之比
        matrix::Vector<float, N> test_ratio;    ///< 融合误差的平差与后验协方差之比
    };

    // 最多支持8个GPS, 若GPS个数大于8, 需要修改uint64_t
    union innovation_fault_status_u {
        struct {
            bool reject_baro:         1;        ///< 0 拒绝气压计融合
            bool reject_range:        1;        ///< 1 拒绝测距仪融合
            bool reject_mag_body_x:   1;        ///< 2 拒绝磁力计x轴融合
            bool reject_mag_body_y:   1;        ///< 3 拒绝磁力计y轴融合
            bool reject_mag_body_z:   1;        ///< 4 拒绝磁力计z轴融合
            bool reject_mag_norm:     1;        ///< 5 拒绝磁场强度融合
            bool reject_mag_inc:      1;        ///< 6 拒绝磁场倾角融合
            bool reject_mag_dec:      1;        ///< 7 拒绝磁场偏角融合
            bool reject_airspeed:     1;        ///< 8 拒绝风速计融合
            bool reject_optflow_x:    1;        ///< 9 拒绝光流x轴融合
            bool reject_optflow_y:    1;        ///< 10 拒绝光流y轴融合
            bool reject_ev_pos_x:  1;           ///< 11 拒绝视觉x轴位置融合
            bool reject_ev_pos_y:  1;           ///< 12 拒绝视觉y轴位置融合
            bool reject_ev_pos_z:  1;           ///< 13 拒绝视觉z轴位置融合
            bool reject_ev_vel_x:  1;           ///< 14 拒绝视觉x轴速度融合
            bool reject_ev_vel_y:  1;           ///< 15 拒绝视觉y轴速度融合
            bool reject_ev_vel_z:  1;           ///< 15 拒绝视觉z轴速度融合
            uint8_t reject_gps_pos_x: NUM_GPS;  ///< 15 + N_GPS 拒绝gps的x轴水平位置融合
            uint8_t reject_gps_pos_y: NUM_GPS;  ///< 15 + 2 * N_GPS 拒绝gps的y轴水平位置融合
            uint8_t reject_gps_pos_z: NUM_GPS;  ///< 15 + 3 * N_GPS 拒绝gps的z轴水平位置融合
            uint8_t reject_gps_vel_x: NUM_GPS;  ///< 15 + 4 * N_GPS 拒绝gps的x轴水平速度融合
            uint8_t reject_gps_vel_y: NUM_GPS;  ///< 15 + 5 * N_GPS 拒绝gps的y轴水平速度融合
            uint8_t reject_gps_vel_z: NUM_GPS;  ///< 15 + 6 * N_GPS 拒绝gps的z轴水平速度融合
        } flags;
        uint64_t value;
    };

    union covariance_fault_status_u {
        struct {
            bool unhealthy_baro: 1;
            bool unhealthy_range: 1;
            bool unhealthy_mag_body_x: 1;
            bool unhealthy_mag_body_y: 1;
            bool unhealthy_mag_body_z: 1;
            bool unhealthy_mag_norm: 1;
            bool unhealthy_mag_inc: 1;
            bool unhealthy_mag_dec: 1;
            bool unhealthy_airspeed: 1;
            bool unhealthy_optflow_x: 1;
            bool unhealthy_optflow_y: 1;
            bool unhealthy_ev_pos_x: 1;
            bool unhealthy_ev_pos_y: 1;
            bool unhealthy_ev_pos_z: 1;
            bool unhealthy_ev_vel_x: 1;
            bool unhealthy_ev_vel_y: 1;
            bool unhealthy_ev_vel_z: 1;
            uint8_t unhealthy_gps_pos_x: NUM_GPS;
            uint8_t unhealthy_gps_pos_y: NUM_GPS;
            uint8_t unhealthy_gps_pos_z: NUM_GPS;
            uint8_t unhealthy_gps_vel_x: NUM_GPS;
            uint8_t unhealthy_gps_vel_y: NUM_GPS;
            uint8_t unhealthy_gps_vel_z: NUM_GPS;
        } flags;
        uint64_t value;
    };

    // 协方差
    union fault_status_u {
        struct {
            bool bad_mag_x: 1;	///< 0 - true if the fusion of the magnetometer X-axis has encountered a numerical error
            bool bad_mag_y: 1;	///< 1 - true if the fusion of the magnetometer Y-axis has encountered a numerical error
            bool bad_mag_z: 1;	///< 2 - true if the fusion of the magnetometer Z-axis has encountered a numerical error
            bool bad_hdg: 1;	///< 3 - true if the fusion of the heading angle has encountered a numerical error
            bool bad_mag_decl: 1;	///< 4 - true if the fusion of the magnetic declination has encountered a numerical error
            bool bad_airspeed: 1;	///< 5 - true if fusion of the airspeed has encountered a numerical error
            bool bad_sideslip: 1;	///< 6 - true if fusion of the synthetic sideslip constraint has encountered a numerical error
            bool bad_optflow_X: 1;	///< 7 - true if fusion of the optical flow X axis has encountered a numerical error
            bool bad_optflow_Y: 1;	///< 8 - true if fusion of the optical flow Y axis has encountered a numerical error
            bool bad_vel_N: 1;	///< 9 - true if fusion of the North velocity has encountered a numerical error
            bool bad_vel_E: 1;	///< 10 - true if fusion of the East velocity has encountered a numerical error
            bool bad_vel_D: 1;	///< 11 - true if fusion of the Down velocity has encountered a numerical error
            bool bad_pos_N: 1;	///< 12 - true if fusion of the North position has encountered a numerical error
            bool bad_pos_E: 1;	///< 13 - true if fusion of the East position has encountered a numerical error
            bool bad_pos_D: 1;	///< 14 - true if fusion of the Down position has encountered a numerical error
            bool bad_acc_bias: 1;	///< 15 - true if bad delta velocity bias estimates have been detected
            bool bad_acc_vertical: 1; ///< 16 - true if bad vertical accelerometer data has been detected
            bool bad_acc_clipping: 1; ///< 17 - true if delta velocity data contains clipping (asymmetric railing)
        } flags;
        uint32_t value;
    };

// publish the status of various GPS quality checks
    union gps_check_fail_status_u {
        struct {
            uint16_t fix    : 1; ///< 0 - true if the fix type is insufficient (no 3D solution)
            uint16_t nsats  : 1; ///< 1 - true if number of satellites used is insufficient
            uint16_t pdop   : 1; ///< 2 - true if position dilution of precision is insufficient
            uint16_t hacc   : 1; ///< 3 - true if reported horizontal accuracy is insufficient
            uint16_t vacc   : 1; ///< 4 - true if reported vertical accuracy is insufficient
            uint16_t sacc   : 1; ///< 5 - true if reported speed accuracy is insufficient
            uint16_t hdrift : 1; ///< 6 - true if horizontal drift is excessive (can only be used when stationary on ground)
            uint16_t vdrift : 1; ///< 7 - true if vertical drift is excessive (can only be used when stationary on ground)
            uint16_t hspeed : 1; ///< 8 - true if horizontal speed is excessive (can only be used when stationary on ground)
            uint16_t vspeed : 1; ///< 9 - true if vertical speed error is excessive
        } flags;
        uint16_t value;
    };

// bitmask containing filter control status
    union filter_control_status_u {
        struct {
            bool tilt_align  : 1; ///< 0 - true if the filter tilt alignment is complete
            bool yaw_align   : 1; ///< 1 - true if the filter yaw alignment is complete
            bool gps_vel     : 1; ///< 2 - true if GPS measurement fusion is intended
            bool gps_horz    : 1;
            bool opt_flow    : 1; ///< 3 - true if optical flow measurements fusion is intended
            bool acc_x_bias  : 1; ///< 4 - true if x轴加速度计偏移融合被激活
            bool acc_y_bias  : 1; ///< 5 - true if x轴加速度计偏移融合被激活
            bool acc_z_bias  : 1; ///< 6 - true if x轴加速度计偏移融合被激活
            bool grav        : 1; ///< 7 - true if 重力加速度融合被激活
            bool mag         : 1; ///< 所有与磁场有关的状态被激活
            bool mag_norm    : 1; ///< 8 - true if 磁场强度融合被激活
            bool mag_ang     : 1; ///< 9 - true if 磁偏角融合被激活
            bool mag_bias    : 1; ///< 10 - true if 磁力计偏移融合被激活
            bool in_air      : 1; ///< 11 - true when the vehicle is airborne
            bool wind        : 1; ///< 12 - true when wind velocity is being estimated
            bool baro_hgt    : 1; ///< 13 - true when baro height is being fused as a primary height reference
            bool rng_hgt     : 1; ///< 14 - true when range finder height is being fused as a primary height reference
            bool gps_hgt     : 1; ///< 15 - true when GPS height is being fused as a primary height reference
            bool ev_horz     : 1; ///< 16 - true when local position data fusion from external vision is intended
            bool none_hgt    : 1; ///< 不进行高度融合
            bool ev_yaw      : 1; ///< 17 - true when yaw data from external vision measurements fusion is intended
            bool ev_hgt      : 1; ///< 18 - true when height data from external vision measurements is being fused
            bool fuse_beta   : 1; ///< 19 - true when synthetic sideslip measurements are being fused
            bool mag_field_disturbed : 1; ///< 20 - true when the mag field does not match the expected strength
            bool fixed_wing  : 1; ///< 21 - true when the vehicle is operating as a fixed wing vehicle
            bool mag_fault   : 1; ///< 22 - true when the magnetometer has been declared faulty and is no longer being used
            bool fuse_aspd   : 1; ///< 23 - true when airspeed measurements are being fused
            bool gnd_effect  : 1; ///< 24 - true when protection from ground effect induced static pressure rise is _actived
            bool rng_stuck   : 1; ///< 25 - true when rng data wasn't ready for more than 10s and new rng values haven't changed enough
            bool gps_yaw     : 1; ///< 26 - true when yaw (not ground course) data fusion from a GPS receiver is intended
            bool mag_aligned : 1; ///< 磁场是否已经对准
            bool mag_aligned_in_flight   : 1; ///< 27 - true when the in-flight mag field alignment has been completed
            bool ev_vel      : 1; ///< 28 - true when local frame velocity data fusion from external vision measurements is intended
            bool synthetic_mag_z : 1; ///< 29 - true when we are using a synthesized measurement for the magnetometer Z component
            bool vehicle_at_rest : 1; ///< 30 - true when the vehicle is at rest
            bool gps_yaw_fault : 1; ///< 31 - true when the GNSS heading has been declared faulty and is no longer being used
            bool rng_fault : 1; ///< 32 - true when the range finder has been declared faulty and is no longer being used
            bool inertial_dead_reckoning : 1; ///< 33 - true if we are no longer fusing measurements that constrain horizontal velocity drift
            bool wind_dead_reckoning     : 1; ///< 34 - true if we are navigationg reliant on wind relative measurements
            bool heading_not_required : 1;
        } flags;
        uint64_t value;
    };

// Mavlink bitmask containing state of estimator solution
    union eskf_solution_status {
        struct {
            uint16_t attitude           : 1; ///< 0 - True if the attitude estimate is good
            uint16_t velocity_horiz     : 1; ///< 1 - True if the horizontal velocity estimate is good
            uint16_t velocity_vert      : 1; ///< 2 - True if the vertical velocity estimate is good
            uint16_t pos_horiz_rel      : 1; ///< 3 - True if the horizontal position (relative) estimate is good
            uint16_t pos_horiz_abs      : 1; ///< 4 - True if the horizontal position (absolute) estimate is good
            uint16_t pos_vert_abs       : 1; ///< 5 - True if the vertical position (absolute) estimate is good
            uint16_t pos_vert_agl       : 1; ///< 6 - True if the vertical position (above ground) estimate is good
            uint16_t const_pos_mode     : 1; ///< 7 - True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow)
            uint16_t pred_pos_horiz_rel : 1; ///< 8 - True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate
            uint16_t pred_pos_horiz_abs : 1; ///< 9 - True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
            uint16_t gps_glitch         : 1; ///< 10 - True if the EKF has detected a GPS glitch
            uint16_t accel_error        : 1; ///< 11 - True if the EKF has detected bad accelerometer data
        } flags;
        uint16_t value;
    };

    union terrain_fusion_status_u {
        struct {
            bool range_finder: 1;	///< 0 - true if we are fusing range finder data
            bool flow: 1;		///< 1 - true if we are fusing flow data
        } flags;
        uint8_t value;
    };

// define structure used to communicate information events
    union information_event_status_u {
        struct {
            bool gps_checks_passed		: 1; ///< 0 - true when gps quality checks are passing passed
            bool reset_vel_to_gps		: 1; ///< 1 - true when the velocity states are reset to the gps measurement
            bool reset_vel_to_flow		: 1; ///< 2 - true when the velocity states are reset using the optical flow measurement
            bool reset_vel_to_vision	: 1; ///< 3 - true when the velocity states are reset to the vision system measurement
            bool reset_vel_to_zero		: 1; ///< 4  - true when the velocity states are reset to zero
            bool reset_pos_to_last_known	: 1; ///< 5 - true when the position states are reset to the last known position
            bool reset_pos_to_gps		: 1; ///< 6 - true when the position states are reset to the gps measurement
            bool reset_pos_to_vision	: 1; ///< 7 - true when the position states are reset to the vision system measurement
            bool starting_gps_fusion	: 1; ///< 8 - true when the filter starts using gps measurements to correct the state estimates
            bool starting_vision_pos_fusion	: 1; ///< 9 - true when the filter starts using vision system position measurements to correct the state estimates
            bool starting_vision_vel_fusion	: 1; ///< 10 - true when the filter starts using vision system velocity measurements to correct the state estimates
            bool starting_vision_yaw_fusion	: 1; ///< 11 - true when the filter starts using vision system yaw  measurements to correct the state estimates
            bool yaw_aligned_to_imu_gps	: 1; ///< 12 - true when the filter resets the yaw to an estimate derived from IMU and GPS data
        } flags;
        uint32_t value;
    };

// define structure used to communicate information events
    union warning_event_status_u {
        struct {
            bool gps_quality_poor			: 1; ///< 0 - true when the gps is failing quality checks
            bool gps_fusion_timout			: 1; ///< 1 - true when the gps data has not been used to correct the state estimates for a significant time period
            bool gps_data_stopped			: 1; ///< 2 - true when the gps data has stopped for a significant time period
            bool gps_data_stopped_using_alternate	: 1; ///< 3 - true when the gps data has stopped for a significant time period but the filter is able to use other sources of data to maintain navigation
            bool height_sensor_timeout		: 1; ///< 4 - true when the height sensor has not been used to correct the state estimates for a significant time period
            bool stopping_navigation		: 1; ///< 5 - true when the filter has insufficient data to estimate velocity and position and is falling back to an attitude, height and height rate mode of operation
            bool invalid_accel_bias_cov_reset	: 1; ///< 6 - true when the filter has detected bad acceerometer bias state estimates and has reset the corresponding covariance matrix elements
            bool bad_yaw_using_gps_course		: 1; ///< 7 - true when the fiter has detected an invalid yaw esitmate and has reset the yaw angle to the GPS ground course
            bool stopping_mag_use			: 1; ///< 8 - true when the filter has detected bad magnetometer data and is stopping further use of the magnetomer data
            bool vision_data_stopped		: 1; ///< 9 - true when the vision system data has stopped for a significant time period
            bool emergency_yaw_reset_mag_stopped	: 1; ///< 10 - true when the filter has detected bad magnetometer data, has reset the yaw to anothter source of data and has stopped further use of the magnetomer data
            bool emergency_yaw_reset_gps_yaw_stopped: 1; ///< 11 - true when the filter has detected bad GNSS yaw data, has reset the yaw to anothter source of data and has stopped further use of the GNSS yaw data
        } flags;
        uint32_t value;
    };

#endif //ECL_COMMON_H
