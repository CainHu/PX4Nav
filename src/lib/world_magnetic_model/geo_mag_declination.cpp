//
// Created by Cain on 2023/3/1.
//

#include "geo_mag_declination.h"
#include "geo_magnetic_tables.hpp"

#include <mathlib/mathlib.h>

#include <math.h>
#include <stdint.h>

using math::constrain;

static constexpr unsigned get_lookup_table_index(float *val, float min, float max)
{
    /* for the rare case of hitting the bounds exactly
     * the rounding logic wouldn't fit, so enforce it.
     */

    /* limit to table bounds - required for maxima even when table spans full globe range */
    /* limit to (table bounds - 1) because bilinear interpolation requires checking (index + 1) */
    *val = constrain(*val, min, max - SAMPLING_RES);

    return static_cast<unsigned>((-(min) + *val) / SAMPLING_RES);
}

static constexpr float get_table_data(float lat, float lon, const int16_t table[LAT_DIM][LON_DIM])
{
    lat = math::constrain(lat, SAMPLING_MIN_LAT, SAMPLING_MAX_LAT);

    if (lon > SAMPLING_MAX_LON) {
        lon -= 360;
    }

    if (lon < SAMPLING_MIN_LON) {
        lon += 360;
    }

    /* round down to nearest sampling resolution */
    float min_lat = floorf(lat / SAMPLING_RES) * SAMPLING_RES;
    float min_lon = floorf(lon / SAMPLING_RES) * SAMPLING_RES;

    /* find index of nearest low sampling point */
    unsigned min_lat_index = get_lookup_table_index(&min_lat, SAMPLING_MIN_LAT, SAMPLING_MAX_LAT);
    unsigned min_lon_index = get_lookup_table_index(&min_lon, SAMPLING_MIN_LON, SAMPLING_MAX_LON);

    const float data_sw = table[min_lat_index][min_lon_index];
    const float data_se = table[min_lat_index][min_lon_index + 1];
    const float data_ne = table[min_lat_index + 1][min_lon_index + 1];
    const float data_nw = table[min_lat_index + 1][min_lon_index];

    /* perform bilinear interpolation on the four grid corners */
    const float lat_scale = constrain((lat - min_lat) / SAMPLING_RES, 0.f, 1.f);
    const float lon_scale = constrain((lon - min_lon) / SAMPLING_RES, 0.f, 1.f);

    const float data_min = lon_scale * (data_se - data_sw) + data_sw;
    const float data_max = lon_scale * (data_ne - data_nw) + data_nw;

    return lat_scale * (data_max - data_min) + data_min;
}

float get_mag_declination_radians(float lat, float lon)
{
    return get_table_data(lat, lon, declination_table) * 1e-4f; // declination table stored as 10^-4 radians
}

float get_mag_declination_degrees(float lat, float lon)
{
    return math::degrees(get_mag_declination_radians(lat, lon));
}

float get_mag_inclination_radians(float lat, float lon)
{
    return get_table_data(lat, lon, inclination_table) * 1e-4f; // inclination table stored as 10^-4 radians
}

float get_mag_inclination_degrees(float lat, float lon)
{
    return math::degrees(get_mag_inclination_radians(lat, lon));
}

float get_mag_strength_gauss(float lat, float lon)
{
    return get_table_data(lat, lon, strength_table) * 1e-4f; // strength table table stored as milli-Gauss * 10
}

float get_mag_strength_tesla(float lat, float lon)
{
    return get_mag_strength_gauss(lat, lon) * 1e-4f; // 1 Gauss == 0.0001 Tesla
}