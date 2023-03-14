//
// Created by Cain on 2022/12/26.
//

#include <iostream>
#include <fstream>
#include <ctime>
#include <random>
#include "matrix/math.hpp"
#include "geo/geo.h"
#include <modules/traj_sim/generator.h>
#include <modules/traj_sim/gen_utils.h>
#include <modules/traj_sim/gvar.h>
#include <modules/eskf/common.h>
#include <modules/eskf/geskf.h>
#include <modules/eskf/eskf_runner.h>
#include <modules/ahrs/ahrs.h>
#include <modules/inav/inav.h>
#include <lib/dsp/notch_filter.hpp>
#include <lib/control/loopshaping_auto_tuner.hpp>
#include <lib/dsp/sine_sweep.hpp>
#include <lib/dsp/butterworth.hpp>

using namespace std;
using namespace matrix;

#define BARO_BIAS 0.f
#define TERR 10.f

generator::Generator env;
vector<unsigned long> time_us_seq;

Vector2f generate_optflow_data(float range, const Matrix3f &R, const Vector3f &vel_nav, const Vector3f &rate, const Vector3f &offset) {
    const Vector3f vel_flow = R.transpose() * vel_nav + rate % offset;
    return {vel_flow(1) / range + rate(0), -vel_flow(0) / range + rate(1), };

}

float generate_range_data(const Matrix3f &R, const Vector3f &pos_nav, const Vector3f &offset) {
    const Vector3f pos_corr = pos_nav + R * offset;
    return -(pos_corr(2) - TERR) / R(2, 2);
}

float generate_baro_data(const Matrix3f &R, const Vector3f &pos_nav, const Vector3f &offset) {
    Vector3f pos_corr = pos_nav + R * offset;
    return -pos_corr(2) + BARO_BIAS;
}


void test1() {
    float _dt = 0.001f;

    float _P[24][24] = {};
    for (int i = 0; i < 24; ++i) {
        _P[i][i] = 1.f;
    }
    _P[9][9] = _P[10][10] = _P[11][11] = _dt * _dt;
    _P[12][12] = _P[13][13] = _P[14][14] = _dt * _dt;


    Dcmf _Rnb;
    _Rnb.setIdentity();

    Vector3f _delta_vel_corr_nav(_dt, _dt, _dt);
    _delta_vel_corr_nav = _Rnb * _delta_vel_corr_nav;

    float mR00 = -_Rnb(0, 0);
    float mR01 = -_Rnb(0, 1);
    float mR02 = -_Rnb(0, 2);
    float mR10 = -_Rnb(1, 0);
    float mR11 = -_Rnb(1, 1);
    float mR12 = -_Rnb(1, 2);
    float mR20 = -_Rnb(2, 0);
    float mR21 = -_Rnb(2, 1);
    float mR22 = -_Rnb(2, 2);

    // Δv_nav
    float dv0 = _delta_vel_corr_nav(0);
    float dv1 = _delta_vel_corr_nav(1);
    float dv2 = _delta_vel_corr_nav(2);

    // gamma
    const float gam_b_del_ang = 1.f;
    const float gam_b_del_vel = 1.f;
    const float gam_b_mag = 1.f;
    const float gam_w = 1.f;

    for (int i = 0; i < 10; ++i) {
        // 计算 F * P * F'
        float var[100] {
                _P[3][0] + _P[3][3] * _dt,
                _P[4][3] * _dt,
                _P[4][0] + var[1],
                _P[5][3] * _dt,
                _P[5][0] + var[3],
                _P[7][0] + _P[7][3] * _dt,
                _P[12][0] + _P[12][3] * _dt,
                _P[13][0] + _P[13][3] * _dt,
                _P[14][0] + _P[14][3] * _dt,
                _P[8][0] + _P[8][3] * _dt,
                _P[6][0] + _P[6][3] * _dt,
                _P[15][0] + _P[15][3] * _dt,
                _P[9][0] + _P[9][3] * _dt,
                _P[10][0] + _P[10][3] * _dt,
                _P[11][0] + _P[11][3] * _dt,
                _P[4][1] + _P[4][4] * _dt,
                _P[5][4] * _dt,
                _P[5][1] + var[16],
                _P[7][1] + _P[7][4] * _dt,
                _P[12][1] + _P[12][4] * _dt,
                _P[13][1] + _P[13][4] * _dt,
                _P[14][1] + _P[14][4] * _dt,
                _P[8][1] + _P[8][4] * _dt,
                _P[6][1] + _P[6][4] * _dt,
                _P[15][1] + _P[15][4] * _dt,
                _P[9][1] + _P[9][4] * _dt,
                _P[10][1] + _P[10][4] * _dt,
                _P[11][1] + _P[11][4] * _dt,
                _P[5][2] + _P[5][5] * _dt,
                _P[7][2] + _P[7][5] * _dt,
                _P[12][2] + _P[12][5] * _dt,
                _P[13][2] + _P[13][5] * _dt,
                _P[14][2] + _P[14][5] * _dt,
                _P[8][2] + _P[8][5] * _dt,
                _P[6][2] + _P[6][5] * _dt,
                _P[15][5] * _dt,
                _P[15][2] + var[35],
                _P[9][2] + _P[9][5] * _dt,
                _P[10][2] + _P[10][5] * _dt,
                _P[11][2] + _P[11][5] * _dt,
                _P[12][7] * mR00 + _P[13][7] * mR01 + _P[14][7] * mR02 + _P[7][3] + _P[7][7] * dv2 - _P[8][7] * dv1,
                _P[12][12] * mR00 + _P[12][3] + _P[12][7] * dv2 - _P[12][8] * dv1 + _P[13][12] * mR01 + _P[14][12] * mR02,
                _P[13][12] * mR00 + _P[13][13] * mR01 + _P[13][3] + _P[13][7] * dv2 - _P[13][8] * dv1 + _P[14][13] * mR02,
                _P[14][12] * mR00 + _P[14][13] * mR01 + _P[14][14] * mR02 + _P[14][3] + _P[14][7] * dv2 - _P[14][8] * dv1,
                _P[12][8] * mR00 + _P[13][8] * mR01 + _P[14][8] * mR02 + _P[8][3] + _P[8][7] * dv2 - _P[8][8] * dv1,
                _P[7][6] * dv2,
                _P[8][6] * dv1,
                _P[12][6] * mR00 + _P[13][6] * mR01 + _P[14][6] * mR02 + _P[6][3] + var[45] - var[46],
                _P[15][12] * mR00 + _P[15][13] * mR01 + _P[15][14] * mR02 + _P[15][3] + _P[15][7] * dv2 - _P[15][8] * dv1,
                _P[12][9] * mR00,
                _P[13][9] * mR01 + _P[14][9] * mR02 + _P[9][3] + _P[9][7] * dv2 - _P[9][8] * dv1 + var[49],
                _P[13][10] * mR01,
                _P[10][3] + _P[10][7] * dv2 - _P[10][8] * dv1 + _P[12][10] * mR00 + _P[14][10] * mR02 + var[51],
                _P[14][11] * mR02,
                _P[11][3] + _P[11][7] * dv2 - _P[11][8] * dv1 + _P[12][11] * mR00 + _P[13][11] * mR01 + var[53],
                _P[12][8] * mR10 + _P[13][8] * mR11 + _P[14][8] * mR12 + _P[8][4] - _P[8][6] * dv2 + _P[8][8] * dv0,
                _P[12][12] * mR10 + _P[12][4] - _P[12][6] * dv2 + _P[12][8] * dv0 + _P[13][12] * mR11 + _P[14][12] * mR12,
                _P[13][12] * mR10 + _P[13][13] * mR11 + _P[13][4] - _P[13][6] * dv2 + _P[13][8] * dv0 + _P[14][13] * mR12,
                _P[14][12] * mR10 + _P[14][13] * mR11 + _P[14][14] * mR12 + _P[14][4] - _P[14][6] * dv2 + _P[14][8] * dv0,
                _P[12][6] * mR10 + _P[13][6] * mR11 + _P[14][6] * mR12 + _P[6][4] - _P[6][6] * dv2 + _P[8][6] * dv0,
                _P[15][12] * mR10 + _P[15][13] * mR11 + _P[15][14] * mR12 + _P[15][4] - _P[15][6] * dv2 + _P[15][8] * dv0,
                _P[8][7] * dv0,
                _P[12][7] * mR10 + _P[13][7] * mR11 + _P[14][7] * mR12 + _P[7][4] - var[45] + var[61],
                _P[12][9] * mR10,
                _P[13][9] * mR11 + _P[14][9] * mR12 + _P[9][4] - _P[9][6] * dv2 + _P[9][8] * dv0 + var[63],
                _P[13][10] * mR11,
                _P[10][4] - _P[10][6] * dv2 + _P[10][8] * dv0 + _P[12][10] * mR10 + _P[14][10] * mR12 + var[65],
                _P[14][11] * mR12,
                _P[11][4] - _P[11][6] * dv2 + _P[11][8] * dv0 + _P[12][11] * mR10 + _P[13][11] * mR11 + var[67],
                _P[15][12] * mR20 + _P[15][13] * mR21 + _P[15][14] * mR22 + _P[15][15] * _dt + _P[15][5] + _P[15][6] * dv1 - _P[15][7] * dv0,
                _P[12][6] * mR20 + _P[13][6] * mR21 + _P[14][6] * mR22 + _P[15][6] * _dt + _P[6][5] + _P[6][6] * dv1 - _P[7][6] * dv0,
                _P[12][12] * mR20 + _P[12][5] + _P[12][6] * dv1 - _P[12][7] * dv0 + _P[13][12] * mR21 + _P[14][12] * mR22 + _P[15][12] * _dt,
                _P[13][12] * mR20 + _P[13][13] * mR21 + _P[13][5] + _P[13][6] * dv1 - _P[13][7] * dv0 + _P[14][13] * mR22 + _P[15][13] * _dt,
                _P[14][12] * mR20 + _P[14][13] * mR21 + _P[14][14] * mR22 + _P[14][5] + _P[14][6] * dv1 - _P[14][7] * dv0 + _P[15][14] * _dt,
                _P[12][7] * mR20 + _P[13][7] * mR21 + _P[14][7] * mR22 + _P[15][7] * _dt + _P[7][5] + _P[7][6] * dv1 - _P[7][7] * dv0,
                _P[12][9] * mR20,
                _P[13][9] * mR21 + _P[14][9] * mR22 + _P[15][9] * _dt + _P[9][5] + _P[9][6] * dv1 - _P[9][7] * dv0 + var[75],
                _P[13][10] * mR21,
                _P[10][5] + _P[10][6] * dv1 - _P[10][7] * dv0 + _P[12][10] * mR20 + _P[14][10] * mR22 + _P[15][10] * _dt + var[77],
                _P[14][11] * mR22,
                _P[11][5] + _P[11][6] * dv1 - _P[11][7] * dv0 + _P[12][11] * mR20 + _P[13][11] * mR21 + _P[15][11] * _dt + var[79],
                _P[10][9] * mR01 + _P[11][9] * mR02 + _P[9][6] + _P[9][9] * mR00,
                _P[10][10] * mR01 + _P[10][6] + _P[10][9] * mR00 + _P[11][10] * mR02,
                _P[11][10] * mR01 + _P[11][11] * mR02 + _P[11][6] + _P[11][9] * mR00,
                _P[10][9] * mR11 + _P[11][9] * mR12 + _P[9][7] + _P[9][9] * mR10,
                _P[10][10] * mR11 + _P[10][7] + _P[10][9] * mR10 + _P[11][10] * mR12,
                _P[11][10] * mR11 + _P[11][11] * mR12 + _P[11][7] + _P[11][9] * mR10,
                _P[10][9] * mR21 + _P[11][9] * mR22 + _P[9][8] + _P[9][9] * mR20,
                _P[10][10] * mR21 + _P[10][8] + _P[10][9] * mR20 + _P[11][10] * mR22,
                _P[11][10] * mR21 + _P[11][11] * mR22 + _P[11][8] + _P[11][9] * mR20,
                gam_b_del_ang*gam_b_del_ang,
                gam_b_del_ang*gam_b_del_vel,
                gam_b_del_ang*gam_b_mag,
                gam_b_del_ang*gam_w,
                gam_b_del_vel*gam_b_del_vel,
                gam_b_del_vel*gam_b_mag,
                gam_b_del_vel*gam_w,
                gam_b_mag*gam_b_mag,
                gam_b_mag*gam_w,
                gam_w*gam_w
        };

        _P[0][0] = _P[0][0] + _P[3][0] * _dt + _dt * var[0];
        _P[0][1] = _P[1][0] + _P[3][1] * _dt + _dt * var[2];
        _P[1][1] = _P[1][1] + _P[4][1] * _dt + _dt * var[15];
        _P[0][2] = _P[2][0] + _P[3][2] * _dt + _dt * var[4];
        _P[1][2] = _P[2][1] + _P[4][2] * _dt + _dt * var[17];
        _P[2][2] = _P[2][2] + _P[5][2] * _dt + _dt * var[28];
        _P[0][3] = -dv1 * var[9] + dv2 * var[5] + mR00 * var[6] + mR01 * var[7] + mR02 * var[8] + var[0];
        _P[1][3] = _P[3][1] - dv1 * var[22] + dv2 * var[18] + mR00 * var[19] + mR01 * var[20] + mR02 * var[21] + var[1];
        _P[2][3] = _P[3][2] - dv1 * var[33] + dv2 * var[29] + mR00 * var[30] + mR01 * var[31] + mR02 * var[32] + var[3];
        _P[3][3] = _P[12][3] * mR00 + _P[13][3] * mR01 + _P[14][3] * mR02 + _P[3][3] + _P[7][3] * dv2 - _P[8][3] * dv1 - dv1 * var[44] + dv2 * var[40] + mR00 * var[41] + mR01 * var[42] + mR02 * var[43];
        _P[0][4] = dv0 * var[9] - dv2 * var[10] + mR10 * var[6] + mR11 * var[7] + mR12 * var[8] + var[2];
        _P[1][4] = dv0 * var[22] - dv2 * var[23] + mR10 * var[19] + mR11 * var[20] + mR12 * var[21] + var[15];
        _P[2][4] = _P[4][2] + dv0 * var[33] - dv2 * var[34] + mR10 * var[30] + mR11 * var[31] + mR12 * var[32] + var[16];
        _P[3][4] = _P[12][4] * mR00 + _P[13][4] * mR01 + _P[14][4] * mR02 + _P[4][3] + _P[7][4] * dv2 - _P[8][4] * dv1 + dv0 * var[44] - dv2 * var[47] + mR10 * var[41] + mR11 * var[42] + mR12 * var[43];
        _P[4][4] = _P[12][4] * mR10 + _P[13][4] * mR11 + _P[14][4] * mR12 + _P[4][4] - _P[6][4] * dv2 + _P[8][4] * dv0 + dv0 * var[55] - dv2 * var[59] + mR10 * var[56] + mR11 * var[57] + mR12 * var[58];
        _P[0][5] = _dt * var[11] - dv0 * var[5] + dv1 * var[10] + mR20 * var[6] + mR21 * var[7] + mR22 * var[8] + var[4];
        _P[1][5] = _dt * var[24] - dv0 * var[18] + dv1 * var[23] + mR20 * var[19] + mR21 * var[20] + mR22 * var[21] + var[17];
        _P[2][5] = _dt * var[36] - dv0 * var[29] + dv1 * var[34] + mR20 * var[30] + mR21 * var[31] + mR22 * var[32] + var[28];
        _P[3][5] = _P[12][5] * mR00 + _P[13][5] * mR01 + _P[14][5] * mR02 + _P[5][3] + _P[7][5] * dv2 - _P[8][5] * dv1 + _dt * var[48] - dv0 * var[40] + dv1 * var[47] + mR20 * var[41] + mR21 * var[42] + mR22 * var[43];
        _P[4][5] = _P[12][5] * mR10 + _P[13][5] * mR11 + _P[14][5] * mR12 + _P[5][4] - _P[6][5] * dv2 + _P[8][5] * dv0 + _dt * var[60] - dv0 * var[62] + dv1 * var[59] + mR20 * var[56] + mR21 * var[57] + mR22 * var[58];
        _P[5][5] = _P[12][5] * mR20 + _P[13][5] * mR21 + _P[14][5] * mR22 + _P[5][5] + _P[6][5] * dv1 - _P[7][5] * dv0 + _dt * var[69] - dv0 * var[74] + dv1 * var[70] + mR20 * var[71] + mR21 * var[72] + mR22 * var[73] + var[35];
        _P[0][6] = mR00 * var[12] + mR01 * var[13] + mR02 * var[14] + var[10];
        _P[1][6] = mR00 * var[25] + mR01 * var[26] + mR02 * var[27] + var[23];
        _P[2][6] = mR00 * var[37] + mR01 * var[38] + mR02 * var[39] + var[34];
        _P[3][6] = mR00 * var[50] + mR01 * var[52] + mR02 * var[54] + var[47];
        _P[4][6] = mR00 * var[64] + mR01 * var[66] + mR02 * var[68] + var[59];
        _P[5][6] = mR00 * var[76] + mR01 * var[78] + mR02 * var[80] + var[70];
        _P[6][6] = _P[10][6] * mR01 + _P[11][6] * mR02 + _P[6][6] + _P[9][6] * mR00 + mR00 * var[81] + mR01 * var[82] + mR02 * var[83];
        _P[0][7] = mR10 * var[12] + mR11 * var[13] + mR12 * var[14] + var[5];
        _P[1][7] = mR10 * var[25] + mR11 * var[26] + mR12 * var[27] + var[18];
        _P[2][7] = mR10 * var[37] + mR11 * var[38] + mR12 * var[39] + var[29];
        _P[3][7] = mR10 * var[50] + mR11 * var[52] + mR12 * var[54] + var[40];
        _P[4][7] = mR10 * var[64] + mR11 * var[66] + mR12 * var[68] + var[62];
        _P[5][7] = mR10 * var[76] + mR11 * var[78] + mR12 * var[80] + var[74];
        _P[6][7] = _P[10][7] * mR01 + _P[11][7] * mR02 + _P[7][6] + _P[9][7] * mR00 + mR10 * var[81] + mR11 * var[82] + mR12 * var[83];
        _P[7][7] = _P[10][7] * mR11 + _P[11][7] * mR12 + _P[7][7] + _P[9][7] * mR10 + mR10 * var[84] + mR11 * var[85] + mR12 * var[86];
        _P[0][8] = mR20 * var[12] + mR21 * var[13] + mR22 * var[14] + var[9];
        _P[1][8] = mR20 * var[25] + mR21 * var[26] + mR22 * var[27] + var[22];
        _P[2][8] = mR20 * var[37] + mR21 * var[38] + mR22 * var[39] + var[33];
        _P[3][8] = mR20 * var[50] + mR21 * var[52] + mR22 * var[54] + var[44];
        _P[4][8] = mR20 * var[64] + mR21 * var[66] + mR22 * var[68] + var[55];
        _P[5][8] = _P[12][8] * mR20 + _P[13][8] * mR21 + _P[14][8] * mR22 + _P[15][8] * _dt + _P[8][5] + mR20 * var[76] + mR21 * var[78] + mR22 * var[80] + var[46] - var[61];
        _P[6][8] = _P[10][8] * mR01 + _P[11][8] * mR02 + _P[8][6] + _P[9][8] * mR00 + mR20 * var[81] + mR21 * var[82] + mR22 * var[83];
        _P[7][8] = _P[10][8] * mR11 + _P[11][8] * mR12 + _P[8][7] + _P[9][8] * mR10 + mR20 * var[84] + mR21 * var[85] + mR22 * var[86];
        _P[8][8] = _P[10][8] * mR21 + _P[11][8] * mR22 + _P[8][8] + _P[9][8] * mR20 + mR20 * var[87] + mR21 * var[88] + mR22 * var[89];
        _P[0][9] = gam_b_del_ang * var[12];
        _P[1][9] = gam_b_del_ang * var[25];
        _P[2][9] = gam_b_del_ang * var[37];
        _P[3][9] = gam_b_del_ang * var[50];
        _P[4][9] = gam_b_del_ang * var[64];
        _P[5][9] = gam_b_del_ang * var[76];
        _P[6][9] = gam_b_del_ang * var[81];
        _P[7][9] = gam_b_del_ang * var[84];
        _P[8][9] = gam_b_del_ang * var[87];
        _P[9][9] = _P[9][9] * var[90];
        _P[0][10] = gam_b_del_ang * var[13];
        _P[1][10] = gam_b_del_ang * var[26];
        _P[2][10] = gam_b_del_ang * var[38];
        _P[3][10] = gam_b_del_ang * var[52];
        _P[4][10] = gam_b_del_ang * var[66];
        _P[5][10] = gam_b_del_ang * var[78];
        _P[6][10] = gam_b_del_ang * var[82];
        _P[7][10] = gam_b_del_ang * var[85];
        _P[8][10] = gam_b_del_ang * var[88];
        _P[9][10] = _P[10][9] * var[90];
        _P[10][10] = _P[10][10] * var[90];
        _P[0][11] = gam_b_del_ang * var[14];
        _P[1][11] = gam_b_del_ang * var[27];
        _P[2][11] = gam_b_del_ang * var[39];
        _P[3][11] = gam_b_del_ang * var[54];
        _P[4][11] = gam_b_del_ang * var[68];
        _P[5][11] = gam_b_del_ang * var[80];
        _P[6][11] = gam_b_del_ang * var[83];
        _P[7][11] = gam_b_del_ang * var[86];
        _P[8][11] = gam_b_del_ang * var[89];
        _P[9][11] = _P[11][9] * var[90];
        _P[10][11] = _P[11][10] * var[90];
        _P[11][11] = _P[11][11] * var[90];

        _P[0][12] = gam_b_del_vel * var[6];
        _P[1][12] = gam_b_del_vel * var[19];
        _P[2][12] = gam_b_del_vel * var[30];
        _P[3][12] = gam_b_del_vel * var[41];
        _P[4][12] = gam_b_del_vel * var[56];
        _P[5][12] = gam_b_del_vel * var[71];
        _P[6][12] = gam_b_del_vel * (_P[12][10] * mR01 + _P[12][11] * mR02 + _P[12][6] + var[49]);
        _P[7][12] = gam_b_del_vel * (_P[12][10] * mR11 + _P[12][11] * mR12 + _P[12][7] + var[63]);
        _P[8][12] = gam_b_del_vel * (_P[12][10] * mR21 + _P[12][11] * mR22 + _P[12][8] + var[75]);
        _P[9][12] = _P[12][9] * var[91];
        _P[10][12] = _P[12][10] * var[91];
        _P[11][12] = _P[12][11] * var[91];
        _P[12][12] = _P[12][12] * var[94];

        _P[0][13] = gam_b_del_vel * var[7];
        _P[1][13] = gam_b_del_vel * var[20];
        _P[2][13] = gam_b_del_vel * var[31];
        _P[3][13] = gam_b_del_vel * var[42];
        _P[4][13] = gam_b_del_vel * var[57];
        _P[5][13] = gam_b_del_vel * var[72];
        _P[6][13] = gam_b_del_vel * (_P[13][11] * mR02 + _P[13][6] + _P[13][9] * mR00 + var[51]);
        _P[7][13] = gam_b_del_vel * (_P[13][11] * mR12 + _P[13][7] + _P[13][9] * mR10 + var[65]);
        _P[8][13] = gam_b_del_vel * (_P[13][11] * mR22 + _P[13][8] + _P[13][9] * mR20 + var[77]);
        _P[9][13] = _P[13][9] * var[91];
        _P[10][13] = _P[13][10] * var[91];
        _P[11][13] = _P[13][11] * var[91];
        _P[12][13] = _P[13][12] * var[94];
        _P[13][13] = _P[13][13] * var[94];

        _P[0][14] = gam_b_del_vel * var[8];
        _P[1][14] = gam_b_del_vel * var[21];
        _P[2][14] = gam_b_del_vel * var[32];
        _P[3][14] = gam_b_del_vel * var[43];
        _P[4][14] = gam_b_del_vel * var[58];
        _P[5][14] = gam_b_del_vel * var[73];
        _P[6][14] = gam_b_del_vel * (_P[14][10] * mR01 + _P[14][6] + _P[14][9] * mR00 + var[53]);
        _P[7][14] = gam_b_del_vel * (_P[14][10] * mR11 + _P[14][7] + _P[14][9] * mR10 + var[67]);
        _P[8][14] = gam_b_del_vel * (_P[14][10] * mR21 + _P[14][8] + _P[14][9] * mR20 + var[79]);
        _P[9][14] = _P[14][9] * var[91];
        _P[10][14] = _P[14][10] * var[91];
        _P[11][14] = _P[14][11] * var[91];
        _P[12][14] = _P[14][12] * var[94];
        _P[13][14] = _P[14][13] * var[94];
        _P[14][14] = _P[14][14] * var[94];

        _P[0][15] = var[11];
        _P[1][15] = var[24];
        _P[2][15] = var[36];
        _P[3][15] = var[48];
        _P[4][15] = var[60];
        _P[5][15] = var[69];
        _P[6][15] = _P[15][10] * mR01 + _P[15][11] * mR02 + _P[15][6] + _P[15][9] * mR00;
        _P[7][15] = _P[15][10] * mR11 + _P[15][11] * mR12 + _P[15][7] + _P[15][9] * mR10;
        _P[8][15] = _P[15][10] * mR21 + _P[15][11] * mR22 + _P[15][8] + _P[15][9] * mR20;
        _P[9][15] = _P[15][9] * gam_b_del_ang;
        _P[10][15] = _P[15][10] * gam_b_del_ang;
        _P[11][15] = _P[15][11] * gam_b_del_ang;
        _P[12][15] = _P[15][12] * gam_b_del_vel;
        _P[13][15] = _P[15][13] * gam_b_del_vel;
        _P[14][15] = _P[15][14] * gam_b_del_vel;
        _P[15][15] = _P[15][15];
    }

    for (int i = 0; i < 16; ++i) {
        for (int j = 0; j < 16; ++j) {
            cout << _P[i][j] << ", ";
        }
        cout << endl;
    }
}


void test2() {
    float _dt = 0.001f;

    float _cov[24][24] = {};
    for (int i = 0; i < 24; ++i) {
        _cov[i][i] = 1.f;
    }
//    _cov[9][9] = _cov[10][10] = _cov[11][11] = _dt * _dt;

    Dcmf _rot;
    _rot.setIdentity();

    const array<array<float, 3>, 3> rdt {
            {{-_rot(0, 0) * _dt, -_rot(0, 1) * _dt, -_rot(0, 2) * _dt},
                    {-_rot(1, 0) * _dt, -_rot(1, 1) * _dt, -_rot(1, 2) * _dt},
                    {-_rot(2, 0) * _dt, -_rot(2, 1) * _dt, -_rot(2, 2) * _dt}}
    };

    // (a - ba)
    const Vector3f a_corr(1.f, 1.f, 1.f);

    // -R * (a - ba) * dt
    const array<float, 3> v_b = {
            rdt[0][0] * a_corr(0) + rdt[0][1] * a_corr(1) + rdt[0][2] * a_corr(2),
            rdt[1][0] * a_corr(0) + rdt[1][1] * a_corr(1) + rdt[1][2] * a_corr(2),
            rdt[2][0] * a_corr(0) + rdt[2][1] * a_corr(1) + rdt[2][2] * a_corr(2)
    };

    // -(R * (a - ba))^ * dt
    const array<array<float, 3>, 3> x {
            {{0.f, -v_b[2], v_b[1]},
                    {v_b[2], 0.f, -v_b[0]},
                    {-v_b[1], v_b[0], 0.f}}
    };

    for (int i = 0; i < 10; ++i) {
        array<float, 88> var;

        var[0] = _cov[3][0] + _cov[3][3]*_dt;
        var[1] = _cov[4][3]*_dt;
        var[2] = _cov[4][0] + var[1];
        var[3] = _cov[5][3]*_dt;
        var[4] = _cov[5][0] + var[3];
        var[5] = _cov[12][0] + _cov[12][3]*_dt;
        var[6] = _cov[13][0] + _cov[13][3]*_dt;
        var[7] = _cov[14][0] + _cov[14][3]*_dt;
        var[8] = _cov[6][0] + _cov[6][3]*_dt;
        var[9] = _cov[7][0] + _cov[7][3]*_dt;
        var[10] = _cov[8][0] + _cov[8][3]*_dt;
        var[11] = _cov[15][0] + _cov[15][3]*_dt;
        var[12] = _cov[9][0] + _cov[9][3]*_dt;
        var[13] = _cov[10][0] + _cov[10][3]*_dt;
        var[14] = _cov[11][0] + _cov[11][3]*_dt;
        var[15] = _cov[4][1] + _cov[4][4]*_dt;
        var[16] = _cov[5][4]*_dt;
        var[17] = _cov[5][1] + var[16];
        var[18] = _cov[12][1] + _cov[12][4]*_dt;
        var[19] = _cov[13][1] + _cov[13][4]*_dt;
        var[20] = _cov[14][1] + _cov[14][4]*_dt;
        var[21] = _cov[6][1] + _cov[6][4]*_dt;
        var[22] = _cov[7][1] + _cov[7][4]*_dt;
        var[23] = _cov[8][1] + _cov[8][4]*_dt;
        var[24] = _cov[15][1] + _cov[15][4]*_dt;
        var[25] = _cov[9][1] + _cov[9][4]*_dt;
        var[26] = _cov[10][1] + _cov[10][4]*_dt;
        var[27] = _cov[11][1] + _cov[11][4]*_dt;
        var[28] = _cov[5][2] + _cov[5][5]*_dt;
        var[29] = _cov[12][2] + _cov[12][5]*_dt;
        var[30] = _cov[13][2] + _cov[13][5]*_dt;
        var[31] = _cov[14][2] + _cov[14][5]*_dt;
        var[32] = _cov[6][2] + _cov[6][5]*_dt;
        var[33] = _cov[7][2] + _cov[7][5]*_dt;
        var[34] = _cov[8][2] + _cov[8][5]*_dt;
        var[35] = _cov[15][5]*_dt;
        var[36] = _cov[15][2] + var[35];
        var[37] = _cov[9][2] + _cov[9][5]*_dt;
        var[38] = _cov[10][2] + _cov[10][5]*_dt;
        var[39] = _cov[11][2] + _cov[11][5]*_dt;
        var[40] = _cov[12][12]*rdt[0][0] + _cov[12][3] + _cov[12][6]*x[0][0] + _cov[12][7]*x[0][1] + _cov[12][8]*x[0][2] + _cov[13][12]*rdt[0][1] + _cov[14][12]*rdt[0][2];
        var[41] = _cov[13][12]*rdt[0][0] + _cov[13][13]*rdt[0][1] + _cov[13][3] + _cov[13][6]*x[0][0] + _cov[13][7]*x[0][1] + _cov[13][8]*x[0][2] + _cov[14][13]*rdt[0][2];
        var[42] = _cov[14][12]*rdt[0][0] + _cov[14][13]*rdt[0][1] + _cov[14][14]*rdt[0][2] + _cov[14][3] + _cov[14][6]*x[0][0] + _cov[14][7]*x[0][1] + _cov[14][8]*x[0][2];
        var[43] = _cov[12][6]*rdt[0][0] + _cov[13][6]*rdt[0][1] + _cov[14][6]*rdt[0][2] + _cov[6][3] + _cov[6][6]*x[0][0] + _cov[7][6]*x[0][1] + _cov[8][6]*x[0][2];
        var[44] = _cov[12][7]*rdt[0][0] + _cov[13][7]*rdt[0][1] + _cov[14][7]*rdt[0][2] + _cov[7][3] + _cov[7][6]*x[0][0] + _cov[7][7]*x[0][1] + _cov[8][7]*x[0][2];
        var[45] = _cov[12][8]*rdt[0][0] + _cov[13][8]*rdt[0][1] + _cov[14][8]*rdt[0][2] + _cov[8][3] + _cov[8][6]*x[0][0] + _cov[8][7]*x[0][1] + _cov[8][8]*x[0][2];
        var[46] = _cov[15][12]*rdt[0][0] + _cov[15][13]*rdt[0][1] + _cov[15][14]*rdt[0][2] + _cov[15][3] + _cov[15][6]*x[0][0] + _cov[15][7]*x[0][1] + _cov[15][8]*x[0][2];
        var[47] = _cov[12][9]*rdt[0][0];
        var[48] = _cov[13][9]*rdt[0][1] + _cov[14][9]*rdt[0][2] + _cov[9][3] + _cov[9][6]*x[0][0] + _cov[9][7]*x[0][1] + _cov[9][8]*x[0][2] + var[47];
        var[49] = _cov[13][10]*rdt[0][1];
        var[50] = _cov[10][3] + _cov[10][6]*x[0][0] + _cov[10][7]*x[0][1] + _cov[10][8]*x[0][2] + _cov[12][10]*rdt[0][0] + _cov[14][10]*rdt[0][2] + var[49];
        var[51] = _cov[14][11]*rdt[0][2];
        var[52] = _cov[11][3] + _cov[11][6]*x[0][0] + _cov[11][7]*x[0][1] + _cov[11][8]*x[0][2] + _cov[12][11]*rdt[0][0] + _cov[13][11]*rdt[0][1] + var[51];
        var[53] = _cov[12][12]*rdt[1][0] + _cov[12][4] + _cov[12][6]*x[1][0] + _cov[12][7]*x[1][1] + _cov[12][8]*x[1][2] + _cov[13][12]*rdt[1][1] + _cov[14][12]*rdt[1][2];
        var[54] = _cov[13][12]*rdt[1][0] + _cov[13][13]*rdt[1][1] + _cov[13][4] + _cov[13][6]*x[1][0] + _cov[13][7]*x[1][1] + _cov[13][8]*x[1][2] + _cov[14][13]*rdt[1][2];
        var[55] = _cov[14][12]*rdt[1][0] + _cov[14][13]*rdt[1][1] + _cov[14][14]*rdt[1][2] + _cov[14][4] + _cov[14][6]*x[1][0] + _cov[14][7]*x[1][1] + _cov[14][8]*x[1][2];
        var[56] = _cov[12][6]*rdt[1][0] + _cov[13][6]*rdt[1][1] + _cov[14][6]*rdt[1][2] + _cov[6][4] + _cov[6][6]*x[1][0] + _cov[7][6]*x[1][1] + _cov[8][6]*x[1][2];
        var[57] = _cov[12][7]*rdt[1][0] + _cov[13][7]*rdt[1][1] + _cov[14][7]*rdt[1][2] + _cov[7][4] + _cov[7][6]*x[1][0] + _cov[7][7]*x[1][1] + _cov[8][7]*x[1][2];
        var[58] = _cov[12][8]*rdt[1][0] + _cov[13][8]*rdt[1][1] + _cov[14][8]*rdt[1][2] + _cov[8][4] + _cov[8][6]*x[1][0] + _cov[8][7]*x[1][1] + _cov[8][8]*x[1][2];
        var[59] = _cov[15][12]*rdt[1][0] + _cov[15][13]*rdt[1][1] + _cov[15][14]*rdt[1][2] + _cov[15][4] + _cov[15][6]*x[1][0] + _cov[15][7]*x[1][1] + _cov[15][8]*x[1][2];
        var[60] = _cov[12][9]*rdt[1][0];
        var[61] = _cov[13][9]*rdt[1][1] + _cov[14][9]*rdt[1][2] + _cov[9][4] + _cov[9][6]*x[1][0] + _cov[9][7]*x[1][1] + _cov[9][8]*x[1][2] + var[60];
        var[62] = _cov[13][10]*rdt[1][1];
        var[63] = _cov[10][4] + _cov[10][6]*x[1][0] + _cov[10][7]*x[1][1] + _cov[10][8]*x[1][2] + _cov[12][10]*rdt[1][0] + _cov[14][10]*rdt[1][2] + var[62];
        var[64] = _cov[14][11]*rdt[1][2];
        var[65] = _cov[11][4] + _cov[11][6]*x[1][0] + _cov[11][7]*x[1][1] + _cov[11][8]*x[1][2] + _cov[12][11]*rdt[1][0] + _cov[13][11]*rdt[1][1] + var[64];
        var[66] = _cov[15][12]*rdt[2][0] + _cov[15][13]*rdt[2][1] + _cov[15][14]*rdt[2][2] + _cov[15][15]*_dt + _cov[15][5] + _cov[15][6]*x[2][0] + _cov[15][7]*x[2][1] + _cov[15][8]*x[2][2];
        var[67] = _cov[12][12]*rdt[2][0] + _cov[12][5] + _cov[12][6]*x[2][0] + _cov[12][7]*x[2][1] + _cov[12][8]*x[2][2] + _cov[13][12]*rdt[2][1] + _cov[14][12]*rdt[2][2] + _cov[15][12]*_dt;
        var[68] = _cov[13][12]*rdt[2][0] + _cov[13][13]*rdt[2][1] + _cov[13][5] + _cov[13][6]*x[2][0] + _cov[13][7]*x[2][1] + _cov[13][8]*x[2][2] + _cov[14][13]*rdt[2][2] + _cov[15][13]*_dt;
        var[69] = _cov[14][12]*rdt[2][0] + _cov[14][13]*rdt[2][1] + _cov[14][14]*rdt[2][2] + _cov[14][5] + _cov[14][6]*x[2][0] + _cov[14][7]*x[2][1] + _cov[14][8]*x[2][2] + _cov[15][14]*_dt;
        var[70] = _cov[12][6]*rdt[2][0] + _cov[13][6]*rdt[2][1] + _cov[14][6]*rdt[2][2] + _cov[15][6]*_dt + _cov[6][5] + _cov[6][6]*x[2][0] + _cov[7][6]*x[2][1] + _cov[8][6]*x[2][2];
        var[71] = _cov[12][7]*rdt[2][0] + _cov[13][7]*rdt[2][1] + _cov[14][7]*rdt[2][2] + _cov[15][7]*_dt + _cov[7][5] + _cov[7][6]*x[2][0] + _cov[7][7]*x[2][1] + _cov[8][7]*x[2][2];
        var[72] = _cov[12][8]*rdt[2][0] + _cov[13][8]*rdt[2][1] + _cov[14][8]*rdt[2][2] + _cov[15][8]*_dt + _cov[8][5] + _cov[8][6]*x[2][0] + _cov[8][7]*x[2][1] + _cov[8][8]*x[2][2];
        var[73] = _cov[12][9]*rdt[2][0];
        var[74] = _cov[13][9]*rdt[2][1] + _cov[14][9]*rdt[2][2] + _cov[15][9]*_dt + _cov[9][5] + _cov[9][6]*x[2][0] + _cov[9][7]*x[2][1] + _cov[9][8]*x[2][2] + var[73];
        var[75] = _cov[13][10]*rdt[2][1];
        var[76] = _cov[10][5] + _cov[10][6]*x[2][0] + _cov[10][7]*x[2][1] + _cov[10][8]*x[2][2] + _cov[12][10]*rdt[2][0] + _cov[14][10]*rdt[2][2] + _cov[15][10]*_dt + var[75];
        var[77] = _cov[14][11]*rdt[2][2];
        var[78] = _cov[11][5] + _cov[11][6]*x[2][0] + _cov[11][7]*x[2][1] + _cov[11][8]*x[2][2] + _cov[12][11]*rdt[2][0] + _cov[13][11]*rdt[2][1] + _cov[15][11]*_dt + var[77];
        var[79] = _cov[10][9]*rdt[0][1] + _cov[11][9]*rdt[0][2] + _cov[9][6] + _cov[9][9]*rdt[0][0];
        var[80] = _cov[10][10]*rdt[0][1] + _cov[10][6] + _cov[10][9]*rdt[0][0] + _cov[11][10]*rdt[0][2];
        var[81] = _cov[11][10]*rdt[0][1] + _cov[11][11]*rdt[0][2] + _cov[11][6] + _cov[11][9]*rdt[0][0];
        var[82] = _cov[10][9]*rdt[1][1] + _cov[11][9]*rdt[1][2] + _cov[9][7] + _cov[9][9]*rdt[1][0];
        var[83] = _cov[10][10]*rdt[1][1] + _cov[10][7] + _cov[10][9]*rdt[1][0] + _cov[11][10]*rdt[1][2];
        var[84] = _cov[11][10]*rdt[1][1] + _cov[11][11]*rdt[1][2] + _cov[11][7] + _cov[11][9]*rdt[1][0];
        var[85] = _cov[10][9]*rdt[2][1] + _cov[11][9]*rdt[2][2] + _cov[9][8] + _cov[9][9]*rdt[2][0];
        var[86] = _cov[10][10]*rdt[2][1] + _cov[10][8] + _cov[10][9]*rdt[2][0] + _cov[11][10]*rdt[2][2];
        var[87] = _cov[11][10]*rdt[2][1] + _cov[11][11]*rdt[2][2] + _cov[11][8] + _cov[11][9]*rdt[2][0];

        _cov[0][0] = _cov[0][0] + _cov[3][0]*_dt + _dt*var[0];
        _cov[0][1] = _cov[1][0] + _cov[3][1]*_dt + _dt*var[2];
        _cov[1][1] = _cov[1][1] + _cov[4][1]*_dt + _dt*var[15];
        _cov[0][2] = _cov[2][0] + _cov[3][2]*_dt + _dt*var[4];
        _cov[1][2] = _cov[2][1] + _cov[4][2]*_dt + _dt*var[17];
        _cov[2][2] = _cov[2][2] + _cov[5][2]*_dt + _dt*var[28];
        _cov[0][3] = rdt[0][0]*var[5] + rdt[0][1]*var[6] + rdt[0][2]*var[7] + var[0] + var[10]*x[0][2] + var[8]*x[0][0] + var[9]*x[0][1];
        _cov[1][3] = _cov[3][1] + rdt[0][0]*var[18] + rdt[0][1]*var[19] + rdt[0][2]*var[20] + var[1] + var[21]*x[0][0] + var[22]*x[0][1] + var[23]*x[0][2];
        _cov[2][3] = _cov[3][2] + rdt[0][0]*var[29] + rdt[0][1]*var[30] + rdt[0][2]*var[31] + var[32]*x[0][0] + var[33]*x[0][1] + var[34]*x[0][2] + var[3];
        _cov[3][3] = _cov[12][3]*rdt[0][0] + _cov[13][3]*rdt[0][1] + _cov[14][3]*rdt[0][2] + _cov[3][3] + _cov[6][3]*x[0][0] + _cov[7][3]*x[0][1] + _cov[8][3]*x[0][2] + rdt[0][0]*var[40] + rdt[0][1]*var[41] + rdt[0][2]*var[42] + var[43]*x[0][0] + var[44]*x[0][1] + var[45]*x[0][2];
        _cov[0][4] = rdt[1][0]*var[5] + rdt[1][1]*var[6] + rdt[1][2]*var[7] + var[10]*x[1][2] + var[2] + var[8]*x[1][0] + var[9]*x[1][1];
        _cov[1][4] = rdt[1][0]*var[18] + rdt[1][1]*var[19] + rdt[1][2]*var[20] + var[15] + var[21]*x[1][0] + var[22]*x[1][1] + var[23]*x[1][2];
        _cov[2][4] = _cov[4][2] + rdt[1][0]*var[29] + rdt[1][1]*var[30] + rdt[1][2]*var[31] + var[16] + var[32]*x[1][0] + var[33]*x[1][1] + var[34]*x[1][2];
        _cov[3][4] = _cov[12][4]*rdt[0][0] + _cov[13][4]*rdt[0][1] + _cov[14][4]*rdt[0][2] + _cov[4][3] + _cov[6][4]*x[0][0] + _cov[7][4]*x[0][1] + _cov[8][4]*x[0][2] + rdt[1][0]*var[40] + rdt[1][1]*var[41] + rdt[1][2]*var[42] + var[43]*x[1][0] + var[44]*x[1][1] + var[45]*x[1][2];
        _cov[4][4] = _cov[12][4]*rdt[1][0] + _cov[13][4]*rdt[1][1] + _cov[14][4]*rdt[1][2] + _cov[4][4] + _cov[6][4]*x[1][0] + _cov[7][4]*x[1][1] + _cov[8][4]*x[1][2] + rdt[1][0]*var[53] + rdt[1][1]*var[54] + rdt[1][2]*var[55] + var[56]*x[1][0] + var[57]*x[1][1] + var[58]*x[1][2];
        _cov[0][5] = _dt*var[11] + rdt[2][0]*var[5] + rdt[2][1]*var[6] + rdt[2][2]*var[7] + var[10]*x[2][2] + var[4] + var[8]*x[2][0] + var[9]*x[2][1];
        _cov[1][5] = _dt*var[24] + rdt[2][0]*var[18] + rdt[2][1]*var[19] + rdt[2][2]*var[20] + var[17] + var[21]*x[2][0] + var[22]*x[2][1] + var[23]*x[2][2];
        _cov[2][5] = _dt*var[36] + rdt[2][0]*var[29] + rdt[2][1]*var[30] + rdt[2][2]*var[31] + var[28] + var[32]*x[2][0] + var[33]*x[2][1] + var[34]*x[2][2];
        _cov[3][5] = _cov[12][5]*rdt[0][0] + _cov[13][5]*rdt[0][1] + _cov[14][5]*rdt[0][2] + _cov[5][3] + _cov[6][5]*x[0][0] + _cov[7][5]*x[0][1] + _cov[8][5]*x[0][2] + _dt*var[46] + rdt[2][0]*var[40] + rdt[2][1]*var[41] + rdt[2][2]*var[42] + var[43]*x[2][0] + var[44]*x[2][1] + var[45]*x[2][2];
        _cov[4][5] = _cov[12][5]*rdt[1][0] + _cov[13][5]*rdt[1][1] + _cov[14][5]*rdt[1][2] + _cov[5][4] + _cov[6][5]*x[1][0] + _cov[7][5]*x[1][1] + _cov[8][5]*x[1][2] + _dt*var[59] + rdt[2][0]*var[53] + rdt[2][1]*var[54] + rdt[2][2]*var[55] + var[56]*x[2][0] + var[57]*x[2][1] + var[58]*x[2][2];
        _cov[5][5] = _cov[12][5]*rdt[2][0] + _cov[13][5]*rdt[2][1] + _cov[14][5]*rdt[2][2] + _cov[5][5] + _cov[6][5]*x[2][0] + _cov[7][5]*x[2][1] + _cov[8][5]*x[2][2] + _dt*var[66] + rdt[2][0]*var[67] + rdt[2][1]*var[68] + rdt[2][2]*var[69] + var[35] + var[70]*x[2][0] + var[71]*x[2][1] + var[72]*x[2][2];
        _cov[0][6] = rdt[0][0]*var[12] + rdt[0][1]*var[13] + rdt[0][2]*var[14] + var[8];
        _cov[1][6] = rdt[0][0]*var[25] + rdt[0][1]*var[26] + rdt[0][2]*var[27] + var[21];
        _cov[2][6] = rdt[0][0]*var[37] + rdt[0][1]*var[38] + rdt[0][2]*var[39] + var[32];
        _cov[3][6] = rdt[0][0]*var[48] + rdt[0][1]*var[50] + rdt[0][2]*var[52] + var[43];
        _cov[4][6] = rdt[0][0]*var[61] + rdt[0][1]*var[63] + rdt[0][2]*var[65] + var[56];
        _cov[5][6] = rdt[0][0]*var[74] + rdt[0][1]*var[76] + rdt[0][2]*var[78] + var[70];
        _cov[6][6] = _cov[10][6]*rdt[0][1] + _cov[11][6]*rdt[0][2] + _cov[6][6] + _cov[9][6]*rdt[0][0] + rdt[0][0]*var[79] + rdt[0][1]*var[80] + rdt[0][2]*var[81];
        _cov[0][7] = rdt[1][0]*var[12] + rdt[1][1]*var[13] + rdt[1][2]*var[14] + var[9];
        _cov[1][7] = rdt[1][0]*var[25] + rdt[1][1]*var[26] + rdt[1][2]*var[27] + var[22];
        _cov[2][7] = rdt[1][0]*var[37] + rdt[1][1]*var[38] + rdt[1][2]*var[39] + var[33];
        _cov[3][7] = rdt[1][0]*var[48] + rdt[1][1]*var[50] + rdt[1][2]*var[52] + var[44];
        _cov[4][7] = rdt[1][0]*var[61] + rdt[1][1]*var[63] + rdt[1][2]*var[65] + var[57];
        _cov[5][7] = rdt[1][0]*var[74] + rdt[1][1]*var[76] + rdt[1][2]*var[78] + var[71];
        _cov[6][7] = _cov[10][7]*rdt[0][1] + _cov[11][7]*rdt[0][2] + _cov[7][6] + _cov[9][7]*rdt[0][0] + rdt[1][0]*var[79] + rdt[1][1]*var[80] + rdt[1][2]*var[81];
        _cov[7][7] = _cov[10][7]*rdt[1][1] + _cov[11][7]*rdt[1][2] + _cov[7][7] + _cov[9][7]*rdt[1][0] + rdt[1][0]*var[82] + rdt[1][1]*var[83] + rdt[1][2]*var[84];
        _cov[0][8] = rdt[2][0]*var[12] + rdt[2][1]*var[13] + rdt[2][2]*var[14] + var[10];
        _cov[1][8] = rdt[2][0]*var[25] + rdt[2][1]*var[26] + rdt[2][2]*var[27] + var[23];
        _cov[2][8] = rdt[2][0]*var[37] + rdt[2][1]*var[38] + rdt[2][2]*var[39] + var[34];
        _cov[3][8] = rdt[2][0]*var[48] + rdt[2][1]*var[50] + rdt[2][2]*var[52] + var[45];
        _cov[4][8] = rdt[2][0]*var[61] + rdt[2][1]*var[63] + rdt[2][2]*var[65] + var[58];
        _cov[5][8] = rdt[2][0]*var[74] + rdt[2][1]*var[76] + rdt[2][2]*var[78] + var[72];
        _cov[6][8] = _cov[10][8]*rdt[0][1] + _cov[11][8]*rdt[0][2] + _cov[8][6] + _cov[9][8]*rdt[0][0] + rdt[2][0]*var[79] + rdt[2][1]*var[80] + rdt[2][2]*var[81];
        _cov[7][8] = _cov[10][8]*rdt[1][1] + _cov[11][8]*rdt[1][2] + _cov[8][7] + _cov[9][8]*rdt[1][0] + rdt[2][0]*var[82] + rdt[2][1]*var[83] + rdt[2][2]*var[84];
        _cov[8][8] = _cov[10][8]*rdt[2][1] + _cov[11][8]*rdt[2][2] + _cov[8][8] + _cov[9][8]*rdt[2][0] + rdt[2][0]*var[85] + rdt[2][1]*var[86] + rdt[2][2]*var[87];
        _cov[0][9] = var[12];
        _cov[1][9] = var[25];
        _cov[2][9] = var[37];
        _cov[3][9] = var[48];
        _cov[4][9] = var[61];
        _cov[5][9] = var[74];
        _cov[6][9] = var[79];
        _cov[7][9] = var[82];
        _cov[8][9] = var[85];

        _cov[0][10] = var[13];
        _cov[1][10] = var[26];
        _cov[2][10] = var[38];
        _cov[3][10] = var[50];
        _cov[4][10] = var[63];
        _cov[5][10] = var[76];
        _cov[6][10] = var[80];
        _cov[7][10] = var[83];
        _cov[8][10] = var[86];

        _cov[0][11] = var[14];
        _cov[1][11] = var[27];
        _cov[2][11] = var[39];
        _cov[3][11] = var[52];
        _cov[4][11] = var[65];
        _cov[5][11] = var[78];
        _cov[6][11] = var[81];
        _cov[7][11] = var[84];
        _cov[8][11] = var[87];

        _cov[0][12] = var[5];
        _cov[1][12] = var[18];
        _cov[2][12] = var[29];
        _cov[3][12] = var[40];
        _cov[4][12] = var[53];
        _cov[5][12] = var[67];
        _cov[6][12] = _cov[12][10]*rdt[0][1] + _cov[12][11]*rdt[0][2] + _cov[12][6] + var[47];
        _cov[7][12] = _cov[12][10]*rdt[1][1] + _cov[12][11]*rdt[1][2] + _cov[12][7] + var[60];
        _cov[8][12] = _cov[12][10]*rdt[2][1] + _cov[12][11]*rdt[2][2] + _cov[12][8] + var[73];

        _cov[0][13] = var[6];
        _cov[1][13] = var[19];
        _cov[2][13] = var[30];
        _cov[3][13] = var[41];
        _cov[4][13] = var[54];
        _cov[5][13] = var[68];
        _cov[6][13] = _cov[13][11]*rdt[0][2] + _cov[13][6] + _cov[13][9]*rdt[0][0] + var[49];
        _cov[7][13] = _cov[13][11]*rdt[1][2] + _cov[13][7] + _cov[13][9]*rdt[1][0] + var[62];
        _cov[8][13] = _cov[13][11]*rdt[2][2] + _cov[13][8] + _cov[13][9]*rdt[2][0] + var[75];

        _cov[0][14] = var[7];
        _cov[1][14] = var[20];
        _cov[2][14] = var[31];
        _cov[3][14] = var[42];
        _cov[4][14] = var[55];
        _cov[5][14] = var[69];
        _cov[6][14] = _cov[14][10]*rdt[0][1] + _cov[14][6] + _cov[14][9]*rdt[0][0] + var[51];
        _cov[7][14] = _cov[14][10]*rdt[1][1] + _cov[14][7] + _cov[14][9]*rdt[1][0] + var[64];
        _cov[8][14] = _cov[14][10]*rdt[2][1] + _cov[14][8] + _cov[14][9]*rdt[2][0] + var[77];

        _cov[0][15] = var[11];
        _cov[1][15] = var[24];
        _cov[2][15] = var[36];
        _cov[3][15] = var[46];
        _cov[4][15] = var[59];
        _cov[5][15] = var[66];
        _cov[6][15] = _cov[15][10]*rdt[0][1] + _cov[15][11]*rdt[0][2] + _cov[15][6] + _cov[15][9]*rdt[0][0];
        _cov[7][15] = _cov[15][10]*rdt[1][1] + _cov[15][11]*rdt[1][2] + _cov[15][7] + _cov[15][9]*rdt[1][0];
        _cov[8][15] = _cov[15][10]*rdt[2][1] + _cov[15][11]*rdt[2][2] + _cov[15][8] + _cov[15][9]*rdt[2][0];
    }

    for (int i = 0; i < 16; ++i) {
        for (int j = 0; j < 16; ++j) {
            cout << _cov[i][j] << ", ";
        }
        cout << endl;
    }
}


int main() {
    // 飞行数据生成
    float ts = 0.001;
    double ts_sim = 0.001;

    // 参数
    eskf::Parameters params;
    Vector3f gps_offset_left {-0.2f, -0.15f, -0.02f};
    Vector3f gps_offset_right {-0.2f, 0.15f, -0.02f};
    Vector3f baro_offset {0.02f, 0.f, 0.01f};
    Vector3f range_offset {-0.01f, 0.f, 0.02f};

    eskf::GESKF eskf_rtk(params);

    Vector3d euler0(0., 0., 0.3), vn0(0., 0., 0.), pos0(23.1659394 * arcdeg, 113.4522718 * arcdeg, 20.);
    vector<array<double, 5>> wat(13);
    wat[0][0] = 0., wat[0][1] = 0., wat[0][2] = 0., wat[0][3] = 0., wat[0][4] = 10.;            // 静止
    wat[1][0] = 0., wat[1][1] = 0., wat[1][2] = 0., wat[1][3] = 1., wat[1][4] = 10.;            // 加速
    wat[2][0] = 0., wat[2][1] = 0., wat[2][2] = 0., wat[2][3] = 0., wat[2][4] = 10.;            // 匀速
    wat[3][0] = 0., wat[3][1] = 5., wat[3][2] = 0., wat[3][3] = 0., wat[3][4] = 4.;             // 抬头
    wat[4][0] = 0., wat[4][1] = 0., wat[4][2] = 0., wat[4][3] = 0., wat[4][4] = 10.;            // 匀速
    wat[5][0] = 0., wat[5][1] = -5, wat[5][2] = 0., wat[5][3] = 0., wat[5][4] = 4.;             // 低头
    wat[6][0] = 0., wat[6][1] = 0., wat[6][2] = 0., wat[6][3] = 0., wat[6][4] = 10.;            // 匀速
    wat[7][0] = 10., wat[7][1] = 0., wat[7][2] = 0., wat[7][3] = 0., wat[7][4] = 1.;            // 横滚
    wat[8][0] = 0., wat[8][1] = 0., wat[8][2] = 9., wat[8][3] = 0., wat[8][4] = 10.;            // 转弯
    wat[9][0] = -10., wat[9][1] = 0., wat[9][2] = 0., wat[9][3] = 0., wat[9][4] = 1.;           // 横滚
    wat[10][0] = 0., wat[10][1] = 0., wat[10][2] = 0., wat[10][3] = 0., wat[10][4] = 10.;       // 匀速
    wat[11][0] = 0., wat[11][1] = 0., wat[11][2] = 0., wat[11][3] = -1., wat[11][4] = 10.;      // 减速
    wat[12][0] = 0., wat[12][1] = 0., wat[12][2] = 0., wat[12][3] = 0., wat[12][4] = 10.;       // 静止

    for (auto & i : wat) {
        i[0] *= arcdeg;
        i[1] *= arcdeg;
        i[2] *= arcdeg;
    }

    // pve数据
    env.traj_gen(euler0, vn0, pos0, wat, ts_sim);
    const vector<Vector3d> &pos = env.get_pos();
    const vector<Vector3d> &vn = env.get_vn();
    const vector<Vector3d> &euler = env.get_euler();

    // imu数据
    env.ev2imu(euler, vn, pos, ts_sim);
    const vector<Vector3d> &wm = env.get_wm();
    const vector<Vector3d> &vm = env.get_vm();

    // 量测imu数据
    const Vector3d bias_gyro(0.001, 0.001, 0.001);
//    const Vector3d bias_acc(0.0001, 0.0001, 0.0001);
    const Vector3d bias_acc(0., 0., 0.);
    const Vector3d std_gyro(1.5e-2f, 1.5e-2f, 1.5e-2f);
    const Vector3d std_acc(3.5e-1f, 3.5e-1f, 3.5e-1f);
    static vector<Vector3d> wm_meas;
    static vector<Vector3d> vm_meas;
    wm_meas.resize(wm.size());
    vm_meas.resize(vm.size());
    env.imu_add_err(wm, vm, bias_gyro, std_gyro, bias_acc, std_acc, ts_sim, wm_meas, vm_meas);

    // 时间序列
    time_us_seq.resize(pos.size());
    time_us_seq[0] = 0;
    auto dt_us = (unsigned long)(ts_sim * 1e6f);
    for (unsigned int i = 1; i < pos.size(); ++i) {
        time_us_seq[i] = time_us_seq[i - 1] + dt_us;
    }

    // 地球坐标位置转导航坐标
    static vector<Vector3d> pn;
    pn.resize(pos.size());
    for (unsigned int i = 0; i < pos.size(); ++i) {
        pn[i] = diff_geo(pos[i], pos0);
    }

    // 位置和速度量测
    static vector<Vector3f> pl_meas, vl_meas, pr_meas, vr_meas, mb_meas;
    static vector<Vector3f> w_meas, a_meas;
    pl_meas.resize(pos.size()), vl_meas.resize(pos.size()), pr_meas.resize(pos.size()), vr_meas.resize(pos.size()), mb_meas.resize(pos.size());
    w_meas.resize(pos.size()), a_meas.resize(pos.size());

    static vector<float> baro_meas;
    baro_meas.resize(pos.size());

    static vector<float> range_meas;
    range_meas.resize(pos.size());

    static vector<Vector2f> flow_meas;
    flow_meas.resize(pos.size());

    static vector<Vector3f> p_hat, v_hat, bg_hat, ba_hat, mb_hat, bm_hat;
    static vector<Vector3f> euler_hat;
    static vector<float> g_hat;
    p_hat.resize(pos.size()), v_hat.resize(pos.size()), bg_hat.resize(pos.size()), ba_hat.resize(pos.size()), mb_hat.resize(pos.size()), bm_hat.resize(pos.size());
    euler_hat.resize(pos.size());
    g_hat.resize(pos.size());

    default_random_engine random_engine;
    normal_distribution<float> dist(0.f, 1.f);
    const Vector3f& dl = gps_offset_left;
    const Vector3f& dr = gps_offset_right;
    for (unsigned int i = 0; i < pos.size(); ++i) {
        Vector3f npl(dist(random_engine) * params.gps_pos_horz_noise,
                     dist(random_engine) * params.gps_pos_horz_noise,
                     dist(random_engine) * params.gps_pos_vert_noise);
        Vector3f nvl(dist(random_engine) * params.gps_vel_horz_noise,
                     dist(random_engine) * params.gps_vel_horz_noise,
                     dist(random_engine) * params.gps_vel_vert_noise);
        Vector3f npr(dist(random_engine) * params.gps_pos_horz_noise,
                     dist(random_engine) * params.gps_pos_horz_noise,
                     dist(random_engine) * params.gps_pos_vert_noise);
        Vector3f nvr(dist(random_engine) * params.gps_vel_horz_noise,
                     dist(random_engine) * params.gps_vel_horz_noise,
                     dist(random_engine) * params.gps_vel_vert_noise);

        Matrix3d RR = generator::euler2rot(euler[i]);
        Matrix3f R;
        for (uint8_t j = 0; j < 3; ++j) {
            for (uint8_t k = 0; k < 3; ++k) {
                R(j, k) = float(RR(j, k));
            }
        }

        Vector3f p(float(pn[i](0)), float(pn[i](1)), float(pn[i](2)));
        Vector3f v(float(vn[i](0)), float(vn[i](1)), float(vn[i](2)));
        Vector3f delta_ang(float(wm[i](0)), float(wm[i](1)), float(wm[i](2)));
        Vector3f delta_vel(float(vm[i](0)), float(vm[i](1)), float(vm[i](2)));

        pl_meas[i] = p + R * dl + npl*0.f;
        vl_meas[i] = v - R * (dl.cross(Vector3f(float(wm[i](0)), float(wm[i](1)), float(wm[i](2))) / ts)) + nvl*0.f;
        pr_meas[i] = p + R * dr + npr*0.f;
        vr_meas[i] = v - R * (dr.cross(Vector3f(float(wm[i](0)), float(wm[i](1)), float(wm[i](2))) / ts)) + nvr*0.f;
        w_meas[i] = Vector3f(float(wm_meas[i](0)), float(wm_meas[i](1)), float(wm_meas[i](2)))/ ts;
        a_meas[i] = Vector3f(float(vm_meas[i](0)), float(vm_meas[i](1)), float(vm_meas[i](2))) / ts;


        float n_baro = dist(random_engine) * params.baro_noise;
        float baro = generate_baro_data(R, p, baro_offset);
        baro_meas[i] = baro + n_baro;

        float n_range = dist(random_engine) * params.range_noise;
        float range = generate_range_data(R, p, range_offset);
        range_meas[i] = range + n_range;

        Vector2f n_flow(dist(random_engine) * params.flow_noise, dist(random_engine) * params.flow_noise);
        Vector2f flow = generate_optflow_data(range, R, v, w_meas[i], range_offset);
//        cout << "flow: " << flow(0) << ", " << flow(1) << ", vel: " << v(0) << ", " << v(1) << endl;
        flow_meas[i] = flow + n_flow * 0.f;
//        cout << n_flow(0) << endl;

//        cout << pn[i](2) << ", " << range_meas[i] << endl;
    }

    // 双天线经纬度与高度计算
    MapProjection horz_ref;
    horz_ref.initReference(pos0(0), pos0(1));

    double alt_ref;
    alt_ref = pos0(2);

    static vector<Vector3d> gps_l;
    static vector<Vector3d> gps_r;
    gps_l.resize(pos.size());
    gps_r.resize(pos.size());
    for (unsigned int i = 0; i < pos.size(); ++i) {
        Vector3f npl(dist(random_engine) * params.gps_pos_horz_noise,
                     dist(random_engine) * params.gps_pos_horz_noise,
                     dist(random_engine) * params.gps_pos_vert_noise);
        Vector3f npr(dist(random_engine) * params.gps_pos_horz_noise,
                     dist(random_engine) * params.gps_pos_horz_noise,
                     dist(random_engine) * params.gps_pos_vert_noise);

        Matrix3d RR = generator::euler2rot(euler[i]);
        Matrix3f R;
        for (uint8_t j = 0; j < 3; ++j) {
            for (uint8_t k = 0; k < 3; ++k) {
                R(j, k) = float(RR(j, k));
            }
        }

        horz_ref.initReference(pos[i](0) / arcdeg, pos[i](1) / arcdeg);
        alt_ref = pos[i](2);

        Vector3f offset_nav = R * gps_offset_left + npl*0.f;
        horz_ref.reproject(offset_nav(0), offset_nav(1), gps_l[i](0), gps_l[i](1));
        gps_l[i](2) = double(offset_nav(2)) + alt_ref;

//        cout << int(pos[i](0)*1.0e7) << ", " << int(pos[i](1)*1.0e7) << ", " << int(gps_l[i](0)*1.0e7) << ", " << int(gps_l[i](1)*1.0e7) << endl;

        offset_nav = R * gps_offset_right + npr*0.f;
        horz_ref.reproject(offset_nav(0), offset_nav(1), gps_r[i](0), gps_r[i](1));
        gps_r[i](2) = double(offset_nav(2)) + alt_ref;
    }

    // 磁场数据
    float mag_declination = get_mag_declination_radians((float)gps_l[0](0), (float)gps_l[0](1));
    float mag_inclination = get_mag_inclination_radians((float)gps_l[0](0), (float)gps_l[0](1));
    float mag_strength = get_mag_strength_gauss((float)gps_l[0](0), (float)gps_l[0](1));
    Vector2f mag_ang = {mag_inclination, mag_declination};
    const float cos_y = cosf(mag_ang(0));
    Vector3f mag_earth = {mag_strength * cosf(mag_ang(1)) * cos_y,
                          mag_strength * sinf(mag_ang(1)) * cos_y,
                          -mag_strength * sinf(mag_ang(0))};

    cout << "mag_earth: " << mag_earth(0) << ", " << mag_earth(1) << ", " << mag_earth(2) << endl;
    cout << "mag_earth: " << mag_strength << ", " << mag_inclination << ", " << mag_declination << endl;

    static vector<Vector3f> mag_meas;
    mag_meas.resize(pos.size());
    for (unsigned int i = 0; i < pos.size(); ++i) {
        Vector3f nm(dist(random_engine) * params.mag_body_noise,
                     dist(random_engine) * params.mag_body_noise,
                     dist(random_engine) * params.mag_body_noise);

        Matrix3d RR = generator::euler2rot(euler[i]);
        Matrix3f R;
        for (uint8_t j = 0; j < 3; ++j) {
            for (uint8_t k = 0; k < 3; ++k) {
                R(j, k) = float(RR(j, k));
            }
        }

        mag_meas[i] = R.transpose() * mag_earth;
    }

    // 数据融合
    eskf_rtk.enable_estimation_acc_bias();
    eskf_rtk.enable_estimation_gravity();
    eskf_rtk.enable_estimation_mag_norm();
    eskf_rtk.enable_estimation_mag_ang();
    eskf_rtk.enable_estimation_mag_bias();
    // eskf_rtk.enable_estimation_magnet();
    // eskf_rtk.enable_estimation_declination();
    // eskf_rtk.enable_estimation_magnet_bias();
//    eskf_rtk.disable_estimation_acc_bias();
//    eskf_rtk.disable_estimation_gravity();
//    eskf_rtk.disable_estimation_mag_norm();
//    eskf_rtk.disable_estimation_mag_ang();
//    eskf_rtk.disable_estimation_mag_bias();
    eskf_rtk.disable_estimation_wind();
    eskf_rtk.initialize();

//    const float *eskf_q = eskf_rtk.get_proc_noise_matrix();
//    for (int i = 0; i < 24; ++i) {
//        cout << eskf_q[i] << ", ";
//    }
//    cout << endl;




//    Quatd tmp(double(eskf_rtk.get_quaternion()(0)), double(eskf_rtk.get_quaternion()(1)), double(eskf_rtk.get_quaternion()(2)), double(eskf_rtk.get_quaternion()(3)));
//    Vector3d tmp1 = generator::quat2euler(tmp);
//
////    clock_t t1 = clock();
//    for (unsigned i = 0; i < pos.size(); ++i) {
////        break;
//
//        Vector3f delta_ang(float(wm[i](0)), float(wm[i](1)), float(wm[i](2))), delta_vel(float(vm[i](0)), float(vm[i](1)), float(vm[i](2)));
//        if (i > 0) {
//            Vector3d delta_ang_meas = wm[i] + 1./12. * wm[i - 1].cross(wm[i]);
//            delta_ang = {float(delta_ang_meas(0)), float(delta_ang_meas(1)), float(delta_ang_meas(2))};
//
//            Vector3d delta_vel_meas = vm[i] + 0.5 * wm[i].cross(vm[i]) + 1./12. * (wm[i - 1].cross(vm[i]) + vm[i - 1].cross(wm[i]));
//            delta_vel = {float(delta_vel_meas(0)), float(delta_vel_meas(1)), float(delta_vel_meas(2))};
//        } else {
//            const Vector3d &delta_ang_meas = wm[i];
//            delta_ang = {float(delta_ang_meas(0)), float(delta_ang_meas(1)), float(delta_ang_meas(2))};
//
//            const Vector3d &delta_vel_meas = vm[i];
//            delta_vel = {float(delta_vel_meas(0)), float(delta_vel_meas(1)), float(delta_vel_meas(2))};
//        }
//
//        Vector<bool, 3> gyro_clipping = {};
//        Vector<bool, 3> acc_clipping = {};
//
//        eskf::ImuSample imu_sample;
//        imu_sample.delta_ang = delta_ang;
//        imu_sample.delta_vel = delta_vel;
////        imu_sample.delta_ang = Vector3f(ts, ts, ts);
////        imu_sample.delta_vel = Vector3f(ts, ts, ts);
//        imu_sample.delta_ang_dt = ts;
//        imu_sample.delta_vel_dt = ts;
//
//        eskf_rtk.predict_state(imu_sample);
//        eskf_rtk.predict_covariance(imu_sample);
//
////        break;
//
//        Vector3f gps_offset_left_nav = eskf_rtk.get_Rnb() * gps_offset_left;
//        Vector3f w_cross_offset_left_body = eskf_rtk.get_gyro_corr() % gps_offset_left;
//        Vector3f w_cross_offset_left_nav = eskf_rtk.get_Rnb() * w_cross_offset_left_body;
//
//        Vector3f gps_offset_right_nav = eskf_rtk.get_Rnb() * gps_offset_right;
//        Vector3f w_cross_offset_right_body = eskf_rtk.get_gyro_corr() % gps_offset_right;
//        Vector3f w_cross_offset_right_nav = eskf_rtk.get_Rnb() * w_cross_offset_right_body;
//
//        eskf::FuseData<2> pos_left_horz_fuse_data;
//        eskf::FuseData<1> pos_left_vert_fuse_data;
//        eskf::FuseData<2> vel_left_horz_fuse_data;
//        eskf::FuseData<1> vel_left_vert_fuse_data;
//        eskf::FuseData<2> pos_right_horz_fuse_data;
//        eskf::FuseData<1> pos_right_vert_fuse_data;
//        eskf::FuseData<2> vel_right_horz_fuse_data;
//        eskf::FuseData<1> vel_right_vert_fuse_data;
//
//        if (i % 10 == 0) {
//            eskf_rtk.fuse_pos_horz(pl_meas[i].xy(), gps_offset_left, gps_offset_left_nav, params.gps_pos_horz_innov_gate, params.gps_pos_horz_noise, pos_left_horz_fuse_data);
//            eskf_rtk.fuse_pos_vert(pl_meas[i](2), gps_offset_left, gps_offset_left_nav, params.gps_pos_vert_innov_gate, params.gps_pos_vert_noise, pos_left_vert_fuse_data);
//            eskf_rtk.fuse_vel_horz(vl_meas[i].xy(), gps_offset_left, gps_offset_left_nav, w_cross_offset_left_body, w_cross_offset_left_nav, params.gps_vel_horz_innov_gate, params.gps_vel_horz_noise, vel_left_horz_fuse_data);
//            eskf_rtk.fuse_vel_vert(vl_meas[i](2), gps_offset_left, gps_offset_left_nav, w_cross_offset_left_body, w_cross_offset_left_nav, params.gps_vel_horz_innov_gate, params.gps_vel_horz_noise, vel_left_vert_fuse_data);
//
//            eskf_rtk.fuse_pos_horz(pr_meas[i].xy(), gps_offset_right, gps_offset_right_nav, params.gps_pos_horz_innov_gate, params.gps_pos_horz_noise, pos_right_horz_fuse_data);
//            eskf_rtk.fuse_pos_vert(pr_meas[i](2), gps_offset_right, gps_offset_right_nav, params.gps_pos_vert_innov_gate, params.gps_pos_vert_noise, pos_right_vert_fuse_data);
//            eskf_rtk.fuse_vel_horz(vr_meas[i].xy(), gps_offset_right, gps_offset_right_nav, w_cross_offset_right_body, w_cross_offset_right_nav, params.gps_vel_horz_innov_gate, params.gps_vel_horz_noise, vel_right_horz_fuse_data);
//            eskf_rtk.fuse_vel_vert(vr_meas[i](2), gps_offset_right, gps_offset_right_nav, w_cross_offset_right_body, w_cross_offset_right_nav, params.gps_vel_horz_innov_gate, params.gps_vel_horz_noise, vel_right_vert_fuse_data);
//        }
//
//        eskf_rtk.correct_covariance();
//        eskf_rtk.correct_state();
//
//        Quatd tmp(double(eskf_rtk.get_quaternion()(0)), double(eskf_rtk.get_quaternion()(1)), double(eskf_rtk.get_quaternion()(2)), double(eskf_rtk.get_quaternion()(3)));
//        Vector3d tmp1 = generator::quat2euler(tmp);
//
//        euler_hat[i] = {float(tmp1(0)), float(tmp1(1)), float(tmp1(2))};
//        p_hat[i] = eskf_rtk.get_position();
//        v_hat[i] = eskf_rtk.get_velocity();
//        bg_hat[i] = eskf_rtk.get_gyro_bias();
//        ba_hat[i] = eskf_rtk.get_acc_bias();
//        g_hat[i] = eskf_rtk.get_gravity();
//
////        cout << "euler: " << euler[i](0) << ", " << euler[i](1) << ", " << euler[i](2) << ", ";
////        cout << euler_hat[i](0) << ", " << euler_hat[i](1) << ", " << euler_hat[i](2) << endl;
////        cout << "ba_x: " << ba_hat[i](0) << ", ba_y" << ba_hat[i](1) << ", " << ba_hat[i](2) << endl;
////        cout << "g: " << g_hat[i] << endl;
////        cout << "pos_horz: " << pn[i](0) << ", " << pn[i](1) << ", " << pn[i](2) << ", ";
////        cout << p_hat[i](0) << ", " << p_hat[i](1) << ", " << p_hat[i](2) << endl;
////        cout << "bias_acc: " << bias_acc(0) << ", " << bias_acc(1) << ", " << bias_acc(2) << ", ";
////        cout << ba_hat[i](0) << ", " << ba_hat[i](1) << ", " << ba_hat[i](2) << endl;
////        cout << "bias_gyro: " << bias_gyro(0) << ", " << bias_gyro(1) << ", " << bias_gyro(2) << ", ";
////        cout << bg_hat[i](0) << ", " << bg_hat[i](1) << ", " << bg_hat[i](2) << endl;
////        cout << "ang_var: " << eskf_rtk.get_covariance_matrix()[6][6] << ", " << eskf_rtk.get_covariance_matrix()[7][7] << ", " << eskf_rtk.get_covariance_matrix()[8][8] << endl;
//
//        cout << "vel: " << vn[i](0) << ", " << vn[i](1) << ", " << vn[i](2) << ", ";
//        cout << v_hat[i](0) << ", " << v_hat[i](1) << ", " << v_hat[i](2) << endl;
//
////        cout << "P[2][2] = " << eskf_rtk.get_covariance_matrix()[2][2] << ", P[5][5] = " << eskf_rtk.get_covariance_matrix()[5][5] << endl;
//
//    }
////    eskf::ImuSample imu_sample;
////    imu_sample.delta_ang = Vector3f(ts, ts, ts);
////    imu_sample.delta_vel = Vector3f(ts, ts, ts);
////    imu_sample.delta_ang_dt = ts;
////    imu_sample.delta_vel_dt = ts;
////    cout << imu_sample.delta_vel_clipping[0] << endl;
////
////    for (int i = 0; i < 10; ++i) {
////        eskf_rtk.predict_state(imu_sample);
////        eskf_rtk.predict_covariance(imu_sample);
////    }
////
//////    cout << eskf_rtk.get_velocity()(0) << ", " << eskf_rtk.get_velocity()(1) << ", " << eskf_rtk.get_velocity()(2) << endl;
////    const float (*cov)[24] = eskf_rtk.get_covariance_matrix();
////    for (int i = 0; i < 24; ++i) {
////        for (int j = 0; j < 24; ++j) {
////            cout << cov[i][j] << ", ";
////        }
////        cout << endl;
////    }
////    cout << eskf_rtk.get_velocity()(0) << ", " << eskf_rtk.get_velocity()(1) << ", " << eskf_rtk.get_velocity()(2) << endl;
//
//
////    clock_t t2 = clock();
////    cout << t2 - t1 << endl;





//    ahrs::AHRS nonCF(ts);
//
//    eskf::ESKFRunner<1> eskf_runner(eskf_rtk);
////    eskf_runner._ned_origin_initialised = true;
////    eskf_runner._control_status.flags.tilt_align = true;
////    eskf_runner._control_status.flags.yaw_align = true;
//    eskf_runner._control_status.flags.gps_hgt = true;
////    eskf_runner._control_status.flags.gps_horz = true;
//    eskf_runner._control_status.flags.gps_vel = true;
////    eskf_runner.set_in_air_status(true);
//    eskf_runner.set_vehicle_at_rest(true);
//
//    eskf_rtk.set_terrain(TERR);
//
//    eskf_rtk.set_mag_norm(mag_strength);
//    eskf_rtk.set_mag_ang(mag_ang);
//
//    eskf::ImuSample imu_sample;
////    eskf::GpsSample gps_sample[2];
//    eskf::GpsMessage gps_message;
//    eskf::BaroSample baro_sample;
//    eskf::RangeSample range_sample;
//    eskf::FlowSample flow_sample;
//    eskf::MagSample mag_sample;
//    for (unsigned i = 0; i < pos.size(); ++i) {
//        if (i > 1000) {
//            eskf_runner.set_in_air_status(true);
//            eskf_runner.set_vehicle_at_rest(false);
//        }
//
//        imu_sample.delta_ang = {float(wm_meas[i](0)), float(wm_meas[i](1)), float(wm_meas[i](2))};
//        imu_sample.delta_vel = {float(vm_meas[i](0)), float(vm_meas[i](1)), float(vm_meas[i](2))};
//        imu_sample.delta_ang_dt = ts;
//        imu_sample.delta_vel_dt = ts;
//        imu_sample.time_us += (uint64_t)(1e6f * ts);
//
//        eskf_runner.set_imu_data(imu_sample);
//
////        std::cout << "step of eskf_runner = " << int(i) << std::endl;
////        cout << "flow_meas[i] = " << flow_meas[i](0) << ", " << flow_meas[i](1) << endl;
////        flow_sample.flow_xy_rad += flow_meas[i] * ts;
////        flow_sample.dt += ts;
//        if (i % 10 == 0) {
////            gps_sample[0].pos_horz = pl_meas[i].xy();
////            gps_sample[0].hgt = -pl_meas[i](2);
////            gps_sample[0].vel = vl_meas[i];
////            gps_sample[0].fix_type = 5;
////            gps_sample[0].time_us = imu_sample.time_us;
////
////            gps_sample[1].pos_horz = pr_meas[i].xy();
////            gps_sample[1].hgt = -pr_meas[i](2);
////            gps_sample[1].vel = vr_meas[i];
////            gps_sample[1].fix_type = 5;
////            gps_sample[1].time_us = imu_sample.time_us;
////
//////            if (i > 1000) {
//////                gps_sample[0].fix_type = 5.f;
//////            }
////
////            eskf_runner.set_gps_data(gps_sample[0], 0);
////            eskf_runner.set_gps_data(gps_sample[1], 1);
//
//            gps_message.fix_type = 5;
//            gps_message.nsats = 30;
//            gps_message.eph = 0.5f;
//            gps_message.epv = 0.5f;
//            gps_message.sacc = 0.3f;
//            gps_message.pdop = 5.f;
//            gps_message.vel_ned_valid = true;
//            gps_message.yaw_offset = 0.f;
//            gps_message.lat = gps_l[i](0) * 1.0e7;
//            gps_message.lon = gps_l[i](1) * 1.0e7;
//            gps_message.alt = gps_l[i](2) * 1.0e3;
//            gps_message.vel_ned = vl_meas[i];
//            gps_message.vel_m_s = vl_meas[i].length();
//            gps_message.yaw = NAN;
//            gps_message.time_us = imu_sample.time_us;
//
//            eskf_runner.set_gps_data(gps_message, 0);
////            cout << pl_meas[i](0) << ", " << pl_meas[i](1) << ", " << pl_meas[i](2) << endl;
//
//            gps_message.fix_type = 5;
//            gps_message.nsats = 30;
//            gps_message.eph = 0.5f;
//            gps_message.epv = 0.5f;
//            gps_message.sacc = 0.3f;
//            gps_message.pdop = 5.f;
//            gps_message.vel_ned_valid = true;
//            gps_message.yaw_offset = 0.f;
//            gps_message.lat = gps_r[i](0) * 1.0e7;
//            gps_message.lon = gps_r[i](1) * 1.0e7;
//            gps_message.alt = gps_r[i](2) * 1.0e3;
//            gps_message.vel_ned = vr_meas[i];
//            gps_message.vel_m_s = vr_meas[i].length();
//            gps_message.yaw = NAN;
//            gps_message.time_us = imu_sample.time_us;
//
//            eskf_runner.set_gps_data(gps_message, 1);
////            cout << pr_meas[i](0) << ", " << pr_meas[i](1) << ", " << pr_meas[i](2) << endl;
//
//            baro_sample.time_us = imu_sample.time_us;
//            baro_sample.hgt = baro_meas[i];
//            eskf_runner.set_baro_date(baro_sample);
//
//            range_sample.time_us = imu_sample.time_us;
//            range_sample.rng = range_meas[i];
//            range_sample.quality = (uint8_t)255;
//            eskf_runner.set_range_data(range_sample);
//
//            mag_sample.time_us = imu_sample.time_us;
//            mag_sample.mag = mag_meas[i];
//            eskf_runner.set_magnet_data(mag_sample);
////            std::cout << "set gps data" << std::endl;
//        }
//
////        flow_sample.time_us = imu_sample.time_us;
////        flow_sample.quality = (uint8_t)255;
////        eskf_runner.set_flow_date(flow_sample);
////
////        flow_sample.flow_xy_rad.setZero();
////        flow_sample.dt = 0.f;
//
//        eskf_runner.update();
//
//        nonCF.feed_imu(imu_sample.delta_ang, imu_sample.delta_vel, imu_sample.delta_ang_dt);
//        if (i % 10 == 0) {
////            Vector3f cf_vel(vn[i](0), vn[i](1), vn[i](2));
//            Vector3f cf_vel;
//            cf_vel = vl_meas[i] - nonCF.r() * ((w_meas[i] + nonCF.delta_ang_offset() / ts) % dl);
//            nonCF.feed_vel(cf_vel);
//            Vector3f diff_nav = pl_meas[i] - pr_meas[i];
//            Vector3f diff_body = dl - dr;
//            nonCF.feed_aux_meas(diff_nav, diff_body, 0.05f);
//            nonCF.fit();
//        }
//
//        Quatd tmp(double(eskf_rtk.get_quaternion()(0)), double(eskf_rtk.get_quaternion()(1)), double(eskf_rtk.get_quaternion()(2)), double(eskf_rtk.get_quaternion()(3)));
//        Vector3d tmp1 = generator::quat2euler(tmp);
//
//        euler_hat[i] = {float(tmp1(0)), float(tmp1(1)), float(tmp1(2))};
//        p_hat[i] = eskf_rtk.get_position();
//        v_hat[i] = eskf_rtk.get_velocity();
//        bg_hat[i] = eskf_rtk.get_gyro_bias();
//        ba_hat[i] = eskf_rtk.get_acc_bias();
//        g_hat[i] = eskf_rtk.get_gravity();
//
//        tmp(0) = double(nonCF.q()(0));
//        tmp(1) = double(nonCF.q()(1));
//        tmp(2) = double(nonCF.q()(2));
//        tmp(3) = double(nonCF.q()(3));
//        tmp1 = generator::quat2euler(tmp);
//
////        cout << "euler: " << euler[i](0) << ", " << euler[i](1) << ", " << euler[i](2) << ", ";
////        cout << euler_hat[i](0) << ", " << euler_hat[i](1) << ", " << euler_hat[i](2) << endl;
////        cout << "mag_earth: " << eskf_rtk.get_magnet_earth()(0) << ", " << eskf_rtk.get_magnet_earth()(1) << ", " << eskf_rtk.get_magnet_earth()(2) << endl;
////        cout << "baro bias: " << eskf_rtk.get_baro_bias() << endl;
////        cout << "terr: " << eskf_rtk.get_terrain() << ", bias: " << eskf_rtk.get_baro_bias() << std::endl;
////        cout << tmp1(0) << ", " << tmp1(1) << ", " << tmp1(2) << endl;
//
//        cout << "pos: " << pn[i](0) << ", " << pn[i](1) << ", " << pn[i](2) << ", ";
//        cout << p_hat[i](0) << ", " << p_hat[i](1) << ", " << p_hat[i](2) << endl;
//
////        cout << "vel: " << vn[i](0) << ", " << vn[i](1) << ", " << vn[i](2) << ", ";
//////        cout << nonCF.v()(0) << "," << nonCF.v()(1) << "," << nonCF.v()(2) << endl;
////        cout << v_hat[i](0) << ", " << v_hat[i](1) << ", " << v_hat[i](2) << endl;
//
//    }
//
//
////    test1();
////    test2();





//    inav::INAV i_nav(eskf_rtk, 10);
//
//    i_nav._gps.update();
//    i_nav._baro.update();
//    i_nav._range.update();
//    i_nav.update();



//    digital_signal_processing::NotchFilter notch_filter(800.f, 18.9954f, 15.0226f, -17.0334f);
//
//    notch_filter.reset_filter_by_output(1.f);
//    cout << notch_filter(1.f) << endl;

//    for (uint32_t i = 0; i < 2000; ++i) {
//        float t = float(i) / 800.f;
//        cout << notch_filter(sinf(2.f * M_PI_F * 18.9954f * t)) << endl;
//    }

//    control::ParamsLoopShaping tuner()


    ofstream dataFile;
    dataFile.open("butter_worth_sweep.csv", ios::out | ios::trunc);

    digital_signal_processing::SineSweep sine_sweep(0.01f, 30.f, 0.00125f, 600.f);
    digital_signal_processing::Butterworth<2> butter_worth(800.f, 20.f);

    while (!sine_sweep.finished()) {
        float u = sine_sweep();
        float y = butter_worth(u);

        dataFile << u << ",";
        dataFile << y << endl;
    }
    dataFile.close();

    return 0;
}

