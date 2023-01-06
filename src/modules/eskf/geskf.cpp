//
// Created by Cain on 2023/1/3.
//

#include "geskf.h"
#include <iostream>

namespace eskf {
    void GESKF::predict_covariance(const Vector3f &delta_ang, const Vector3f &delta_vel,
                                   const Vector<bool, 3> &gyro_clipping, const Vector<bool, 3> &acc_clipping) {
        // -Rnb
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
        const float gam_b_del_ang = math::constrain(1.f - _dt * _params.gyro_bias_tau_inv, 0.f, 1.f);
        const float gam_b_del_vel = math::constrain(1.f - _dt * _params.acc_bias_tau_inv, 0.f, 1.f);
        const float gam_b_mag = math::constrain(1.f - _dt * _params.mag_bias_tau_inv, 0.f, 1.f);
        const float gam_w = math::constrain(1.f - _dt * _params.wind_tau_inv, 0.f, 1.f);

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

        /*
         * R * diag(nx*nx, ny*ny, nz*nz) * R'
         * 如果acc达到了限幅, 则需要增大该量测噪声
         * */
        float bad_var = sq(_params.acc_noise) * _dt2;
        float good_var = sq(_params.acc_bad_noise) * _dt2;
        Vector3f var_delta_vel;
        var_delta_vel(0) = acc_clipping(0) ? bad_var : good_var;
        var_delta_vel(1) = acc_clipping(1) ? bad_var : good_var;
        var_delta_vel(2) = acc_clipping(2) ? bad_var : good_var;
        float sx = var_delta_vel(0);
        float sy = var_delta_vel(1);
        float sz = var_delta_vel(2);
        float r00_sx = mR00 * sx;
        float r01_sy = mR01 * sy;
        float r02_sz = mR02 * sz;
        float r10_sx = mR10 * sx;
        float r11_sy = mR11 * sy;
        float r12_sz = mR12 * sz;
        float r00_sx_r10_plus_r01_sy_r11_plus_r02_sz_r12 = r00_sx * mR10 + r01_sy * mR11 + r02_sz * mR12;
        float r00_sx_r20_plus_r01_sy_r21_plus_r02_sz_r22 = r00_sx * mR20 + r01_sy * mR21 + r02_sz * mR22;
        float r10_r20_sx_plus_r11_r21_sy_plus_r12_r22_sz = r10_sx * mR20 + r11_sy * mR21 + r12_sz * mR22;
        _P[3][3] = kahan_summation(_P[3][3], mR00 * r00_sx + mR01 * r01_sy + mR02 * r02_sz, _accumulator[3]);
        _P[3][4] += r00_sx_r10_plus_r01_sy_r11_plus_r02_sz_r12;
        _P[3][5] += r00_sx_r20_plus_r01_sy_r21_plus_r02_sz_r22;
        _P[4][4] = kahan_summation(_P[4][4], mR10 * r10_sx + mR11 * r11_sy + mR12 * r12_sz, _accumulator[4]);
        _P[4][5] += r10_r20_sx_plus_r11_r21_sy_plus_r12_r22_sz;
        _P[5][5] = kahan_summation(_P[5][5], mR20 * mR20 * sx + mR21 * mR21 * sy + mR22 * mR22 * sz, _accumulator[5]);

        /*
         * R * diag(nx*nx, ny*ny, nz*nz) * R'
         * 如果gyro达到了限幅, 则需要增大该量测噪声
         * */
        bad_var = _params.gyro_bad_noise * _params.gyro_bad_noise * _dt2;
        good_var = _params.gyro_noise * _params.gyro_noise * _dt2;
        Vector3f var_delta_ang;
        var_delta_ang(0) = gyro_clipping(0) ? bad_var : good_var;
        var_delta_ang(1) = gyro_clipping(1) ? bad_var : good_var;
        var_delta_ang(2) = gyro_clipping(2) ? bad_var : good_var;
        sx = var_delta_ang(0);
        sy = var_delta_ang(1);
        sz = var_delta_ang(2);
        r00_sx = mR00 * sx;
        r01_sy = mR01 * sy;
        r02_sz = mR02 * sz;
        r10_sx = mR10 * sx;
        r11_sy = mR11 * sy;
        r12_sz = mR12 * sz;
        r00_sx_r10_plus_r01_sy_r11_plus_r02_sz_r12 = r00_sx * mR10 + r01_sy * mR11 + r02_sz * mR12;
        r00_sx_r20_plus_r01_sy_r21_plus_r02_sz_r22 = r00_sx * mR20 + r01_sy * mR21 + r02_sz * mR22;
        r10_r20_sx_plus_r11_r21_sy_plus_r12_r22_sz = r10_sx * mR20 + r11_sy * mR21 + r12_sz * mR22;
        _P[6][6] = kahan_summation(_P[6][6], mR00 * r00_sx + mR01 * r01_sy + mR02 * r02_sz, _accumulator[6]);
        _P[6][7] += r00_sx_r10_plus_r01_sy_r11_plus_r02_sz_r12;
        _P[6][8] += r00_sx_r20_plus_r01_sy_r21_plus_r02_sz_r22;
        _P[7][7] = kahan_summation(_P[7][7], mR10 * r10_sx + mR11 * r11_sy + mR12 * r12_sz, _accumulator[7]);
        _P[7][8] += r10_r20_sx_plus_r11_r21_sy_plus_r12_r22_sz;
        _P[8][8] = kahan_summation(_P[8][8], mR20 * mR20 * sx + mR21 * mR21 * sy + mR22 * mR22 * sz, _accumulator[8]);

        // F * P * F' + Q
        float proc_var_pos = sq(_params.pos_proc_noise) * _dt2;
        float proc_var_vel = sq(_params.vel_proc_noise) * _dt2;
        float proc_var_ang = sq(_params.ang_axis_proc_noise) * _dt2;
        float proc_var_delta_ang = sq(_params.gyro_bias_proc_noise) * _dt4;
        float proc_var_delta_vel = sq(_params.acc_bias_proc_noise) * _dt4;
        for (uint8_t i = 0; i < 3; ++i) {
            _P[i][i] = kahan_summation(_P[i][i], proc_var_pos, _accumulator[i]);
            _P[i + 3][i + 3] = kahan_summation(_P[i + 3][i + 3], proc_var_pos, _accumulator[i + 3]);
            _P[i + 6][i + 6] = kahan_summation(_P[i + 6][i + 6], proc_var_pos, _accumulator[i + 6]);
            _P[i + 9][i + 9] = kahan_summation(_P[i + 9][i + 9], proc_var_pos, _accumulator[i + 9]);
        }

        if (_control_status.flags.acc_x_bias) {
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

            _P[12][12] = kahan_summation(_P[12][12], proc_var_delta_vel, _accumulator[12]);
        }

        if (_control_status.flags.acc_y_bias) {
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

            _P[13][13] = kahan_summation(_P[13][13], proc_var_delta_vel, _accumulator[13]);
        }

        if (_control_status.flags.acc_z_bias) {
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

            _P[14][14] = kahan_summation(_P[14][14], proc_var_delta_vel, _accumulator[14]);
        }

        if (_control_status.flags.grav) {
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

            float proc_var_grav = sq(_params.grav_proc_noise) * _dt2;
            _P[15][15] = kahan_summation(_P[15][15], proc_var_grav, _accumulator[15]);
        }

        if (_control_status.flags.mag_norm) {
            _P[0][16] = _P[16][0] + _P[16][3] * _dt;
            _P[1][16] = _P[16][1] + _P[16][4] * _dt;
            _P[2][16] = _P[16][2] + _P[16][5] * _dt;
            _P[3][16] = _P[16][12] * mR00 + _P[16][13] * mR01 + _P[16][14] * mR02 + _P[16][3] + _P[16][7] * dv2 - _P[16][8] * dv1;
            _P[4][16] = _P[16][12] * mR10 + _P[16][13] * mR11 + _P[16][14] * mR12 + _P[16][4] - _P[16][6] * dv2 + _P[16][8] * dv0;
            _P[5][16] = _P[16][12] * mR20 + _P[16][13] * mR21 + _P[16][14] * mR22 + _P[16][15] * _dt + _P[16][5] + _P[16][6] * dv1 - _P[16][7] * dv0;
            _P[6][16] = _P[16][10] * mR01 + _P[16][11] * mR02 + _P[16][6] + _P[16][9] * mR00;
            _P[7][16] = _P[16][10] * mR11 + _P[16][11] * mR12 + _P[16][7] + _P[16][9] * mR10;
            _P[8][16] = _P[16][10] * mR21 + _P[16][11] * mR22 + _P[16][8] + _P[16][9] * mR20;
            _P[9][16] = _P[16][9] * gam_b_del_ang;
            _P[10][16] = _P[16][10] * gam_b_del_ang;
            _P[11][16] = _P[16][11] * gam_b_del_ang;
            _P[12][16] = _P[16][12] * gam_b_del_vel;
            _P[13][16] = _P[16][13] * gam_b_del_vel;
            _P[14][16] = _P[16][14] * gam_b_del_vel;
            _P[15][16] = _P[16][15];
            _P[16][16] = _P[16][16];

            float proc_var_mag_norm = sq(_params.mag_norm_proc_noise) * _dt2;
            _P[16][16] = kahan_summation(_P[16][16], proc_var_mag_norm, _accumulator[16]);
        }

        if (_control_status.flags.mag_ang) {
            _P[0][17] = _P[17][0] + _P[17][3] * _dt;
            _P[1][17] = _P[17][1] + _P[17][4] * _dt;
            _P[2][17] = _P[17][2] + _P[17][5] * _dt;
            _P[3][17] = _P[17][12] * mR00 + _P[17][13] * mR01 + _P[17][14] * mR02 + _P[17][3] + _P[17][7] * dv2 - _P[17][8] * dv1;
            _P[4][17] = _P[17][12] * mR10 + _P[17][13] * mR11 + _P[17][14] * mR12 + _P[17][4] - _P[17][6] * dv2 + _P[17][8] * dv0;
            _P[5][17] = _P[17][12] * mR20 + _P[17][13] * mR21 + _P[17][14] * mR22 + _P[17][15] * _dt + _P[17][5] + _P[17][6] * dv1 - _P[17][7] * dv0;
            _P[6][17] = _P[17][10] * mR01 + _P[17][11] * mR02 + _P[17][6] + _P[17][9] * mR00;
            _P[7][17] = _P[17][10] * mR11 + _P[17][11] * mR12 + _P[17][7] + _P[17][9] * mR10;
            _P[8][17] = _P[17][10] * mR21 + _P[17][11] * mR22 + _P[17][8] + _P[17][9] * mR20;
            _P[9][17] = _P[17][9] * gam_b_del_ang;
            _P[10][17] = _P[17][10] * gam_b_del_ang;
            _P[11][17] = _P[17][11] * gam_b_del_ang;
            _P[12][17] = _P[17][12] * gam_b_del_vel;
            _P[13][17] = _P[17][13] * gam_b_del_vel;
            _P[14][17] = _P[17][14] * gam_b_del_vel;
            _P[15][17] = _P[17][15];
            _P[16][17] = _P[17][16];
            _P[17][17] = _P[17][17];
            _P[0][18] = _P[18][0] + _P[18][3] * _dt;
            _P[1][18] = _P[18][1] + _P[18][4] * _dt;
            _P[2][18] = _P[18][2] + _P[18][5] * _dt;
            _P[3][18] = _P[18][12] * mR00 + _P[18][13] * mR01 + _P[18][14] * mR02 + _P[18][3] + _P[18][7] * dv2 - _P[18][8] * dv1;
            _P[4][18] = _P[18][12] * mR10 + _P[18][13] * mR11 + _P[18][14] * mR12 + _P[18][4] - _P[18][6] * dv2 + _P[18][8] * dv0;
            _P[5][18] = _P[18][12] * mR20 + _P[18][13] * mR21 + _P[18][14] * mR22 + _P[18][15] * _dt + _P[18][5] + _P[18][6] * dv1 - _P[18][7] * dv0;
            _P[6][18] = _P[18][10] * mR01 + _P[18][11] * mR02 + _P[18][6] + _P[18][9] * mR00;
            _P[7][18] = _P[18][10] * mR11 + _P[18][11] * mR12 + _P[18][7] + _P[18][9] * mR10;
            _P[8][18] = _P[18][10] * mR21 + _P[18][11] * mR22 + _P[18][8] + _P[18][9] * mR20;
            _P[9][18] = _P[18][9] * gam_b_del_ang;
            _P[10][18] = _P[18][10] * gam_b_del_ang;
            _P[11][18] = _P[18][11] * gam_b_del_ang;
            _P[12][18] = _P[18][12] * gam_b_del_vel;
            _P[13][18] = _P[18][13] * gam_b_del_vel;
            _P[14][18] = _P[18][14] * gam_b_del_vel;
            _P[15][18] = _P[18][15];
            _P[16][18] = _P[18][16];
            _P[17][18] = _P[18][17];
            _P[18][18] = _P[18][18];

            float proc_var_mag_ang = sq(_params.mag_ang_proc_noise) * _dt2;
            _P[17][17] = kahan_summation(_P[17][17], proc_var_mag_ang, _accumulator[17]);
            _P[18][18] = kahan_summation(_P[18][18], proc_var_mag_ang, _accumulator[18]);
        }

        if (_control_status.flags.mag_bias) {
            _P[0][19] = gam_b_mag * (_P[19][0] + _P[19][3] * _dt);
            _P[1][19] = gam_b_mag * (_P[19][1] + _P[19][4] * _dt);
            _P[2][19] = gam_b_mag * (_P[19][2] + _P[19][5] * _dt);
            _P[3][19] = gam_b_mag * (_P[19][12] * mR00 + _P[19][13] * mR01 + _P[19][14] * mR02 + _P[19][3] + _P[19][7] * dv2 - _P[19][8] * dv1);
            _P[4][19] = gam_b_mag * (_P[19][12] * mR10 + _P[19][13] * mR11 + _P[19][14] * mR12 + _P[19][4] - _P[19][6] * dv2 + _P[19][8] * dv0);
            _P[5][19] = gam_b_mag * (_P[19][12] * mR20 + _P[19][13] * mR21 + _P[19][14] * mR22 + _P[19][15] * _dt + _P[19][5] + _P[19][6] * dv1 - _P[19][7] * dv0);
            _P[6][19] = gam_b_mag * (_P[19][10] * mR01 + _P[19][11] * mR02 + _P[19][6] + _P[19][9] * mR00);
            _P[7][19] = gam_b_mag * (_P[19][10] * mR11 + _P[19][11] * mR12 + _P[19][7] + _P[19][9] * mR10);
            _P[8][19] = gam_b_mag * (_P[19][10] * mR21 + _P[19][11] * mR22 + _P[19][8] + _P[19][9] * mR20);
            _P[9][19] = _P[19][9] * var[92];
            _P[10][19] = _P[19][10] * var[92];
            _P[11][19] = _P[19][11] * var[92];
            _P[12][19] = _P[19][12] * var[95];
            _P[13][19] = _P[19][13] * var[95];
            _P[14][19] = _P[19][14] * var[95];
            _P[15][19] = _P[19][15] * gam_b_mag;
            _P[16][19] = _P[19][16] * gam_b_mag;
            _P[17][19] = _P[19][17] * gam_b_mag;
            _P[18][19] = _P[19][18] * gam_b_mag;
            _P[19][19] = _P[19][19] * var[97];
            _P[0][20] = gam_b_mag * (_P[20][0] + _P[20][3] * _dt);
            _P[1][20] = gam_b_mag * (_P[20][1] + _P[20][4] * _dt);
            _P[2][20] = gam_b_mag * (_P[20][2] + _P[20][5] * _dt);
            _P[3][20] = gam_b_mag * (_P[20][12] * mR00 + _P[20][13] * mR01 + _P[20][14] * mR02 + _P[20][3] + _P[20][7] * dv2 - _P[20][8] * dv1);
            _P[4][20] = gam_b_mag * (_P[20][12] * mR10 + _P[20][13] * mR11 + _P[20][14] * mR12 + _P[20][4] - _P[20][6] * dv2 + _P[20][8] * dv0);
            _P[5][20] = gam_b_mag * (_P[20][12] * mR20 + _P[20][13] * mR21 + _P[20][14] * mR22 + _P[20][15] * _dt + _P[20][5] + _P[20][6] * dv1 - _P[20][7] * dv0);
            _P[6][20] = gam_b_mag * (_P[20][10] * mR01 + _P[20][11] * mR02 + _P[20][6] + _P[20][9] * mR00);
            _P[7][20] = gam_b_mag * (_P[20][10] * mR11 + _P[20][11] * mR12 + _P[20][7] + _P[20][9] * mR10);
            _P[8][20] = gam_b_mag * (_P[20][10] * mR21 + _P[20][11] * mR22 + _P[20][8] + _P[20][9] * mR20);
            _P[9][20] = _P[20][9] * var[92];
            _P[10][20] = _P[20][10] * var[92];
            _P[11][20] = _P[20][11] * var[92];
            _P[12][20] = _P[20][12] * var[95];
            _P[13][20] = _P[20][13] * var[95];
            _P[14][20] = _P[20][14] * var[95];
            _P[15][20] = _P[20][15] * gam_b_mag;
            _P[16][20] = _P[20][16] * gam_b_mag;
            _P[17][20] = _P[20][17] * gam_b_mag;
            _P[18][20] = _P[20][18] * gam_b_mag;
            _P[19][20] = _P[20][19] * var[97];
            _P[20][20] = _P[20][20] * var[97];
            _P[0][21] = gam_b_mag * (_P[21][0] + _P[21][3] * _dt);
            _P[1][21] = gam_b_mag * (_P[21][1] + _P[21][4] * _dt);
            _P[2][21] = gam_b_mag * (_P[21][2] + _P[21][5] * _dt);
            _P[3][21] = gam_b_mag * (_P[21][12] * mR00 + _P[21][13] * mR01 + _P[21][14] * mR02 + _P[21][3] + _P[21][7] * dv2 - _P[21][8] * dv1);
            _P[4][21] = gam_b_mag * (_P[21][12] * mR10 + _P[21][13] * mR11 + _P[21][14] * mR12 + _P[21][4] - _P[21][6] * dv2 + _P[21][8] * dv0);
            _P[5][21] = gam_b_mag * (_P[21][12] * mR20 + _P[21][13] * mR21 + _P[21][14] * mR22 + _P[21][15] * _dt + _P[21][5] + _P[21][6] * dv1 - _P[21][7] * dv0);
            _P[6][21] = gam_b_mag * (_P[21][10] * mR01 + _P[21][11] * mR02 + _P[21][6] + _P[21][9] * mR00);
            _P[7][21] = gam_b_mag * (_P[21][10] * mR11 + _P[21][11] * mR12 + _P[21][7] + _P[21][9] * mR10);
            _P[8][21] = gam_b_mag * (_P[21][10] * mR21 + _P[21][11] * mR22 + _P[21][8] + _P[21][9] * mR20);
            _P[9][21] = _P[21][9] * var[92];
            _P[10][21] = _P[21][10] * var[92];
            _P[11][21] = _P[21][11] * var[92];
            _P[12][21] = _P[21][12] * var[95];
            _P[13][21] = _P[21][13] * var[95];
            _P[14][21] = _P[21][14] * var[95];
            _P[15][21] = _P[21][15] * gam_b_mag;
            _P[16][21] = _P[21][16] * gam_b_mag;
            _P[17][21] = _P[21][17] * gam_b_mag;
            _P[18][21] = _P[21][18] * gam_b_mag;
            _P[19][21] = _P[21][19] * var[97];
            _P[20][21] = _P[21][20] * var[97];
            _P[21][21] = _P[21][21] * var[97];

            float proc_var_mag_bias = sq(_params.mag_bias_proc_noise) * _dt2;
            _P[19][19] = kahan_summation(_P[19][19], proc_var_mag_bias, _accumulator[19]);
            _P[20][20] = kahan_summation(_P[20][20], proc_var_mag_bias, _accumulator[20]);
            _P[21][21] = kahan_summation(_P[21][21], proc_var_mag_bias, _accumulator[21]);
        }

        if (_control_status.flags.wind) {
            _P[0][22] = gam_w * (_P[22][0] + _P[22][3] * _dt);
            _P[1][22] = gam_w * (_P[22][1] + _P[22][4] * _dt);
            _P[2][22] = gam_w * (_P[22][2] + _P[22][5] * _dt);
            _P[3][22] = gam_w * (_P[22][12] * mR00 + _P[22][13] * mR01 + _P[22][14] * mR02 + _P[22][3] + _P[22][7] * dv2 - _P[22][8] * dv1);
            _P[4][22] = gam_w * (_P[22][12] * mR10 + _P[22][13] * mR11 + _P[22][14] * mR12 + _P[22][4] - _P[22][6] * dv2 + _P[22][8] * dv0);
            _P[5][22] = gam_w * (_P[22][12] * mR20 + _P[22][13] * mR21 + _P[22][14] * mR22 + _P[22][15] * _dt + _P[22][5] + _P[22][6] * dv1 - _P[22][7] * dv0);
            _P[6][22] = gam_w * (_P[22][10] * mR01 + _P[22][11] * mR02 + _P[22][6] + _P[22][9] * mR00);
            _P[7][22] = gam_w * (_P[22][10] * mR11 + _P[22][11] * mR12 + _P[22][7] + _P[22][9] * mR10);
            _P[8][22] = gam_w * (_P[22][10] * mR21 + _P[22][11] * mR22 + _P[22][8] + _P[22][9] * mR20);
            _P[9][22] = _P[22][9] * var[93];
            _P[10][22] = _P[22][10] * var[93];
            _P[11][22] = _P[22][11] * var[93];
            _P[12][22] = _P[22][12] * var[96];
            _P[13][22] = _P[22][13] * var[96];
            _P[14][22] = _P[22][14] * var[96];
            _P[15][22] = _P[22][15] * gam_w;
            _P[16][22] = _P[22][16] * gam_w;
            _P[17][22] = _P[22][17] * gam_w;
            _P[18][22] = _P[22][18] * gam_w;
            _P[19][22] = _P[22][19] * var[98];
            _P[20][22] = _P[22][20] * var[98];
            _P[21][22] = _P[22][21] * var[98];
            _P[22][22] = _P[22][22] * var[99];
            _P[0][23] = gam_w * (_P[23][0] + _P[23][3] * _dt);
            _P[1][23] = gam_w * (_P[23][1] + _P[23][4] * _dt);
            _P[2][23] = gam_w * (_P[23][2] + _P[23][5] * _dt);
            _P[3][23] = gam_w * (_P[23][12] * mR00 + _P[23][13] * mR01 + _P[23][14] * mR02 + _P[23][3] + _P[23][7] * dv2 - _P[23][8] * dv1);
            _P[4][23] = gam_w * (_P[23][12] * mR10 + _P[23][13] * mR11 + _P[23][14] * mR12 + _P[23][4] - _P[23][6] * dv2 + _P[23][8] * dv0);
            _P[5][23] = gam_w * (_P[23][12] * mR20 + _P[23][13] * mR21 + _P[23][14] * mR22 + _P[23][15] * _dt + _P[23][5] + _P[23][6] * dv1 - _P[23][7] * dv0);
            _P[6][23] = gam_w * (_P[23][10] * mR01 + _P[23][11] * mR02 + _P[23][6] + _P[23][9] * mR00);
            _P[7][23] = gam_w * (_P[23][10] * mR11 + _P[23][11] * mR12 + _P[23][7] + _P[23][9] * mR10);
            _P[8][23] = gam_w * (_P[23][10] * mR21 + _P[23][11] * mR22 + _P[23][8] + _P[23][9] * mR20);
            _P[9][23] = _P[23][9] * var[93];
            _P[10][23] = _P[23][10] * var[93];
            _P[11][23] = _P[23][11] * var[93];
            _P[12][23] = _P[23][12] * var[96];
            _P[13][23] = _P[23][13] * var[96];
            _P[14][23] = _P[23][14] * var[96];
            _P[15][23] = _P[23][15] * gam_w;
            _P[16][23] = _P[23][16] * gam_w;
            _P[17][23] = _P[23][17] * gam_w;
            _P[18][23] = _P[23][18] * gam_w;
            _P[19][23] = _P[23][19] * var[98];
            _P[20][23] = _P[23][20] * var[98];
            _P[21][23] = _P[23][21] * var[98];
            _P[22][23] = _P[23][22] * var[99];
            _P[23][23] = _P[23][23] * var[99];

            float proc_var_wind = sq(_params.wind_proc_noise) * _dt2;
            _P[22][22] = kahan_summation(_P[22][22], proc_var_wind, _accumulator[22]);
            _P[23][23] = kahan_summation(_P[23][23], proc_var_wind, _accumulator[23]);
        }

//        regular_covariance_to_symmetric<DIM>(0);
        constrain_covariance();
    }

    uint8_t GESKF::fuse_pos_horz(const Vector2f &pos, const Vector3f &offset_body, const Vector3f &offset_nav,
                                 const float &gate, const float &noise_std, FuseData<2> &fuse_data) {
        uint8_t flag = 0;
        float HP[DIM];

        const float &offset_nav_x = offset_nav(0);
        const float &offset_nav_y = offset_nav(1);
        const float &offset_nav_z = offset_nav(2);

        // x轴, H = [O, I, -(R*dis)^, O, O, O, O, O, O, O]
        HP[0] = _P[0][0] + _P[0][7] * offset_nav_z - _P[0][8] * offset_nav_y;
        HP[1] = _P[0][1] + _P[1][7] * offset_nav_z - _P[1][8] * offset_nav_y;
        HP[2] = _P[0][2] + _P[2][7] * offset_nav_z - _P[2][8] * offset_nav_y;
        HP[3] = _P[0][3] + _P[3][7] * offset_nav_z - _P[3][8] * offset_nav_y;
        HP[4] = _P[0][4] + _P[4][7] * offset_nav_z - _P[4][8] * offset_nav_y;
        HP[5] = _P[0][5] + _P[5][7] * offset_nav_z - _P[5][8] * offset_nav_y;
        HP[6] = _P[0][6] + _P[6][7] * offset_nav_z - _P[6][8] * offset_nav_y;
        HP[7] = _P[0][7] + _P[7][7] * offset_nav_z - _P[7][8] * offset_nav_y;
        HP[8] = _P[0][8] + _P[7][8] * offset_nav_z - _P[8][8] * offset_nav_y;
        HP[9] = _P[0][9] + _P[7][9] * offset_nav_z - _P[8][9] * offset_nav_y;
        HP[10] = _P[0][10] + _P[7][10] * offset_nav_z - _P[8][10] * offset_nav_y;
        HP[11] = _P[0][11] + _P[7][11] * offset_nav_z - _P[8][11] * offset_nav_y;

        if (_control_status.flags.acc_x_bias) {
            HP[12] = _P[0][12] + _P[7][12] * offset_nav_z - _P[8][12] * offset_nav_y;
        } else {
            HP[12] = 0.f;
        }

        if (_control_status.flags.acc_y_bias) {
            HP[13] = _P[0][13] + _P[7][13] * offset_nav_z - _P[8][13] * offset_nav_y;
        } else {
            HP[13] = 0.f;
        }

        if (_control_status.flags.acc_z_bias) {
            HP[14] = _P[0][14] + _P[7][14] * offset_nav_z - _P[8][14] * offset_nav_y;
        } else {
            HP[14] = 0.f;
        }

        if (_control_status.flags.grav) {
            HP[15] = _P[0][15] + _P[7][15] * offset_nav_z - _P[8][15] * offset_nav_y;
        } else {
            HP[15] = 0.f;
        }

        if (_control_status.flags.mag_norm) {
            HP[16] = _P[0][16] + _P[7][16] * offset_nav_z - _P[8][16] * offset_nav_y;
        } else {
            HP[16] = 0.f;
        }

        if (_control_status.flags.mag_ang) {
            HP[17] = _P[0][17] + _P[7][17] * offset_nav_z - _P[8][17] * offset_nav_y;
            HP[18] = _P[0][18] + _P[7][18] * offset_nav_z - _P[8][18] * offset_nav_y;
        } else {
            HP[17] = 0.f;
            HP[18] = 0.f;
        }

        if (_control_status.flags.mag_bias) {
            HP[19] = _P[0][19] + _P[7][19] * offset_nav_z - _P[8][19] * offset_nav_y;
            HP[20] = _P[0][20] + _P[7][20] * offset_nav_z - _P[8][20] * offset_nav_y;
            HP[21] = _P[0][21] + _P[7][21] * offset_nav_z - _P[8][21] * offset_nav_y;
        } else {
            HP[19] = 0.f;
            HP[20] = 0.f;
            HP[21] = 0.f;
        }

        if (_control_status.flags.wind) {
            HP[22] = _P[0][22] + _P[7][22] * offset_nav_z - _P[8][22] * offset_nav_y;
            HP[23] = _P[0][23] + _P[7][23] * offset_nav_z - _P[8][23] * offset_nav_y;
        } else {
            HP[22] = 0.f;
            HP[23] = 0.f;
        }

        // H = [O, I, -(R*dis)^, O, O, O, O, O, O]
        fuse_data.innov_var(0) = HP[0] + HP[7] * offset_nav_z - HP[8] * offset_nav_y + sq(noise_std);

        // _P + R * dis
        fuse_data.innov(0) = pos(0) - (_state.pos(0) + offset_nav_x);

        fuse_data.test_ratio(0) = sq(fuse_data.innov(0) / gate) / fuse_data.innov_var(0);
        if (fuse_data.test_ratio(0) > 1.f) {
            flag |= 1;
        } else {
            if (!posterior_estimate(HP, fuse_data.innov_var(0), fuse_data.innov(0))) {
                flag |= 2;
            }
        }

        // y轴
        // H = [O, I, -(R*dis)^, O, O, O]
        HP[0] = _P[0][1] + _P[0][8] * offset_nav_x - _P[0][6] * offset_nav_z;
        HP[1] = _P[1][1] + _P[1][8] * offset_nav_x - _P[1][6] * offset_nav_z;
        HP[2] = _P[1][2] + _P[2][8] * offset_nav_x - _P[2][6] * offset_nav_z;
        HP[3] = _P[1][3] + _P[3][8] * offset_nav_x - _P[3][6] * offset_nav_z;
        HP[4] = _P[1][4] + _P[4][8] * offset_nav_x - _P[4][6] * offset_nav_z;
        HP[5] = _P[1][5] + _P[5][8] * offset_nav_x - _P[5][6] * offset_nav_z;
        HP[6] = _P[1][6] + _P[6][8] * offset_nav_x - _P[6][6] * offset_nav_z;
        HP[7] = _P[1][7] + _P[7][8] * offset_nav_x - _P[6][7] * offset_nav_z;
        HP[8] = _P[1][8] + _P[8][8] * offset_nav_x - _P[6][8] * offset_nav_z;
        HP[9] = _P[1][9] + _P[8][9] * offset_nav_x - _P[6][9] * offset_nav_z;
        HP[10] = _P[1][10] + _P[8][10] * offset_nav_x - _P[6][10] * offset_nav_z;
        HP[11] = _P[1][11] + _P[8][11] * offset_nav_x - _P[6][11] * offset_nav_z;

        if (_control_status.flags.acc_x_bias) {
            HP[12] = _P[1][12] + _P[8][12] * offset_nav_x - _P[6][12] * offset_nav_z;
        } else {
            HP[12] = 0.f;
        }

        if (_control_status.flags.acc_y_bias) {
            HP[13] = _P[1][13] + _P[8][13] * offset_nav_x - _P[6][13] * offset_nav_z;
        } else {
            HP[13] = 0.f;
        }

        if (_control_status.flags.acc_z_bias) {
            HP[14] = _P[1][14] + _P[8][14] * offset_nav_x - _P[6][14] * offset_nav_z;
        } else {
            HP[14] = 0.f;
        }

        if (_control_status.flags.grav) {
            HP[15] = _P[1][15] + _P[8][15] * offset_nav_x - _P[6][15] * offset_nav_z;
        } else {
            HP[15] = 0.f;
        }

        if (_control_status.flags.mag_norm) {
            HP[16] = _P[1][16] + _P[8][16] * offset_nav_x - _P[6][16] * offset_nav_z;
        } else {
            HP[16] = 0.f;
        }

        if (_control_status.flags.mag_ang) {
            HP[17] = _P[1][17] + _P[8][17] * offset_nav_x - _P[6][17] * offset_nav_z;
            HP[18] = _P[1][18] + _P[8][18] * offset_nav_x - _P[6][18] * offset_nav_z;
        } else {
            HP[17] = 0.f;
            HP[18] = 0.f;
        }

        if (_control_status.flags.mag_bias) {
            HP[19] = _P[1][19] + _P[8][19] * offset_nav_x - _P[6][19] * offset_nav_z;
            HP[20] = _P[1][20] + _P[8][20] * offset_nav_x - _P[6][20] * offset_nav_z;
            HP[21] = _P[1][21] + _P[8][21] * offset_nav_x - _P[6][21] * offset_nav_z;
        } else {
            HP[19] = 0.f;
            HP[20] = 0.f;
            HP[21] = 0.f;
        }

        if (_control_status.flags.wind) {
            HP[22] = _P[1][22] + _P[8][22] * offset_nav_x - _P[6][22] * offset_nav_z;
            HP[23] = _P[1][23] + _P[8][23] * offset_nav_x - _P[6][23] * offset_nav_z;
        } else {
            HP[22] = 0.f;
            HP[23] = 0.f;
        }

        // H = [O, I, -(R*dis)^, O, O, O]
        fuse_data.innov_var(1) = HP[1] + HP[8] * offset_nav_x - HP[6] * offset_nav_z + sq(noise_std);

        // p + R * dis
        fuse_data.innov(1) = pos(1) - (_state.pos(1) + offset_nav_y);

        fuse_data.test_ratio(1) = sq(fuse_data.innov(1) / gate) / fuse_data.innov_var(1);
        if (fuse_data.test_ratio(1) > 1.f) {
            flag |= 4;
        } else {
            if (!posterior_estimate(HP, fuse_data.innov_var(1), fuse_data.innov(1))) {
                flag |= 8;
            }
        }

        regular_covariance_to_symmetric<DIM>(0);

        return flag;
    }

    uint8_t GESKF::fuse_pos_vert(const float &pos, const Vector3f &offset_body, const Vector3f &offset_nav, const float &gate, const float &noise_std, FuseData<1> &fuse_data) {
        uint8_t flag = 0;
        float HP[DIM];

        const float &offset_nav_x = offset_nav(0);
        const float &offset_nav_y = offset_nav(1);
        const float &offset_nav_z = offset_nav(2);

        // z轴, H = [O, I, -(R*dis)^, O, O, O, O, O, O]
        HP[0] = _P[0][2] + _P[0][6] * offset_nav_y - _P[0][7] * offset_nav_x;
        HP[1] = _P[1][2] + _P[1][6] * offset_nav_y - _P[1][7] * offset_nav_x;
        HP[2] = _P[2][2] + _P[2][6] * offset_nav_y - _P[2][7] * offset_nav_x;
        HP[3] = _P[2][3] + _P[3][6] * offset_nav_y - _P[3][7] * offset_nav_x;
        HP[4] = _P[2][4] + _P[4][6] * offset_nav_y - _P[4][7] * offset_nav_x;
        HP[5] = _P[2][5] + _P[5][6] * offset_nav_y - _P[5][7] * offset_nav_x;
        HP[6] = _P[2][6] + _P[6][6] * offset_nav_y - _P[6][7] * offset_nav_x;
        HP[7] = _P[2][7] + _P[6][7] * offset_nav_y - _P[7][7] * offset_nav_x;
        HP[8] = _P[2][8] + _P[6][8] * offset_nav_y - _P[7][8] * offset_nav_x;
        HP[9] = _P[2][9] + _P[6][9] * offset_nav_y - _P[7][9] * offset_nav_x;
        HP[10] = _P[2][10] + _P[6][10] * offset_nav_y - _P[7][10] * offset_nav_x;
        HP[11] = _P[2][11] + _P[6][11] * offset_nav_y - _P[7][11] * offset_nav_x;

        if (_control_status.flags.acc_x_bias) {
            HP[12] = _P[2][12] + _P[6][12] * offset_nav_y - _P[7][12] * offset_nav_x;
        } else {
            HP[12] = 0.f;
        }

        if (_control_status.flags.acc_y_bias) {
            HP[13] = _P[2][13] + _P[6][13] * offset_nav_y - _P[7][13] * offset_nav_x;
        } else {
            HP[13] = 0.f;
        }

        if (_control_status.flags.acc_z_bias) {
            HP[14] = _P[2][14] + _P[6][14] * offset_nav_y - _P[7][14] * offset_nav_x;
        } else {
            HP[14] = 0.f;
        }

        if (_control_status.flags.grav) {
            HP[15] = _P[2][15] + _P[6][15] * offset_nav_y - _P[7][15] * offset_nav_x;
        } else {
            HP[15] = 0.f;
        }

        if (_control_status.flags.mag_norm) {
            HP[16] = _P[2][16] + _P[6][16] * offset_nav_y - _P[7][16] * offset_nav_x;
        } else {
            HP[16] = 0.f;
        }

        if (_control_status.flags.mag_ang) {
            HP[17] = _P[2][17] + _P[6][17] * offset_nav_y - _P[7][17] * offset_nav_x;
            HP[18] = _P[2][18] + _P[6][18] * offset_nav_y - _P[7][18] * offset_nav_x;
        } else {
            HP[17] = 0.f;
            HP[18] = 0.f;
        }

        if (_control_status.flags.mag_bias) {
            HP[19] = _P[2][19] + _P[6][19] * offset_nav_y - _P[7][19] * offset_nav_x;
            HP[20] = _P[2][20] + _P[6][20] * offset_nav_y - _P[7][20] * offset_nav_x;
            HP[21] = _P[2][21] + _P[6][21] * offset_nav_y - _P[7][21] * offset_nav_x;
        } else {
            HP[19] = 0.f;
            HP[20] = 0.f;
            HP[21] = 0.f;
        }

        if (_control_status.flags.wind) {
            HP[22] = _P[2][22] + _P[6][22] * offset_nav_y - _P[7][22] * offset_nav_x;
            HP[23] = _P[2][23] + _P[6][23] * offset_nav_y - _P[7][23] * offset_nav_x;
        } else {
            HP[22] = 0.f;
            HP[23] = 0.f;
        }

        // H = [O, I, -(R*dis)^, O, O, O, O, O, O]
        fuse_data.innov_var(0) = HP[2] + HP[6] * offset_nav_y - HP[7] * offset_nav_x + sq(noise_std);

        // p + R * dis
        fuse_data.innov(0) = pos - (_state.pos(2) + offset_nav_z);

        fuse_data.test_ratio(0) = sq(fuse_data.innov(0) / gate) / fuse_data.innov_var(0);
        if (fuse_data.test_ratio(0) > 1.f) {
            flag |= 1;
        } else {
            if (!posterior_estimate(HP, fuse_data.innov_var(0), fuse_data.innov(0))) {
                flag |= 2;
            }
        }

        regular_covariance_to_symmetric<DIM>(0);

        return flag;
    }

    uint8_t GESKF::fuse_vel_horz(const Vector2f &vel, const Vector3f &offset_body, const Vector3f &offset_nav, const Vector3f &w_cross_offset_body, const Vector3f &w_cross_offset_nav, const float &gate, const float &noise_std, FuseData<2> &fuse_data) {
        uint8_t flag = 0;
        float HP[DIM];

        const float &v_nav_x = w_cross_offset_nav(0);
        const float &v_nav_y = w_cross_offset_nav(1);
        const float &v_nav_z = w_cross_offset_nav(2);

        // x轴, H = [O, I, -(R*w^dis)^, R*dis^, O, O]
        float Hx = (_Rnb(0, 1) * offset_body(2) - _Rnb(0, 2) * offset_body(1)) / _dt;
        float Hy = (_Rnb(0, 2) * offset_body(0) - _Rnb(0, 0) * offset_body(2)) / _dt;
        float Hz = (_Rnb(0, 0) * offset_body(1) - _Rnb(0, 1) * offset_body(0)) / _dt;

        HP[0] = _P[0][3] + _P[0][7] * v_nav_z - _P[0][8] * v_nav_y + _P[0][9] * Hx + _P[0][10] * Hy + _P[0][11] * Hz;
        HP[1] = _P[1][3] + _P[1][7] * v_nav_z - _P[1][8] * v_nav_y + _P[1][9] * Hx + _P[1][10] * Hy + _P[1][11] * Hz;
        HP[2] = _P[2][3] + _P[2][7] * v_nav_z - _P[2][8] * v_nav_y + _P[2][9] * Hx + _P[2][10] * Hy + _P[2][11] * Hz;
        HP[3] = _P[3][3] + _P[3][7] * v_nav_z - _P[3][8] * v_nav_y + _P[3][9] * Hx + _P[3][10] * Hy + _P[3][11] * Hz;
        HP[4] = _P[3][4] + _P[4][7] * v_nav_z - _P[4][8] * v_nav_y + _P[4][9] * Hx + _P[4][10] * Hy + _P[4][11] * Hz;
        HP[5] = _P[3][5] + _P[5][7] * v_nav_z - _P[5][8] * v_nav_y + _P[5][9] * Hx + _P[5][10] * Hy + _P[5][11] * Hz;
        HP[6] = _P[3][6] + _P[6][7] * v_nav_z - _P[6][8] * v_nav_y + _P[6][9] * Hx + _P[6][10] * Hy + _P[6][11] * Hz;
        HP[7] = _P[3][7] + _P[7][7] * v_nav_z - _P[7][8] * v_nav_y + _P[7][9] * Hx + _P[7][10] * Hy + _P[7][11] * Hz;
        HP[8] = _P[3][8] + _P[7][8] * v_nav_z - _P[8][8] * v_nav_y + _P[8][9] * Hx + _P[8][10] * Hy + _P[8][11] * Hz;
        HP[9] = _P[3][9] + _P[7][9] * v_nav_z - _P[8][9] * v_nav_y + _P[9][9] * Hx + _P[9][10] * Hy + _P[9][11] * Hz;
        HP[10] = _P[3][10] + _P[7][10] * v_nav_z - _P[8][10] * v_nav_y + _P[9][10] * Hx + _P[10][10] * Hy + _P[10][11] * Hz;
        HP[11] = _P[3][11] + _P[7][11] * v_nav_z - _P[8][11] * v_nav_y + _P[9][11] * Hx + _P[10][11] * Hy + _P[11][11] * Hz;

        if (_control_status.flags.acc_x_bias) {
            HP[12] = _P[3][12] + _P[7][12] * v_nav_z - _P[8][12] * v_nav_y + _P[9][12] * Hx + _P[10][12] * Hy + _P[11][12] * Hz;
        } else {
            HP[12] = 0.f;
        }

        if (_control_status.flags.acc_y_bias) {
            HP[13] = _P[3][13] + _P[7][13] * v_nav_z - _P[8][13] * v_nav_y + _P[9][13] * Hx + _P[10][13] * Hy + _P[11][13] * Hz;
        } else {
            HP[13] = 0.f;
        }

        if (_control_status.flags.acc_z_bias) {
            HP[14] = _P[3][14] + _P[7][14] * v_nav_z - _P[8][14] * v_nav_y + _P[9][14] * Hx + _P[10][14] * Hy + _P[11][14] * Hz;
        } else {
            HP[14] = 0.f;
        }

        if (_control_status.flags.grav) {
            HP[15] = _P[3][15] + _P[7][15] * v_nav_z - _P[8][15] * v_nav_y + _P[9][15] * Hx + _P[10][15] * Hy + _P[11][15] * Hz;
        } else {
            HP[15] = 0.f;
        }

        if (_control_status.flags.mag_norm) {
            HP[16] = _P[3][16] + _P[7][16] * v_nav_z - _P[8][16] * v_nav_y + _P[9][16] * Hx + _P[10][16] * Hy + _P[11][16] * Hz;
        } else {
            HP[16] = 0.f;
        }

        if (_control_status.flags.mag_ang) {
            HP[17] = _P[3][17] + _P[7][17] * v_nav_z - _P[8][17] * v_nav_y + _P[9][17] * Hx + _P[10][17] * Hy + _P[11][17] * Hz;
            HP[18] = _P[3][18] + _P[7][18] * v_nav_z - _P[8][18] * v_nav_y + _P[9][18] * Hx + _P[10][18] * Hy + _P[11][18] * Hz;
        } else {
            HP[17] = 0.f;
            HP[18] = 0.f;
        }

        if (_control_status.flags.mag_bias) {
            HP[19] = _P[3][19] + _P[7][19] * v_nav_z - _P[8][19] * v_nav_y + _P[9][19] * Hx + _P[10][19] * Hy + _P[11][19] * Hz;
            HP[20] = _P[3][20] + _P[7][20] * v_nav_z - _P[8][20] * v_nav_y + _P[9][20] * Hx + _P[10][20] * Hy + _P[11][20] * Hz;
            HP[21] = _P[3][21] + _P[7][21] * v_nav_z - _P[8][21] * v_nav_y + _P[9][21] * Hx + _P[10][21] * Hy + _P[11][21] * Hz;
        } else {
            HP[19] = 0.f;
            HP[20] = 0.f;
            HP[21] = 0.f;
        }

        if (_control_status.flags.wind) {
            HP[22] = _P[3][22] + _P[7][22] * v_nav_z - _P[8][22] * v_nav_y + _P[9][22] * Hx + _P[10][22] * Hy + _P[11][22] * Hz;
            HP[23] = _P[3][23] + _P[7][23] * v_nav_z - _P[8][23] * v_nav_y + _P[9][23] * Hx + _P[10][23] * Hy + _P[11][23] * Hz;
        } else {
            HP[22] = 0.f;
            HP[23] = 0.f;
        }

        // H = [O, I, -(R*v_gyro_earth)^, R*dis^, O, O]
        fuse_data.innov_var(0) = HP[3] + HP[7] * v_nav_z - HP[8] * v_nav_y + HP[9] * Hx + HP[10] * Hy + HP[11] * Hz + sq(noise_std);

        // v + R * (w-bg)^ * dis
        fuse_data.innov(0) = vel(0) - (_state.vel(0) + v_nav_x);

        fuse_data.test_ratio(0) = sq(fuse_data.innov(0) / gate) / fuse_data.innov_var(0);
        if (fuse_data.test_ratio(0) > 1.f) {
            flag |= 1;
        } else {
            if (!posterior_estimate(HP, fuse_data.innov_var(0), fuse_data.innov(0))) {
                flag |= 2;
            }
        }

        // y轴, H = [O, I, -(R*w^dis)^, R*dis^, O, O]
        Hx = (_Rnb(1, 1) * offset_body(2) - _Rnb(1, 2) * offset_body(1)) / _dt;
        Hy = (_Rnb(1, 2) * offset_body(0) - _Rnb(1, 0) * offset_body(2)) / _dt;
        Hz = (_Rnb(1, 0) * offset_body(1) - _Rnb(1, 1) * offset_body(0)) / _dt;

        HP[0] = _P[0][4] + _P[0][8] * v_nav_x - _P[0][6] * v_nav_z + _P[0][9] * Hx + _P[0][10] * Hy + _P[0][11] * Hz;
        HP[1] = _P[1][4] + _P[1][8] * v_nav_x - _P[1][6] * v_nav_z + _P[1][9] * Hx + _P[1][10] * Hy + _P[1][11] * Hz;
        HP[2] = _P[2][4] + _P[2][8] * v_nav_x - _P[2][6] * v_nav_z + _P[2][9] * Hx + _P[2][10] * Hy + _P[2][11] * Hz;
        HP[3] = _P[3][4] + _P[3][8] * v_nav_x - _P[3][6] * v_nav_z + _P[3][9] * Hx + _P[3][10] * Hy + _P[3][11] * Hz;
        HP[4] = _P[4][4] + _P[4][8] * v_nav_x - _P[4][6] * v_nav_z + _P[4][9] * Hx + _P[4][10] * Hy + _P[4][11] * Hz;
        HP[5] = _P[4][5] + _P[5][8] * v_nav_x - _P[5][6] * v_nav_z + _P[5][9] * Hx + _P[5][10] * Hy + _P[5][11] * Hz;
        HP[6] = _P[4][6] + _P[6][8] * v_nav_x - _P[6][6] * v_nav_z + _P[6][9] * Hx + _P[6][10] * Hy + _P[6][11] * Hz;
        HP[7] = _P[4][7] + _P[7][8] * v_nav_x - _P[6][7] * v_nav_z + _P[7][9] * Hx + _P[7][10] * Hy + _P[7][11] * Hz;
        HP[8] = _P[4][8] + _P[8][8] * v_nav_x - _P[6][8] * v_nav_z + _P[8][9] * Hx + _P[8][10] * Hy + _P[8][11] * Hz;
        HP[9] = _P[4][9] + _P[8][9] * v_nav_x - _P[6][9] * v_nav_z + _P[9][9] * Hx + _P[9][10] * Hy + _P[9][11] * Hz;
        HP[10] = _P[4][10] + _P[8][10] * v_nav_x - _P[6][10] * v_nav_z + _P[9][10] * Hx + _P[10][10] * Hy + _P[10][11] * Hz;
        HP[11] = _P[4][11] + _P[8][11] * v_nav_x - _P[6][11] * v_nav_z + _P[9][11] * Hx + _P[10][11] * Hy + _P[11][11] * Hz;

        if (_control_status.flags.acc_x_bias) {
            HP[12] = _P[4][12] + _P[8][12] * v_nav_x - _P[6][12] * v_nav_z + _P[9][12] * Hx + _P[10][12] * Hy + _P[11][12] * Hz;
        } else {
            HP[12] = 0.f;
        }

        if (_control_status.flags.acc_y_bias) {
            HP[13] = _P[4][13] + _P[8][13] * v_nav_x - _P[6][13] * v_nav_z + _P[9][13] * Hx + _P[10][13] * Hy + _P[11][13] * Hz;
        } else {
            HP[13] = 0.f;
        }

        if (_control_status.flags.acc_z_bias) {
            HP[14] = _P[4][14] + _P[8][14] * v_nav_x - _P[6][14] * v_nav_z + _P[9][14] * Hx + _P[10][14] * Hy + _P[11][14] * Hz;
        } else {
            HP[14] = 0.f;
        }

        if (_control_status.flags.grav) {
            HP[15] = _P[4][15] + _P[8][15] * v_nav_x - _P[6][15] * v_nav_z + _P[9][15] * Hx + _P[10][15] * Hy + _P[11][15] * Hz;
        } else {
            HP[15] = 0.f;
        }

        if (_control_status.flags.mag_norm) {
            HP[16] = _P[4][16] + _P[8][16] * v_nav_x - _P[6][16] * v_nav_z + _P[9][16] * Hx + _P[10][16] * Hy + _P[11][16] * Hz;
        } else {
            HP[16] = 0.f;
        }

        if (_control_status.flags.mag_ang) {
            HP[17] = _P[4][17] + _P[8][17] * v_nav_x - _P[6][17] * v_nav_z + _P[9][17] * Hx + _P[10][17] * Hy + _P[11][17] * Hz;
            HP[18] = _P[4][18] + _P[8][18] * v_nav_x - _P[6][18] * v_nav_z + _P[9][18] * Hx + _P[10][18] * Hy + _P[11][18] * Hz;
        } else {
            HP[17] = 0.f;
            HP[18] = 0.f;
        }

        if (_control_status.flags.mag_bias) {
            HP[19] = _P[4][19] + _P[8][19] * v_nav_x - _P[6][19] * v_nav_z + _P[9][19] * Hx + _P[10][19] * Hy + _P[11][19] * Hz;
            HP[20] = _P[4][20] + _P[8][20] * v_nav_x - _P[6][20] * v_nav_z + _P[9][20] * Hx + _P[10][20] * Hy + _P[11][20] * Hz;
            HP[21] = _P[4][21] + _P[8][21] * v_nav_x - _P[6][21] * v_nav_z + _P[9][21] * Hx + _P[10][21] * Hy + _P[11][21] * Hz;
        } else {
            HP[19] = 0.f;
            HP[20] = 0.f;
            HP[21] = 0.f;
        }

        if (_control_status.flags.wind) {
            HP[22] = _P[4][22] + _P[8][22] * v_nav_x - _P[6][22] * v_nav_z + _P[9][22] * Hx + _P[10][22] * Hy + _P[11][22] * Hz;
            HP[23] = _P[4][23] + _P[8][23] * v_nav_x - _P[6][23] * v_nav_z + _P[9][23] * Hx + _P[10][23] * Hy + _P[11][23] * Hz;
        } else {
            HP[22] = 0.f;
            HP[23] = 0.f;
        }

        // H = [O, I, -(R*v_gyro_earth)^, R*dis^, O, O]
        fuse_data.innov_var(1) = HP[4] + HP[8] * v_nav_x - HP[6] * v_nav_z + HP[9] * Hx + HP[10] * Hy + HP[11] * Hz + sq(noise_std);

        // v + R * (w-bg)^ * dis
        fuse_data.innov(1) = vel(1) - (_state.vel(1) + v_nav_y);

        fuse_data.test_ratio(1) = sq(fuse_data.innov(1) / gate) / fuse_data.innov_var(1);
        if (fuse_data.test_ratio(1) > 1.f) {
            flag |= 4;
        } else {
            if (!posterior_estimate(HP, fuse_data.innov_var(1), fuse_data.innov(1))) {
                flag |= 8;
            }
        }

        regular_covariance_to_symmetric<DIM>(0);

        return flag;
    }

    uint8_t GESKF::fuse_vel_vert(const float &vel, const Vector3f &offset_body, const Vector3f &offset_nav, const Vector3f &w_cross_offset_body, const Vector3f &w_cross_offset_nav, const float &gate, const float &noise_std, FuseData<1> &fuse_data) {
        uint8_t flag = 0;
        float HP[DIM];

        const float &v_nav_x = w_cross_offset_nav(0);
        const float &v_nav_y = w_cross_offset_nav(1);
        const float &v_nav_z = w_cross_offset_nav(2);

        // H = [O, I, -(R*v_gyro_earth)^, R*dis^, O, O]
        float Hx = (_Rnb(2, 1) * offset_body(2) - _Rnb(2, 2) * offset_body(1)) / _dt;
        float Hy = (_Rnb(2, 2) * offset_body(0) - _Rnb(2, 0) * offset_body(2)) / _dt;
        float Hz = (_Rnb(2, 0) * offset_body(1) - _Rnb(2, 1) * offset_body(0)) / _dt;

        HP[0] = _P[0][5] + _P[0][6] * v_nav_y - _P[0][7] * v_nav_x + _P[0][9] * Hx + _P[0][10] * Hy + _P[0][11] * Hz;
        HP[1] = _P[1][5] + _P[1][6] * v_nav_y - _P[1][7] * v_nav_x + _P[1][9] * Hx + _P[1][10] * Hy + _P[1][11] * Hz;
        HP[2] = _P[2][5] + _P[2][6] * v_nav_y - _P[2][7] * v_nav_x + _P[2][9] * Hx + _P[2][10] * Hy + _P[2][11] * Hz;
        HP[3] = _P[3][5] + _P[3][6] * v_nav_y - _P[3][7] * v_nav_x + _P[3][9] * Hx + _P[3][10] * Hy + _P[3][11] * Hz;
        HP[4] = _P[4][5] + _P[4][6] * v_nav_y - _P[4][7] * v_nav_x + _P[4][9] * Hx + _P[4][10] * Hy + _P[4][11] * Hz;
        HP[5] = _P[5][5] + _P[5][6] * v_nav_y - _P[5][7] * v_nav_x + _P[5][9] * Hx + _P[5][10] * Hy + _P[5][11] * Hz;
        HP[6] = _P[5][6] + _P[6][6] * v_nav_y - _P[6][7] * v_nav_x + _P[6][9] * Hx + _P[6][10] * Hy + _P[6][11] * Hz;
        HP[7] = _P[5][7] + _P[6][7] * v_nav_y - _P[7][7] * v_nav_x + _P[7][9] * Hx + _P[7][10] * Hy + _P[7][11] * Hz;
        HP[8] = _P[5][8] + _P[6][8] * v_nav_y - _P[7][8] * v_nav_x + _P[8][9] * Hx + _P[8][10] * Hy + _P[8][11] * Hz;
        HP[9] = _P[5][9] + _P[6][9] * v_nav_y - _P[7][9] * v_nav_x + _P[9][9] * Hx + _P[9][10] * Hy + _P[9][11] * Hz;
        HP[10] = _P[5][10] + _P[6][10] * v_nav_y - _P[7][10] * v_nav_x + _P[9][10] * Hx + _P[10][10] * Hy + _P[10][11] * Hz;
        HP[11] = _P[5][11] + _P[6][11] * v_nav_y - _P[7][11] * v_nav_x + _P[9][11] * Hx + _P[10][11] * Hy + _P[11][11] * Hz;

        if (_control_status.flags.acc_x_bias) {
            HP[12] = _P[5][12] + _P[6][12] * v_nav_y - _P[7][12] * v_nav_x + _P[9][12] * Hx + _P[10][12] * Hy + _P[11][12] * Hz;
        } else {
            HP[12] = 0.f;
        }

        if (_control_status.flags.acc_y_bias) {
            HP[13] = _P[5][13] + _P[6][13] * v_nav_y - _P[7][13] * v_nav_x + _P[9][13] * Hx + _P[10][13] * Hy + _P[11][13] * Hz;
        } else {
            HP[13] = 0.f;
        }

        if (_control_status.flags.acc_z_bias) {
            HP[14] = _P[5][14] + _P[6][14] * v_nav_y - _P[7][14] * v_nav_x + _P[9][14] * Hx + _P[10][14] * Hy + _P[11][14] * Hz;
        } else {
            HP[14] = 0.f;
        }

        if (_control_status.flags.grav) {
            HP[15] = _P[5][15] + _P[6][15] * v_nav_y - _P[7][15] * v_nav_x + _P[9][15] * Hx + _P[10][15] * Hy + _P[11][15] * Hz;
        } else {
            HP[15] = 0.f;
        }

        if (_control_status.flags.mag_norm) {
            HP[16] = _P[5][16] + _P[6][16] * v_nav_y - _P[7][16] * v_nav_x + _P[9][16] * Hx + _P[10][16] * Hy + _P[11][16] * Hz;
        } else {
            HP[16] = 0.f;
        }

        if (_control_status.flags.mag_ang) {
            HP[17] = _P[5][17] + _P[6][17] * v_nav_y - _P[7][17] * v_nav_x + _P[9][17] * Hx + _P[10][17] * Hy + _P[11][17] * Hz;
            HP[18] = _P[5][18] + _P[6][18] * v_nav_y - _P[7][18] * v_nav_x + _P[9][18] * Hx + _P[10][18] * Hy + _P[11][18] * Hz;
        } else {
            HP[17] = 0.f;
            HP[18] = 0.f;
        }

        if (_control_status.flags.mag_bias) {
            HP[19] = _P[5][19] + _P[6][19] * v_nav_y - _P[7][19] * v_nav_x + _P[9][19] * Hx + _P[10][19] * Hy + _P[11][19] * Hz;
            HP[20] = _P[5][20] + _P[6][20] * v_nav_y - _P[7][20] * v_nav_x + _P[9][20] * Hx + _P[10][20] * Hy + _P[11][20] * Hz;
            HP[21] = _P[5][21] + _P[6][21] * v_nav_y - _P[7][21] * v_nav_x + _P[9][21] * Hx + _P[10][21] * Hy + _P[11][21] * Hz;
        } else {
            HP[19] = 0.f;
            HP[20] = 0.f;
            HP[21] = 0.f;
        }

        if (_control_status.flags.wind) {
            HP[22] = _P[5][22] + _P[6][22] * v_nav_y - _P[7][22] * v_nav_x + _P[9][22] * Hx + _P[10][22] * Hy + _P[11][22] * Hz;
            HP[23] = _P[5][23] + _P[6][23] * v_nav_y - _P[7][23] * v_nav_x + _P[9][23] * Hx + _P[10][23] * Hy + _P[11][23] * Hz;
        } else {
            HP[22] = 0.f;
            HP[23] = 0.f;
        }

        // H = [O, I, -(R*v_gyro_earth)^, R*dis^, O, O]
        fuse_data.innov_var(0) = HP[5] + HP[6] * v_nav_y - HP[7] * v_nav_x + HP[9] * Hx + HP[10] * Hy + HP[11] * Hz + sq(noise_std);

        // v + R * (w-bg)^ * dis
        fuse_data.innov(0) = vel - (_state.vel(2) + v_nav_z);

        fuse_data.test_ratio(0) = sq(fuse_data.innov(0) / gate) / fuse_data.innov_var(0);
        if (fuse_data.test_ratio(0) > 1.f) {
            flag |= 1;
        } else {
            if (!posterior_estimate(HP, fuse_data.innov_var(0), fuse_data.innov(0))) {
                flag |= 2;
            }
        }

        regular_covariance_to_symmetric<DIM>(0);

        return flag;
    }

    uint8_t GESKF::fuse_vel_body_horz(const Vector2f &vel, const Vector3f &offset_body, const Vector3f &offset_nav, const Vector3f &w_cross_offset_body, const Vector3f &w_cross_offset_nav, const float &gate, const float &noise_std, FuseData<2> &fuse_data) {
        uint8_t flag = 0;
        float HP[DIM];

        Vector3f vel_body = _Rnb.transpose() * _state.vel;

        // x轴
        float Hvx = _Rnb(0, 0), Hvy = _Rnb(1, 0), Hvz = _Rnb(2, 0);
        float Hax = _Rnb(1, 0) * _state.vel(2) - _Rnb(2, 0) * _state.vel(1);
        float Hay = _Rnb(2, 0) * _state.vel(0) - _Rnb(0, 0) * _state.vel(2);
        float Haz = _Rnb(0, 0) * _state.vel(1) - _Rnb(1, 0) * _state.vel(0);
        float Hbwx = 0.f, Hbwy = -offset_body(2) / _dt, Hbwz = offset_body(1) / _dt;

        HP[0] = _P[0][3] * Hvx + _P[0][4] * Hvy + _P[0][5] * Hvz + _P[0][6] * Hax + _P[0][7] * Hay + _P[0][8] * Haz + _P[0][10] * Hbwy + _P[0][11] * Hbwz;
        HP[1] = _P[1][3] * Hvx + _P[1][4] * Hvy + _P[1][5] * Hvz + _P[1][6] * Hax + _P[1][7] * Hay + _P[1][8] * Haz + _P[1][10] * Hbwy + _P[1][11] * Hbwz;
        HP[2] = _P[2][3] * Hvx + _P[2][4] * Hvy + _P[2][5] * Hvz + _P[2][6] * Hax + _P[2][7] * Hay + _P[2][8] * Haz + _P[2][10] * Hbwy + _P[2][11] * Hbwz;
        HP[3] = _P[3][3] * Hvx + _P[3][4] * Hvy + _P[3][5] * Hvz + _P[3][6] * Hax + _P[3][7] * Hay + _P[3][8] * Haz + _P[3][10] * Hbwy + _P[3][11] * Hbwz;
        HP[4] = _P[3][4] * Hvx + _P[4][4] * Hvy + _P[4][5] * Hvz + _P[4][6] * Hax + _P[4][7] * Hay + _P[4][8] * Haz + _P[4][10] * Hbwy + _P[4][11] * Hbwz;
        HP[5] = _P[3][5] * Hvx + _P[4][5] * Hvy + _P[5][5] * Hvz + _P[5][6] * Hax + _P[5][7] * Hay + _P[5][8] * Haz + _P[5][10] * Hbwy + _P[5][11] * Hbwz;
        HP[6] = _P[3][6] * Hvx + _P[4][6] * Hvy + _P[5][6] * Hvz + _P[6][6] * Hax + _P[6][7] * Hay + _P[6][8] * Haz + _P[6][10] * Hbwy + _P[6][11] * Hbwz;
        HP[7] = _P[3][7] * Hvx + _P[4][7] * Hvy + _P[5][7] * Hvz + _P[6][7] * Hax + _P[7][7] * Hay + _P[7][8] * Haz + _P[7][10] * Hbwy + _P[7][11] * Hbwz;
        HP[8] = _P[3][8] * Hvx + _P[4][8] * Hvy + _P[5][8] * Hvz + _P[6][8] * Hax + _P[7][8] * Hay + _P[8][8] * Haz + _P[8][10] * Hbwy + _P[8][11] * Hbwz;
        HP[9] = _P[3][9] * Hvx + _P[4][9] * Hvy + _P[5][9] * Hvz + _P[6][9] * Hax + _P[7][9] * Hay + _P[8][9] * Haz + _P[9][10] * Hbwy + _P[9][11] * Hbwz;
        HP[10] = _P[3][10] * Hvx + _P[4][10] * Hvy + _P[5][10] * Hvz + _P[6][10] * Hax + _P[7][10] * Hay + _P[8][10] * Haz + _P[10][10] * Hbwy + _P[10][11] * Hbwz;
        HP[11] = _P[3][11] * Hvx + _P[4][11] * Hvy + _P[5][11] * Hvz + _P[6][11] * Hax + _P[7][11] * Hay + _P[8][11] * Haz + _P[10][11] * Hbwy + _P[11][11] * Hbwz;

        if (_control_status.flags.acc_x_bias) {
            HP[12] = _P[3][12] * Hvx + _P[4][12] * Hvy + _P[5][12] * Hvz + _P[6][12] * Hax + _P[7][12] * Hay + _P[8][12] * Haz + _P[10][12] * Hbwy + _P[11][12] * Hbwz;
        } else {
            HP[12] = 0.f;
        }

        if (_control_status.flags.acc_y_bias) {
            HP[13] = _P[3][13] * Hvx + _P[4][13] * Hvy + _P[5][13] * Hvz + _P[6][13] * Hax + _P[7][13] * Hay + _P[8][13] * Haz + _P[10][13] * Hbwy + _P[11][13] * Hbwz;
        } else {
            HP[13] = 0.f;
        }

        if (_control_status.flags.acc_z_bias) {
            HP[14] = _P[3][14] * Hvx + _P[4][14] * Hvy + _P[5][14] * Hvz + _P[6][14] * Hax + _P[7][14] * Hay + _P[8][14] * Haz + _P[10][14] * Hbwy + _P[11][14] * Hbwz;
        } else {
            HP[14] = 0.f;
        }

        if (_control_status.flags.grav) {
            HP[15] = _P[3][15] * Hvx + _P[4][15] * Hvy + _P[5][15] * Hvz + _P[6][15] * Hax + _P[7][15] * Hay + _P[8][15] * Haz + _P[10][15] * Hbwy + _P[11][15] * Hbwz;
        } else {
            HP[15] = 0.f;
        }

        if (_control_status.flags.mag_norm) {
            HP[16] = _P[3][16] * Hvx + _P[4][16] * Hvy + _P[5][16] * Hvz + _P[6][16] * Hax + _P[7][16] * Hay + _P[8][16] * Haz + _P[10][16] * Hbwy + _P[11][16] * Hbwz;
        } else {
            HP[16] = 0.f;
        }

        if (_control_status.flags.mag_ang) {
            HP[17] = _P[3][17] * Hvx + _P[4][17] * Hvy + _P[5][17] * Hvz + _P[6][17] * Hax + _P[7][17] * Hay + _P[8][17] * Haz + _P[10][17] * Hbwy + _P[11][17] * Hbwz;
            HP[18] = _P[3][18] * Hvx + _P[4][18] * Hvy + _P[5][18] * Hvz + _P[6][18] * Hax + _P[7][18] * Hay + _P[8][18] * Haz + _P[10][18] * Hbwy + _P[11][18] * Hbwz;
        } else {
            HP[17] = 0.f;
            HP[18] = 0.f;
        }

        if (_control_status.flags.mag_bias) {
            HP[19] = _P[3][19] * Hvx + _P[4][19] * Hvy + _P[5][19] * Hvz + _P[6][19] * Hax + _P[7][19] * Hay + _P[8][19] * Haz + _P[10][19] * Hbwy + _P[11][19] * Hbwz;
            HP[20] = _P[3][20] * Hvx + _P[4][20] * Hvy + _P[5][20] * Hvz + _P[6][20] * Hax + _P[7][20] * Hay + _P[8][20] * Haz + _P[10][20] * Hbwy + _P[11][20] * Hbwz;
            HP[21] = _P[3][21] * Hvx + _P[4][21] * Hvy + _P[5][21] * Hvz + _P[6][21] * Hax + _P[7][21] * Hay + _P[8][21] * Haz + _P[10][21] * Hbwy + _P[11][21] * Hbwz;
        } else {
            HP[19] = 0.f;
            HP[20] = 0.f;
            HP[21] = 0.f;
        }

        if (_control_status.flags.wind) {
            HP[22] = _P[3][22] * Hvx + _P[4][22] * Hvy + _P[5][22] * Hvz + _P[6][22] * Hax + _P[7][22] * Hay + _P[8][22] * Haz + _P[10][22] * Hbwy + _P[11][22] * Hbwz;
            HP[23] = _P[3][23] * Hvx + _P[4][23] * Hvy + _P[5][23] * Hvz + _P[6][23] * Hax + _P[7][23] * Hay + _P[8][23] * Haz + _P[10][23] * Hbwy + _P[11][23] * Hbwz;
        } else {
            HP[22] = 0.f;
            HP[23] = 0.f;
        }

        // H * P * H' + R
        fuse_data.innov_var(0) = HP[3] * Hvx + HP[4] * Hvy + HP[5] * Hvz + HP[6] * Hax + HP[7] * Hay + HP[8] * Haz + HP[10] * Hbwy + HP[11] * Hbwz + sq(noise_std);

        // vb - (R'*v + w^d)
        fuse_data.innov(0) = vel(0) - (vel_body(0) + w_cross_offset_body(0));

        fuse_data.test_ratio(0) = sq(fuse_data.innov(0) / gate) / fuse_data.innov_var(0);
        if (fuse_data.test_ratio(0) > 1.f) {
            flag |= 1;
        } else {
            if (!posterior_estimate(HP, fuse_data.innov_var(0), fuse_data.innov(0))) {
                flag |= 2;
            }
        }

        // y轴
        Hvx = _Rnb(0, 1), Hvy = _Rnb(1, 1), Hvz = _Rnb(2, 1);
        Hax = _Rnb(1, 1) * _state.vel(2) - _Rnb(2, 1) * _state.vel(1);
        Hay = _Rnb(2, 1) * _state.vel(0) - _Rnb(0, 1) * _state.vel(2);
        Haz = _Rnb(0, 1) * _state.vel(1) - _Rnb(1, 1) * _state.vel(0);
        Hbwx = offset_body(2) / _dt, Hbwy = 0.f, Hbwz = -offset_body(0) / _dt;

        HP[0] = _P[0][3] * Hvx + _P[0][4] * Hvy + _P[0][5] * Hvz + _P[0][6] * Hax + _P[0][7] * Hay + _P[0][8] * Haz + _P[0][9] * Hbwx + _P[0][11] * Hbwz;
        HP[1] = _P[1][3] * Hvx + _P[1][4] * Hvy + _P[1][5] * Hvz + _P[1][6] * Hax + _P[1][7] * Hay + _P[1][8] * Haz + _P[1][9] * Hbwx + _P[1][11] * Hbwz;
        HP[2] = _P[2][3] * Hvx + _P[2][4] * Hvy + _P[2][5] * Hvz + _P[2][6] * Hax + _P[2][7] * Hay + _P[2][8] * Haz + _P[2][9] * Hbwx + _P[2][11] * Hbwz;
        HP[3] = _P[3][3] * Hvx + _P[3][4] * Hvy + _P[3][5] * Hvz + _P[3][6] * Hax + _P[3][7] * Hay + _P[3][8] * Haz + _P[3][9] * Hbwx + _P[3][11] * Hbwz;
        HP[4] = _P[3][4] * Hvx + _P[4][4] * Hvy + _P[4][5] * Hvz + _P[4][6] * Hax + _P[4][7] * Hay + _P[4][8] * Haz + _P[4][9] * Hbwx + _P[4][11] * Hbwz;
        HP[5] = _P[3][5] * Hvx + _P[4][5] * Hvy + _P[5][5] * Hvz + _P[5][6] * Hax + _P[5][7] * Hay + _P[5][8] * Haz + _P[5][9] * Hbwx + _P[5][11] * Hbwz;
        HP[6] = _P[3][6] * Hvx + _P[4][6] * Hvy + _P[5][6] * Hvz + _P[6][6] * Hax + _P[6][7] * Hay + _P[6][8] * Haz + _P[6][9] * Hbwx + _P[6][11] * Hbwz;
        HP[7] = _P[3][7] * Hvx + _P[4][7] * Hvy + _P[5][7] * Hvz + _P[6][7] * Hax + _P[7][7] * Hay + _P[7][8] * Haz + _P[7][9] * Hbwx + _P[7][11] * Hbwz;
        HP[8] = _P[3][8] * Hvx + _P[4][8] * Hvy + _P[5][8] * Hvz + _P[6][8] * Hax + _P[7][8] * Hay + _P[8][8] * Haz + _P[8][9] * Hbwx + _P[8][11] * Hbwz;
        HP[9] = _P[3][9] * Hvx + _P[4][9] * Hvy + _P[5][9] * Hvz + _P[6][9] * Hax + _P[7][9] * Hay + _P[8][9] * Haz + _P[9][9] * Hbwx + _P[9][11] * Hbwz;
        HP[10] = _P[3][10] * Hvx + _P[4][10] * Hvy + _P[5][10] * Hvz + _P[6][10] * Hax + _P[7][10] * Hay + _P[8][10] * Haz + _P[9][10] * Hbwx + _P[10][11] * Hbwz;
        HP[11] = _P[3][11] * Hvx + _P[4][11] * Hvy + _P[5][11] * Hvz + _P[6][11] * Hax + _P[7][11] * Hay + _P[8][11] * Haz + _P[9][11] * Hbwx + _P[11][11] * Hbwz;

        if (_control_status.flags.acc_x_bias) {
            HP[12] = _P[3][12] * Hvx + _P[4][12] * Hvy + _P[5][12] * Hvz + _P[6][12] * Hax + _P[7][12] * Hay + _P[8][12] * Haz + _P[9][12] * Hbwx + _P[11][12] * Hbwz;
        } else {
            HP[12] = 0.f;
        }

        if (_control_status.flags.acc_y_bias) {
            HP[13] = _P[3][13] * Hvx + _P[4][13] * Hvy + _P[5][13] * Hvz + _P[6][13] * Hax + _P[7][13] * Hay + _P[8][13] * Haz + _P[9][13] * Hbwx + _P[11][13] * Hbwz;
        } else {
            HP[13] = 0.f;
        }

        if (_control_status.flags.acc_z_bias) {
            HP[14] = _P[3][14] * Hvx + _P[4][14] * Hvy + _P[5][14] * Hvz + _P[6][14] * Hax + _P[7][14] * Hay + _P[8][14] * Haz + _P[9][14] * Hbwx + _P[11][14] * Hbwz;
        } else {
            HP[14] = 0.f;
        }

        if (_control_status.flags.grav) {
            HP[15] = _P[3][15] * Hvx + _P[4][15] * Hvy + _P[5][15] * Hvz + _P[6][15] * Hax + _P[7][15] * Hay + _P[8][15] * Haz + _P[9][15] * Hbwx + _P[11][15] * Hbwz;
        } else {
            HP[15] = 0.f;
        }

        if (_control_status.flags.mag_norm) {
            HP[16] = _P[3][16] * Hvx + _P[4][16] * Hvy + _P[5][16] * Hvz + _P[6][16] * Hax + _P[7][16] * Hay + _P[8][16] * Haz + _P[9][16] * Hbwx + _P[11][16] * Hbwz;
        } else {
            HP[16] = 0.f;
        }

        if (_control_status.flags.mag_ang) {
            HP[17] = _P[3][17] * Hvx + _P[4][17] * Hvy + _P[5][17] * Hvz + _P[6][17] * Hax + _P[7][17] * Hay + _P[8][17] * Haz + _P[9][17] * Hbwx + _P[11][17] * Hbwz;
            HP[18] = _P[3][18] * Hvx + _P[4][18] * Hvy + _P[5][18] * Hvz + _P[6][18] * Hax + _P[7][18] * Hay + _P[8][18] * Haz + _P[9][18] * Hbwx + _P[11][18] * Hbwz;
        } else {
            HP[17] = 0.f;
            HP[18] = 0.f;
        }

        if (_control_status.flags.mag_bias) {
            HP[19] = _P[3][19] * Hvx + _P[4][19] * Hvy + _P[5][19] * Hvz + _P[6][19] * Hax + _P[7][19] * Hay + _P[8][19] * Haz + _P[9][19] * Hbwx + _P[11][19] * Hbwz;
            HP[20] = _P[3][20] * Hvx + _P[4][20] * Hvy + _P[5][20] * Hvz + _P[6][20] * Hax + _P[7][20] * Hay + _P[8][20] * Haz + _P[9][20] * Hbwx + _P[11][20] * Hbwz;
            HP[21] = _P[3][21] * Hvx + _P[4][21] * Hvy + _P[5][21] * Hvz + _P[6][21] * Hax + _P[7][21] * Hay + _P[8][21] * Haz + _P[9][21] * Hbwx + _P[11][21] * Hbwz;
        } else {
            HP[19] = 0.f;
            HP[20] = 0.f;
            HP[21] = 0.f;
        }

        if (_control_status.flags.wind) {
            HP[22] = _P[3][22] * Hvx + _P[4][22] * Hvy + _P[5][22] * Hvz + _P[6][22] * Hax + _P[7][22] * Hay + _P[8][22] * Haz + _P[9][22] * Hbwx + _P[11][22] * Hbwz;
            HP[23] = _P[3][23] * Hvx + _P[4][23] * Hvy + _P[5][23] * Hvz + _P[6][23] * Hax + _P[7][23] * Hay + _P[8][23] * Haz + _P[9][23] * Hbwx + _P[11][23] * Hbwz;
        } else {
            HP[22] = 0.f;
            HP[23] = 0.f;
        }

        // H * P * H' + R
        fuse_data.innov_var(1) = HP[3] * Hvx + HP[4] * Hvy + HP[5] * Hvz + HP[6] * Hax + HP[7] * Hay + HP[8] * Haz + HP[9] * Hbwx + HP[11] * Hbwz + sq(noise_std);

        // vb - (R'*v + w^d)
        fuse_data.innov(1) = vel(1) - (vel_body(1) + w_cross_offset_body(1));

        fuse_data.test_ratio(1) = sq(fuse_data.innov(1) / gate) / fuse_data.innov_var(1);
        if (fuse_data.test_ratio(1) > 1.f) {
            flag |= 4;
        } else {
            if (!posterior_estimate(HP, fuse_data.innov_var(1), fuse_data.innov(1))) {
                flag |= 8;
            }
        }

        regular_covariance_to_symmetric<DIM>(0);

        return flag;

    }

    uint8_t GESKF::fuse_mag_norm(const float &mag_norm, const float &gate, const float &noise_std, FuseData<1> &fuse_data) {
        uint8_t flag = 0;
        float HP[DIM];

        /*
        K = P * H * (H * P * H' + R)^-1
        x = x + K * (y - h)
        P = (I - K * H) * P
        where, H = [O, O, O, O, O, O, [1, 0, 0], O, O
                    O, O, O, O, O, O, [0, 1, 0], O, O]
               obs = [mag_inc
                      mag_dec]
        */

        // H * P  or  P * H'
        HP[0] = _P[0][16];
        HP[1] = _P[1][16];
        HP[2] = _P[2][16];
        HP[3] = _P[3][16];
        HP[4] = _P[4][16];
        HP[5] = _P[5][16];
        HP[6] = _P[6][16];
        HP[7] = _P[7][16];
        HP[8] = _P[8][16];
        HP[9] = _P[9][16];
        HP[10] = _P[10][16];
        HP[11] = _P[11][16];

        if (_control_status.flags.acc_x_bias) {
            HP[12] = _P[12][16];
        } else {
            HP[12] = 0.f;
        }

        if (_control_status.flags.acc_y_bias) {
            HP[13] = _P[13][16];
        } else {
            HP[13] = 0.f;
        }

        if (_control_status.flags.acc_z_bias) {
            HP[14] = _P[14][16];
        } else {
            HP[14] = 0.f;
        }

        if (_control_status.flags.grav) {
            HP[15] = _P[15][16];
        } else {
            HP[15] = 0.f;
        }

        HP[16] = _P[16][16];

        if (_control_status.flags.mag_ang) {
            HP[17] = _P[16][17];
            HP[18] = _P[16][18];
        } else {
            HP[17] = 0.f;
            HP[18] = 0.f;
        }

        if (_control_status.flags.mag_bias) {
            HP[19] = _P[16][19];
            HP[20] = _P[16][20];
            HP[21] = _P[16][21];
        } else {
            HP[19] = 0.f;
            HP[20] = 0.f;
            HP[21] = 0.f;
        }

        if (_control_status.flags.wind) {
            HP[22] = _P[16][22];
            HP[23] = _P[16][23];
        } else {
            HP[22] = 0.f;
            HP[23] = 0.f;
        }

        // H * P * H' + R
        fuse_data.innov_var(0) = HP[16] + sq(noise_std);

        // e = mag_norm_meas - mag_norm
        fuse_data.innov(0) = mag_norm - _state.mag_norm;

        fuse_data.test_ratio(0) = sq(fuse_data.innov(0) / gate) / fuse_data.innov_var(0);
        if (fuse_data.test_ratio(0) > 1.f) {
            flag |= 1;
        } else {
            if (!posterior_estimate(HP, fuse_data.innov_var(0), fuse_data.innov(0))) {
                flag |= 2;
            }
        }

        regular_covariance_to_symmetric<DIM>(0);

        return flag;
    }

    uint8_t GESKF::fuse_mag_ang(const Vector2f &mag_ang, const float &gate, const float &noise_std, FuseData<2> &fuse_data) {
        uint8_t flag = 0;
        float HP[DIM];

        for (uint8_t dim = 0; dim < 2; ++dim) {
            /*
                K = P * H * (H * P * H' + R)^-1
                x = x + K * (y - h)
                P = (I - K * H) * P
                where, H = [O, O, O, O, O, O, [0, 1, 0], O, O
                            O, O, O, O, O, O, [0, 0, 1], O, O]
                       obs = mag_dec
                */

            // H * P  or  P * H'
            const uint8_t index = 17 + dim;
            HP[0] = _P[0][index];
            HP[1] = _P[1][index];
            HP[2] = _P[2][index];
            HP[3] = _P[3][index];
            HP[4] = _P[4][index];
            HP[5] = _P[5][index];
            HP[6] = _P[6][index];
            HP[7] = _P[7][index];
            HP[8] = _P[8][index];
            HP[9] = _P[9][index];
            HP[10] = _P[10][index];
            HP[11] = _P[11][index];

            if (_control_status.flags.acc_x_bias) {
                HP[12] = _P[12][index];
            } else {
                HP[12] = 0.f;
            }

            if (_control_status.flags.acc_y_bias) {
                HP[13] = _P[13][index];
            } else {
                HP[13] = 0.f;
            }

            if (_control_status.flags.acc_z_bias) {
                HP[14] = _P[14][index];
            } else {
                HP[14] = 0.f;
            }

            if (_control_status.flags.grav) {
                HP[15] = _P[15][index];
            } else {
                HP[15] = 0.f;
            }

            if (_control_status.flags.mag_norm) {
                HP[16] = _P[16][index];
            } else {
                HP[16] = 0.f;
            }

            HP[17] = _P[17][index];
            HP[18] = _P[index][18];

            if (_control_status.flags.mag_bias) {
                HP[19] = _P[index][19];
                HP[20] = _P[index][20];
                HP[21] = _P[index][21];
            } else {
                HP[19] = 0.f;
                HP[20] = 0.f;
                HP[21] = 0.f;
            }

            if (_control_status.flags.wind) {
                HP[22] = _P[index][22];
                HP[23] = _P[index][23];
            } else {
                HP[22] = 0.f;
                HP[23] = 0.f;
            }

            // H * P * H' + R
            fuse_data.innov_var(dim) = HP[index] + sq(noise_std);

            // e = dec_meas - dec
            fuse_data.innov(dim) = wrap_pi(mag_ang(dim) - _state.mag_ang(dim));

            fuse_data.test_ratio(dim) = sq(fuse_data.innov(dim) / gate) / fuse_data.innov_var(dim);
            if (fuse_data.test_ratio(dim) > 1.f) {
                flag |= (1 << 2 * dim);
            } else {
                if (!posterior_estimate(HP, fuse_data.innov_var(dim), fuse_data.innov(dim))) {
                    flag |= (2 << 2 * dim);
                }
            }
        }

        regular_covariance_to_symmetric<DIM>(0);

        return flag;
    }

    uint8_t GESKF::fuse_mag_body(const Vector3f &mag_body, const float &gate, const float &noise_std, FuseData<3> &fuse_data) {
        uint8_t flag = 0;
        float HP[DIM];

        // Hb = y - bm
        const Vector3f mag_corr = mag_body - _state.mag_bias;

        const float cos_y = cosf(_state.mag_ang(0)), sin_y = sinf(_state.mag_ang(0));
        const float cos_z = cosf(_state.mag_ang(1)), sin_z = sinf(_state.mag_ang(1));

        // Rz * Ry * ex
        const float rz_ry_ex[3] = {cos_z * cos_y, sin_z * cos_y, -sin_y};

        // 1 / H
        const float h_inv = (_state.mag_norm < 1e-6f) ? 1.f / _params.mag_norm : 1.f / _state.mag_norm;

        for (uint8_t dim = 0; dim < 3; ++dim) {
            /*
            K = P * H * (H * P * H' + R)^-1
            x = x + K * (y - h)
            P = (I - K * H) * P
            where, H = [O, O, R'*m^, O, O, O, R', I, O]
                h = R' * m
            */

            // R' * RZ * RY * ex
            const float rt_rz_ry_ex = _Rnb(0, dim) * rz_ry_ex[0] + _Rnb(1, dim) * rz_ry_ex[1] + _Rnb(2, dim) * rz_ry_ex[2];

            // R' * RZ * RY * ex / H
            const float rt_rz_ry_ex_h_inv = rt_rz_ry_ex * h_inv;

            // R' * (RZ * RY * ex)^
            const float rt_rz_ry_ex_hat[3] = {
                    _Rnb(1, dim) * rz_ry_ex[2] - _Rnb(2, dim) * rz_ry_ex[1],
                    _Rnb(2, dim) * rz_ry_ex[0] - _Rnb(0, dim) * rz_ry_ex[2],
                    _Rnb(0, dim) * rz_ry_ex[1] - _Rnb(1, dim) * rz_ry_ex[0]
            };

            const float param_y = rt_rz_ry_ex_hat[1]*cos_z - sin_z*rt_rz_ry_ex_hat[0];

            // H * P  or  P * H'
            const uint8_t index = 19 + dim;

            HP[0] = _P[0][index] * h_inv + _P[0][6] * rt_rz_ry_ex_hat[0] + _P[0][7] * rt_rz_ry_ex_hat[1] + _P[0][8] * rt_rz_ry_ex_hat[2] + _P[0][16] * rt_rz_ry_ex_h_inv - _P[0][17] * param_y - _P[0][18] * rt_rz_ry_ex_hat[2];
            HP[1] = _P[1][index] * h_inv + _P[1][6] * rt_rz_ry_ex_hat[0] + _P[1][7] * rt_rz_ry_ex_hat[1] + _P[1][8] * rt_rz_ry_ex_hat[2] + _P[1][16] * rt_rz_ry_ex_h_inv - _P[1][17] * param_y - _P[1][18] * rt_rz_ry_ex_hat[2];
            HP[2] = _P[2][index] * h_inv + _P[2][6] * rt_rz_ry_ex_hat[0] + _P[2][7] * rt_rz_ry_ex_hat[1] + _P[2][8] * rt_rz_ry_ex_hat[2] + _P[2][16] * rt_rz_ry_ex_h_inv - _P[2][17] * param_y - _P[2][18] * rt_rz_ry_ex_hat[2];
            HP[3] = _P[3][index] * h_inv + _P[3][6] * rt_rz_ry_ex_hat[0] + _P[3][7] * rt_rz_ry_ex_hat[1] + _P[3][8] * rt_rz_ry_ex_hat[2] + _P[3][16] * rt_rz_ry_ex_h_inv - _P[3][17] * param_y - _P[3][18] * rt_rz_ry_ex_hat[2];
            HP[4] = _P[4][index] * h_inv + _P[4][6] * rt_rz_ry_ex_hat[0] + _P[4][7] * rt_rz_ry_ex_hat[1] + _P[4][8] * rt_rz_ry_ex_hat[2] + _P[4][16] * rt_rz_ry_ex_h_inv - _P[4][17] * param_y - _P[4][18] * rt_rz_ry_ex_hat[2];
            HP[5] = _P[5][index] * h_inv + _P[5][6] * rt_rz_ry_ex_hat[0] + _P[5][7] * rt_rz_ry_ex_hat[1] + _P[5][8] * rt_rz_ry_ex_hat[2] + _P[5][16] * rt_rz_ry_ex_h_inv - _P[5][17] * param_y - _P[5][18] * rt_rz_ry_ex_hat[2];
            HP[6] = _P[6][index] * h_inv + _P[6][6] * rt_rz_ry_ex_hat[0] + _P[6][7] * rt_rz_ry_ex_hat[1] + _P[6][8] * rt_rz_ry_ex_hat[2] + _P[6][16] * rt_rz_ry_ex_h_inv - _P[6][17] * param_y - _P[6][18] * rt_rz_ry_ex_hat[2];
            HP[7] = _P[7][index] * h_inv + _P[6][7] * rt_rz_ry_ex_hat[0] + _P[7][7] * rt_rz_ry_ex_hat[1] + _P[7][8] * rt_rz_ry_ex_hat[2] + _P[7][16] * rt_rz_ry_ex_h_inv - _P[7][17] * param_y - _P[7][18] * rt_rz_ry_ex_hat[2];
            HP[8] = _P[8][index] * h_inv + _P[6][8] * rt_rz_ry_ex_hat[0] + _P[7][8] * rt_rz_ry_ex_hat[1] + _P[8][8] * rt_rz_ry_ex_hat[2] + _P[8][16] * rt_rz_ry_ex_h_inv - _P[8][17] * param_y - _P[8][18] * rt_rz_ry_ex_hat[2];
            HP[9] = _P[9][index] * h_inv + _P[6][9] * rt_rz_ry_ex_hat[0] + _P[7][9] * rt_rz_ry_ex_hat[1] + _P[8][9] * rt_rz_ry_ex_hat[2] + _P[9][16] * rt_rz_ry_ex_h_inv - _P[9][17] * param_y - _P[9][18] * rt_rz_ry_ex_hat[2];
            HP[10] = _P[10][index] * h_inv + _P[6][10] * rt_rz_ry_ex_hat[0] + _P[7][10] * rt_rz_ry_ex_hat[1] + _P[8][10] * rt_rz_ry_ex_hat[2] + _P[10][16] * rt_rz_ry_ex_h_inv - _P[10][17] * param_y - _P[10][18] * rt_rz_ry_ex_hat[2];
            HP[11] = _P[11][index] * h_inv + _P[6][11] * rt_rz_ry_ex_hat[0] + _P[7][11] * rt_rz_ry_ex_hat[1] + _P[8][11] * rt_rz_ry_ex_hat[2] + _P[11][16] * rt_rz_ry_ex_h_inv - _P[11][17] * param_y - _P[11][18] * rt_rz_ry_ex_hat[2];

            if (_control_status.flags.acc_x_bias) {
                HP[12] = _P[12][index] * h_inv + _P[6][12] * rt_rz_ry_ex_hat[0] + _P[7][12] * rt_rz_ry_ex_hat[1] + _P[8][12] * rt_rz_ry_ex_hat[2] + _P[12][16] * rt_rz_ry_ex_h_inv - _P[12][17] * param_y - _P[12][18] * rt_rz_ry_ex_hat[2];
            } else {
                HP[12] = 0.f;
            }

            if (_control_status.flags.acc_y_bias) {
                HP[13] = _P[13][index] * h_inv + _P[6][13] * rt_rz_ry_ex_hat[0] + _P[7][13] * rt_rz_ry_ex_hat[1] + _P[8][13] * rt_rz_ry_ex_hat[2] + _P[13][16] * rt_rz_ry_ex_h_inv - _P[13][17] * param_y - _P[13][18] * rt_rz_ry_ex_hat[2];
            } else {
                HP[13] = 0.f;
            }

            if (_control_status.flags.acc_z_bias) {
                HP[14] = _P[14][index] * h_inv + _P[6][14] * rt_rz_ry_ex_hat[0] + _P[7][14] * rt_rz_ry_ex_hat[1] + _P[8][14] * rt_rz_ry_ex_hat[2] + _P[14][16] * rt_rz_ry_ex_h_inv - _P[14][17] * param_y - _P[14][18] * rt_rz_ry_ex_hat[2];
            } else {
                HP[14] = 0.f;
            }

            if (_control_status.flags.grav) {
                HP[15] = _P[15][index] * h_inv + _P[6][15] * rt_rz_ry_ex_hat[0] + _P[7][15] * rt_rz_ry_ex_hat[1] + _P[8][15] * rt_rz_ry_ex_hat[2] + _P[15][16] * rt_rz_ry_ex_h_inv - _P[15][17] * param_y - _P[15][18] * rt_rz_ry_ex_hat[2];
            } else {
                HP[15] = 0.f;
            }

            if (_control_status.flags.mag_norm) {
                HP[16] = _P[16][index] * h_inv + _P[6][16] * rt_rz_ry_ex_hat[0] + _P[7][16] * rt_rz_ry_ex_hat[1] + _P[8][16] * rt_rz_ry_ex_hat[2] + _P[16][16] * rt_rz_ry_ex_h_inv - _P[16][17] * param_y - _P[16][18] * rt_rz_ry_ex_hat[2];
            } else {
                HP[16] = 0.f;
            }

            if (_control_status.flags.mag_ang) {
                HP[17] = _P[17][index] * h_inv + _P[6][17] * rt_rz_ry_ex_hat[0] + _P[7][17] * rt_rz_ry_ex_hat[1] + _P[8][17] * rt_rz_ry_ex_hat[2] + _P[16][17] * rt_rz_ry_ex_h_inv - _P[17][17] * param_y - _P[17][18] * rt_rz_ry_ex_hat[2];
                HP[18] = _P[18][index] * h_inv + _P[6][18] * rt_rz_ry_ex_hat[0] + _P[7][18] * rt_rz_ry_ex_hat[1] + _P[8][18] * rt_rz_ry_ex_hat[2] + _P[16][18] * rt_rz_ry_ex_h_inv - _P[17][18] * param_y - _P[18][18] * rt_rz_ry_ex_hat[2];
            } else {
                HP[17] = 0.f;
                HP[18] = 0.f;
            }

            if (_control_status.flags.mag_bias) {
                const float cov_20_index = (dim == 2) ? _P[20][index] : _P[index][20];
                HP[19] = _P[19][index] * h_inv + _P[6][19] * rt_rz_ry_ex_hat[0] + _P[7][19] * rt_rz_ry_ex_hat[1] + _P[8][19] * rt_rz_ry_ex_hat[2] + _P[16][19] * rt_rz_ry_ex_h_inv - _P[17][19] * param_y - _P[18][19] * rt_rz_ry_ex_hat[2];
                HP[20] = cov_20_index * h_inv + _P[6][20] * rt_rz_ry_ex_hat[0] + _P[7][20] * rt_rz_ry_ex_hat[1] + _P[8][20] * rt_rz_ry_ex_hat[2] + _P[16][20] * rt_rz_ry_ex_h_inv - _P[17][20] * param_y - _P[18][20] * rt_rz_ry_ex_hat[2];
                HP[21] = _P[index][21] * h_inv + _P[6][21] * rt_rz_ry_ex_hat[0] + _P[7][21] * rt_rz_ry_ex_hat[1] + _P[8][21] * rt_rz_ry_ex_hat[2] + _P[16][21] * rt_rz_ry_ex_h_inv - _P[17][21] * param_y - _P[18][21] * rt_rz_ry_ex_hat[2];
            } else {
                HP[19] = 0.f;
                HP[20] = 0.f;
                HP[21] = 0.f;
            }

            if (_control_status.flags.wind) {
                HP[22] = _P[index][22] * h_inv + _P[6][22] * rt_rz_ry_ex_hat[0] + _P[7][22] * rt_rz_ry_ex_hat[1] + _P[8][22] * rt_rz_ry_ex_hat[2] + _P[16][22] * rt_rz_ry_ex_h_inv - _P[17][22] * param_y - _P[18][22] * rt_rz_ry_ex_hat[2];
                HP[23] = _P[index][23] * h_inv + _P[6][23] * rt_rz_ry_ex_hat[0] + _P[7][23] * rt_rz_ry_ex_hat[1] + _P[8][23] * rt_rz_ry_ex_hat[2] + _P[16][23] * rt_rz_ry_ex_h_inv - _P[17][23] * param_y - _P[18][23] * rt_rz_ry_ex_hat[2];
            } else {
                HP[22] = 0.f;
                HP[23] = 0.f;
            }

            // H * _P * H' + R
            fuse_data.innov_var(dim) = HP[index] * h_inv + HP[6] * rt_rz_ry_ex_hat[0] + HP[7] * rt_rz_ry_ex_hat[1] + HP[8] * rt_rz_ry_ex_hat[2] + HP[16] * rt_rz_ry_ex_h_inv - HP[17] * param_y - HP[18] * rt_rz_ry_ex_hat[2] + sq(noise_std);

            // h = m
            // e = (y - bm) - R' * m
            fuse_data.innov(dim) = mag_corr(dim) * h_inv - rt_rz_ry_ex;

            fuse_data.test_ratio(dim) = sq(fuse_data.innov(dim) / gate) / fuse_data.innov_var(dim);
            if (fuse_data.test_ratio(dim) > 1.f) {
                flag |= 1 << (2 * dim);
            } else {
                if (!posterior_estimate(HP, fuse_data.innov_var(dim), fuse_data.innov(dim))) {
                    flag |= 2 << (2 * dim);
                }
            }
        }

        regular_covariance_to_symmetric<DIM>(0);

        return flag;
    }

    uint8_t GESKF::fuse_flow(const Vector2f &flow, const Vector3f &offset_body, const Vector3f &offset_nav, const float &gate, const float &noise_std, FuseData<2> &fuse_data) {
        uint8_t flag = 0;
        float HP[DIM];

        // 杆臂产生的速度
        const Vector3f vel_rel_imu_body = _gyro_corr % offset_body;

        // 光流传感器相对于地球的速度, 投影到机体坐标系
        const Vector3f vel_body = _Rnb.transpose() * _state.vel + vel_rel_imu_body;

        /*
         * _imu_hgt 在ESKF::predict_state()中更新
         * */
//        // imu距地高度(向上为正)
//        _imu_hgt = _terrain_vpos - _state.pos(2);

        // 光流距地高度(向上为正)
        _flow_hgt = _imu_hgt - offset_nav(2);

        // 约束光流最小距地高度
        _flow_hgt = math::max(_flow_hgt, _params.rng_gnd_clearance);

        float range_inv = _Rnb(2, 2) / _flow_hgt;
        _flow_range = 1.f / range_inv;

        // 光流计算得到的速度
        _flow_vel_body(0) = -flow(1) / range_inv;
        _flow_vel_body(1) = flow(0) / range_inv;
        _flow_vel_nav(0) = _Rnb(0, 0) * _flow_vel_body(0) + _Rnb(0, 1) * _flow_vel_body(1);
        _flow_vel_nav(1) = _Rnb(1, 0) * _flow_vel_body(0) + _Rnb(1, 1) * _flow_vel_body(1);

        float Hz = vel_body(1) * range_inv / _flow_hgt;
        float Hvx = _Rnb(0, 1) * range_inv;
        float Hvy = _Rnb(1, 1) * range_inv;
        float Hvz = _Rnb(2, 1) * range_inv;
        float Hax = (vel_body(1) * (_Rnb(1, 2) + offset_nav(1) * range_inv)
                     + (_Rnb(1, 1) * _state.vel(2) - _Rnb(2, 1) * _state.vel(1)) * _Rnb(2, 2)
                     ) / _flow_hgt;
        float Hay = -(vel_body(1) * (_Rnb(0, 2) + offset_nav(0) * range_inv)
                      + (_Rnb(0, 1) * _state.vel(2) - _Rnb(2, 1) * _state.vel(0)) * _Rnb(2, 2)
                     ) / _flow_hgt;
        float Haz = (_Rnb(0, 1) * _state.vel(1) - _Rnb(1, 1) * _state.vel(0)) * range_inv;
        float Hdelta_ang_bias_x = (1.f + offset_body(2) * range_inv) / _dt;
        float Hdelta_ang_bias_y = 0.f;
        float Hdelta_ang_bias_z = -offset_body(0) * range_inv / _dt;

        HP[0] = _P[0][2] * Hz + _P[0][3] * Hvx + _P[0][4] * Hvy + _P[0][5] * Hvz + _P[0][6] * Hax + _P[0][7] * Hay + _P[0][8] * Haz + _P[0][9] * Hdelta_ang_bias_x + _P[0][11] * Hdelta_ang_bias_z;
        HP[1] = _P[1][2] * Hz + _P[1][3] * Hvx + _P[1][4] * Hvy + _P[1][5] * Hvz + _P[1][6] * Hax + _P[1][7] * Hay + _P[1][8] * Haz + _P[1][9] * Hdelta_ang_bias_x + _P[1][11] * Hdelta_ang_bias_z;
        HP[2] = _P[2][2] * Hz + _P[2][3] * Hvx + _P[2][4] * Hvy + _P[2][5] * Hvz + _P[2][6] * Hax + _P[2][7] * Hay + _P[2][8] * Haz + _P[2][9] * Hdelta_ang_bias_x + _P[2][11] * Hdelta_ang_bias_z;
        HP[3] = _P[2][3] * Hz + _P[3][3] * Hvx + _P[3][4] * Hvy + _P[3][5] * Hvz + _P[3][6] * Hax + _P[3][7] * Hay + _P[3][8] * Haz + _P[3][9] * Hdelta_ang_bias_x + _P[3][11] * Hdelta_ang_bias_z;
        HP[4] = _P[2][4] * Hz + _P[3][4] * Hvx + _P[4][4] * Hvy + _P[4][5] * Hvz + _P[4][6] * Hax + _P[4][7] * Hay + _P[4][8] * Haz + _P[4][9] * Hdelta_ang_bias_x + _P[4][11] * Hdelta_ang_bias_z;
        HP[5] = _P[2][5] * Hz + _P[3][5] * Hvx + _P[4][5] * Hvy + _P[5][5] * Hvz + _P[5][6] * Hax + _P[5][7] * Hay + _P[5][8] * Haz + _P[5][9] * Hdelta_ang_bias_x + _P[5][11] * Hdelta_ang_bias_z;
        HP[6] = _P[2][6] * Hz + _P[3][6] * Hvx + _P[4][6] * Hvy + _P[5][6] * Hvz + _P[6][6] * Hax + _P[6][7] * Hay + _P[6][8] * Haz + _P[6][9] * Hdelta_ang_bias_x + _P[6][11] * Hdelta_ang_bias_z;
        HP[7] = _P[2][7] * Hz + _P[3][7] * Hvx + _P[4][7] * Hvy + _P[5][7] * Hvz + _P[6][7] * Hax + _P[7][7] * Hay + _P[7][8] * Haz + _P[7][9] * Hdelta_ang_bias_x + _P[7][11] * Hdelta_ang_bias_z;
        HP[8] = _P[2][8] * Hz + _P[3][8] * Hvx + _P[4][8] * Hvy + _P[5][8] * Hvz + _P[6][8] * Hax + _P[7][8] * Hay + _P[8][8] * Haz + _P[8][9] * Hdelta_ang_bias_x + _P[8][11] * Hdelta_ang_bias_z;
        HP[9] = _P[2][9] * Hz + _P[3][9] * Hvx + _P[4][9] * Hvy + _P[5][9] * Hvz + _P[6][9] * Hax + _P[7][9] * Hay + _P[8][9] * Haz + _P[9][9] * Hdelta_ang_bias_x + _P[9][11] * Hdelta_ang_bias_z;
        HP[10] = _P[2][10] * Hz + _P[3][10] * Hvx + _P[4][10] * Hvy + _P[5][10] * Hvz + _P[6][10] * Hax + _P[7][10] * Hay + _P[8][10] * Haz + _P[9][10] * Hdelta_ang_bias_x + _P[10][11] * Hdelta_ang_bias_z;
        HP[11] = _P[2][11] * Hz + _P[3][11] * Hvx + _P[4][11] * Hvy + _P[5][11] * Hvz + _P[6][11] * Hax + _P[7][11] * Hay + _P[8][11] * Haz + _P[9][11] * Hdelta_ang_bias_x + _P[11][11] * Hdelta_ang_bias_z;

        if (_control_status.flags.acc_x_bias) {
            HP[12] = _P[2][12] * Hz + _P[3][12] * Hvx + _P[4][12] * Hvy + _P[5][12] * Hvz + _P[6][12] * Hax + _P[7][12] * Hay + _P[8][12] * Haz + _P[9][12] * Hdelta_ang_bias_x + _P[11][12] * Hdelta_ang_bias_z;
        } else {
            HP[12] = 0.f;
        }

        if (_control_status.flags.acc_y_bias) {
            HP[13] = _P[2][13] * Hz + _P[3][13] * Hvx + _P[4][13] * Hvy + _P[5][13] * Hvz + _P[6][13] * Hax + _P[7][13] * Hay + _P[8][13] * Haz + _P[9][13] * Hdelta_ang_bias_x + _P[11][13] * Hdelta_ang_bias_z;
        } else {
            HP[13] = 0.f;
        }

        if (_control_status.flags.acc_z_bias) {
            HP[14] = _P[2][14] * Hz + _P[3][14] * Hvx + _P[4][14] * Hvy + _P[5][14] * Hvz + _P[6][14] * Hax + _P[7][14] * Hay + _P[8][14] * Haz + _P[9][14] * Hdelta_ang_bias_x + _P[11][14] * Hdelta_ang_bias_z;
        } else {
            HP[14] = 0.f;
        }

        if (_control_status.flags.grav) {
            HP[15] = _P[2][15] * Hz + _P[3][15] * Hvx + _P[4][15] * Hvy + _P[5][15] * Hvz + _P[6][15] * Hax + _P[7][15] * Hay + _P[8][15] * Haz + _P[9][15] * Hdelta_ang_bias_x + _P[11][15] * Hdelta_ang_bias_z;
        } else {
            HP[15] = 0.f;
        }

        if (_control_status.flags.mag_norm) {
            HP[16] = _P[2][16] * Hz + _P[3][16] * Hvx + _P[4][16] * Hvy + _P[5][16] * Hvz + _P[6][16] * Hax + _P[7][16] * Hay + _P[8][16] * Haz + _P[9][16] * Hdelta_ang_bias_x + _P[11][16] * Hdelta_ang_bias_z;
        } else {
            HP[16] = 0.f;
        }

        if (_control_status.flags.mag_ang) {
            HP[17] = _P[2][17] * Hz + _P[3][17] * Hvx + _P[4][17] * Hvy + _P[5][17] * Hvz + _P[6][17] * Hax + _P[7][17] * Hay + _P[8][17] * Haz + _P[9][17] * Hdelta_ang_bias_x + _P[11][17] * Hdelta_ang_bias_z;
            HP[18] = _P[2][18] * Hz + _P[3][18] * Hvx + _P[4][18] * Hvy + _P[5][18] * Hvz + _P[6][18] * Hax + _P[7][18] * Hay + _P[8][18] * Haz + _P[9][18] * Hdelta_ang_bias_x + _P[11][18] * Hdelta_ang_bias_z;
        } else {
            HP[17] = 0.f;
            HP[18] = 0.f;
        }

        if (_control_status.flags.mag_bias) {
            HP[19] = _P[2][19] * Hz + _P[3][19] * Hvx + _P[4][19] * Hvy + _P[5][19] * Hvz + _P[6][19] * Hax + _P[7][19] * Hay + _P[8][19] * Haz + _P[9][19] * Hdelta_ang_bias_x + _P[11][19] * Hdelta_ang_bias_z;
            HP[20] = _P[2][20] * Hz + _P[3][20] * Hvx + _P[4][20] * Hvy + _P[5][20] * Hvz + _P[6][20] * Hax + _P[7][20] * Hay + _P[8][20] * Haz + _P[9][20] * Hdelta_ang_bias_x + _P[11][20] * Hdelta_ang_bias_z;
            HP[21] = _P[2][21] * Hz + _P[3][21] * Hvx + _P[4][21] * Hvy + _P[5][21] * Hvz + _P[6][21] * Hax + _P[7][21] * Hay + _P[8][21] * Haz + _P[9][21] * Hdelta_ang_bias_x + _P[11][21] * Hdelta_ang_bias_z;
        } else {
            HP[19] = 0.f;
            HP[20] = 0.f;
            HP[21] = 0.f;
        }

        if (_control_status.flags.wind) {
            HP[22] = _P[2][22] * Hz + _P[3][22] * Hvx + _P[4][22] * Hvy + _P[5][22] * Hvz + _P[6][22] * Hax + _P[7][22] * Hay + _P[8][22] * Haz + _P[9][22] * Hdelta_ang_bias_x + _P[11][22] * Hdelta_ang_bias_z;
            HP[23] = _P[2][23] * Hz + _P[3][23] * Hvx + _P[4][23] * Hvy + _P[5][23] * Hvz + _P[6][23] * Hax + _P[7][23] * Hay + _P[8][23] * Haz + _P[9][23] * Hdelta_ang_bias_x + _P[11][23] * Hdelta_ang_bias_z;
        } else {
            HP[22] = 0.f;
            HP[23] = 0.f;
        }

        // H * P * H' + R
        fuse_data.innov_var(0) = HP[2] * Hz + HP[3] * Hvx + HP[4] * Hvy + HP[5] * Hvz + HP[6] * Hax + HP[7] * Hay + HP[8] * Haz + HP[9]*Hdelta_ang_bias_x + HP[11]*Hdelta_ang_bias_z + sq(noise_std);

        fuse_data.innov(0) = flow(0) - vel_body(1) * range_inv;

        fuse_data.test_ratio(0) = sq(fuse_data.innov(0) / gate) / fuse_data.innov_var(0);
        if (fuse_data.test_ratio(0) > 1.f) {
            flag |= 1;
        } else {
            if (!posterior_estimate(HP, fuse_data.innov_var(0), fuse_data.innov(0))) {
                flag |= 2;
            }
        }

        Hz = -vel_body(0) * range_inv / _flow_hgt;
        Hvx = -_Rnb(0, 0) * range_inv;
        Hvy = -_Rnb(1, 0) * range_inv;
        Hvz = -_Rnb(2, 0) * range_inv;
        Hax = -(vel_body(0) * (_Rnb(1, 2) + offset_nav(1) * range_inv)
                + (_Rnb(1, 0) * _state.vel(2) - _Rnb(2, 0) * _state.vel(1)) * _Rnb(2, 2)
        ) / _flow_hgt;
        Hay = (vel_body(0) * (_Rnb(0, 2) + offset_nav(0) * range_inv)
               + (_Rnb(0, 0) * _state.vel(2) - _Rnb(2, 0) * _state.vel(0)) * _Rnb(2, 2)
              ) / _flow_hgt;
        Haz = -(_Rnb(0, 0) * _state.vel(1) - _Rnb(1, 0) * _state.vel(0)) * range_inv;
        Hdelta_ang_bias_x = 0.f;
        Hdelta_ang_bias_y = (1.f + offset_body(2) * range_inv) / _dt;
        Hdelta_ang_bias_z = -offset_body(0) * range_inv / _dt;

        HP[0] = _P[0][2] * Hz + _P[0][3] * Hvx + _P[0][4] * Hvy + _P[0][5] * Hvz + _P[0][6] * Hax + _P[0][7] * Hay + _P[0][8] * Haz + _P[0][10] * Hdelta_ang_bias_y + _P[0][11] * Hdelta_ang_bias_z;
        HP[1] = _P[1][2] * Hz + _P[1][3] * Hvx + _P[1][4] * Hvy + _P[1][5] * Hvz + _P[1][6] * Hax + _P[1][7] * Hay + _P[1][8] * Haz + _P[1][10] * Hdelta_ang_bias_y + _P[1][11] * Hdelta_ang_bias_z;
        HP[2] = _P[2][2] * Hz + _P[2][3] * Hvx + _P[2][4] * Hvy + _P[2][5] * Hvz + _P[2][6] * Hax + _P[2][7] * Hay + _P[2][8] * Haz + _P[2][10] * Hdelta_ang_bias_y + _P[2][11] * Hdelta_ang_bias_z;
        HP[3] = _P[2][3] * Hz + _P[3][3] * Hvx + _P[3][4] * Hvy + _P[3][5] * Hvz + _P[3][6] * Hax + _P[3][7] * Hay + _P[3][8] * Haz + _P[3][10] * Hdelta_ang_bias_y + _P[3][11] * Hdelta_ang_bias_z;
        HP[4] = _P[2][4] * Hz + _P[3][4] * Hvx + _P[4][4] * Hvy + _P[4][5] * Hvz + _P[4][6] * Hax + _P[4][7] * Hay + _P[4][8] * Haz + _P[4][10] * Hdelta_ang_bias_y + _P[4][11] * Hdelta_ang_bias_z;
        HP[5] = _P[2][5] * Hz + _P[3][5] * Hvx + _P[4][5] * Hvy + _P[5][5] * Hvz + _P[5][6] * Hax + _P[5][7] * Hay + _P[5][8] * Haz + _P[5][10] * Hdelta_ang_bias_y + _P[5][11] * Hdelta_ang_bias_z;
        HP[6] = _P[2][6] * Hz + _P[3][6] * Hvx + _P[4][6] * Hvy + _P[5][6] * Hvz + _P[6][6] * Hax + _P[6][7] * Hay + _P[6][8] * Haz + _P[6][10] * Hdelta_ang_bias_y + _P[6][11] * Hdelta_ang_bias_z;
        HP[7] = _P[2][7] * Hz + _P[3][7] * Hvx + _P[4][7] * Hvy + _P[5][7] * Hvz + _P[6][7] * Hax + _P[7][7] * Hay + _P[7][8] * Haz + _P[7][10] * Hdelta_ang_bias_y + _P[7][11] * Hdelta_ang_bias_z;
        HP[8] = _P[2][8] * Hz + _P[3][8] * Hvx + _P[4][8] * Hvy + _P[5][8] * Hvz + _P[6][8] * Hax + _P[7][8] * Hay + _P[8][8] * Haz + _P[8][10] * Hdelta_ang_bias_y + _P[8][11] * Hdelta_ang_bias_z;
        HP[9] = _P[2][9] * Hz + _P[3][9] * Hvx + _P[4][9] * Hvy + _P[5][9] * Hvz + _P[6][9] * Hax + _P[7][9] * Hay + _P[8][9] * Haz + _P[9][10] * Hdelta_ang_bias_y + _P[9][11] * Hdelta_ang_bias_z;
        HP[10] = _P[2][10] * Hz + _P[3][10] * Hvx + _P[4][10] * Hvy + _P[5][10] * Hvz + _P[6][10] * Hax + _P[7][10] * Hay + _P[8][10] * Haz + _P[10][10] * Hdelta_ang_bias_y + _P[10][11] * Hdelta_ang_bias_z;
        HP[11] = _P[2][11] * Hz + _P[3][11] * Hvx + _P[4][11] * Hvy + _P[5][11] * Hvz + _P[6][11] * Hax + _P[7][11] * Hay + _P[8][11] * Haz + _P[10][11] * Hdelta_ang_bias_y + _P[11][11] * Hdelta_ang_bias_z;

        if (_control_status.flags.acc_x_bias) {
            HP[12] = _P[2][12] * Hz + _P[3][12] * Hvx + _P[4][12] * Hvy + _P[5][12] * Hvz + _P[6][12] * Hax + _P[7][12] * Hay + _P[8][12] * Haz + _P[10][12] * Hdelta_ang_bias_y + _P[11][12] * Hdelta_ang_bias_z;
        } else {
            HP[12] = 0.f;
        }

        if (_control_status.flags.acc_y_bias) {
            HP[13] = _P[2][13] * Hz + _P[3][13] * Hvx + _P[4][13] * Hvy + _P[5][13] * Hvz + _P[6][13] * Hax + _P[7][13] * Hay + _P[8][13] * Haz + _P[10][13] * Hdelta_ang_bias_y + _P[11][13] * Hdelta_ang_bias_z;
        } else {
            HP[13] = 0.f;
        }

        if (_control_status.flags.acc_z_bias) {
            HP[14] = _P[2][14] * Hz + _P[3][14] * Hvx + _P[4][14] * Hvy + _P[5][14] * Hvz + _P[6][14] * Hax + _P[7][14] * Hay + _P[8][14] * Haz + _P[10][14] * Hdelta_ang_bias_y + _P[11][14] * Hdelta_ang_bias_z;
        } else {
            HP[14] = 0.f;
        }

        if (_control_status.flags.grav) {
            HP[15] = _P[2][15] * Hz + _P[3][15] * Hvx + _P[4][15] * Hvy + _P[5][15] * Hvz + _P[6][15] * Hax + _P[7][15] * Hay + _P[8][15] * Haz + _P[10][15] * Hdelta_ang_bias_y + _P[11][15] * Hdelta_ang_bias_z;
        } else {
            HP[15] = 0.f;
        }

        if (_control_status.flags.mag_norm) {
            HP[16] = _P[2][16] * Hz + _P[3][16] * Hvx + _P[4][16] * Hvy + _P[5][16] * Hvz + _P[6][16] * Hax + _P[7][16] * Hay + _P[8][16] * Haz + _P[10][16] * Hdelta_ang_bias_y + _P[11][16] * Hdelta_ang_bias_z;
        } else {
            HP[16] = 0.f;
        }

        if (_control_status.flags.mag_ang) {
            HP[17] = _P[2][17] * Hz + _P[3][17] * Hvx + _P[4][17] * Hvy + _P[5][17] * Hvz + _P[6][17] * Hax + _P[7][17] * Hay + _P[8][17] * Haz + _P[10][17] * Hdelta_ang_bias_y + _P[11][17] * Hdelta_ang_bias_z;
            HP[18] = _P[2][18] * Hz + _P[3][18] * Hvx + _P[4][18] * Hvy + _P[5][18] * Hvz + _P[6][18] * Hax + _P[7][18] * Hay + _P[8][18] * Haz + _P[10][18] * Hdelta_ang_bias_y + _P[11][18] * Hdelta_ang_bias_z;
        } else {
            HP[17] = 0.f;
            HP[18] = 0.f;
        }

        if (_control_status.flags.mag_bias) {
            HP[19] = _P[2][19] * Hz + _P[3][19] * Hvx + _P[4][19] * Hvy + _P[5][19] * Hvz + _P[6][19] * Hax + _P[7][19] * Hay + _P[8][19] * Haz + _P[10][19] * Hdelta_ang_bias_y + _P[11][19] * Hdelta_ang_bias_z;
            HP[20] = _P[2][20] * Hz + _P[3][20] * Hvx + _P[4][20] * Hvy + _P[5][20] * Hvz + _P[6][20] * Hax + _P[7][20] * Hay + _P[8][20] * Haz + _P[10][20] * Hdelta_ang_bias_y + _P[11][20] * Hdelta_ang_bias_z;
            HP[21] = _P[2][21] * Hz + _P[3][21] * Hvx + _P[4][21] * Hvy + _P[5][21] * Hvz + _P[6][21] * Hax + _P[7][21] * Hay + _P[8][21] * Haz + _P[10][21] * Hdelta_ang_bias_y + _P[11][21] * Hdelta_ang_bias_z;
        } else {
            HP[19] = 0.f;
            HP[20] = 0.f;
            HP[21] = 0.f;
        }

        if (_control_status.flags.wind) {
            HP[22] = _P[2][22] * Hz + _P[3][22] * Hvx + _P[4][22] * Hvy + _P[5][22] * Hvz + _P[6][22] * Hax + _P[7][22] * Hay + _P[8][22] * Haz + _P[10][22] * Hdelta_ang_bias_y + _P[11][22] * Hdelta_ang_bias_z;
            HP[23] = _P[2][23] * Hz + _P[3][23] * Hvx + _P[4][23] * Hvy + _P[5][23] * Hvz + _P[6][23] * Hax + _P[7][23] * Hay + _P[8][23] * Haz + _P[10][23] * Hdelta_ang_bias_y + _P[11][23] * Hdelta_ang_bias_z;
        } else {
            HP[22] = 0.f;
            HP[23] = 0.f;
        }

        // H * P * H' + R
        fuse_data.innov_var(1) = HP[2] * Hz + HP[3] * Hvx + HP[4] * Hvy + HP[5] * Hvz + HP[6] * Hax + HP[7] * Hay + HP[8] * Haz + HP[10]*Hdelta_ang_bias_y + HP[11]*Hdelta_ang_bias_z + sq(noise_std);

        fuse_data.innov(1) = flow(1) + vel_body(0) * range_inv;

        fuse_data.test_ratio(1) = sq(fuse_data.innov(1) / gate) / fuse_data.innov_var(1);
        if (fuse_data.test_ratio(1) > 1.f) {
            flag |= 4;
        } else {
            if (!posterior_estimate(HP, fuse_data.innov_var(1), fuse_data.innov(1))) {
                flag |= 8;
            }
        }

        regular_covariance_to_symmetric<DIM>(0);

        return flag;
    }

    uint8_t GESKF::fuse_range(const float &range, const Vector3f &offset_body, const Vector3f &offset_nav, const float &gate, const float &noise_std, FuseData<1> &fuse_data) {
        uint8_t flag = 0;
        float HP[DIM];

        static constexpr float cos_60deg = 0.5f;

        // 如果倾转角度大于60°, 则不进行融合
        if (_Rnb(2, 2) <  cos_60deg) {
            flag |= 4;
            return flag;
        }

        /*
         * _imu_hgt 在ESKF::predict_state()中更新
         * */
//        // IMU距离地面的高度
//        _imu_hgt = _terrain_vpos - _state.pos(2);

        // 传感器距地高度
        _range_hgt = _imu_hgt - offset_nav(2);

        // 拘束传感器距地高度的最小距离
        _range_hgt = math::max(_range_hgt, _params.rng_gnd_clearance);
        _range = _range_hgt / _Rnb(2, 2);

        const float Hx = -(offset_nav(1) + _range * _Rnb(1, 2)) / _Rnb(2, 2);
        const float Hy = (offset_nav(0) + _range * _Rnb(0, 2)) / _Rnb(2, 2);
        const float Hpz = -1.f / _Rnb(2, 2);

        HP[0] = _P[0][2] * Hpz + _P[0][6] * Hx + _P[0][7] * Hy;
        HP[1] = _P[1][2] * Hpz + _P[1][6] * Hx + _P[1][7] * Hy;
        HP[2] = _P[2][2] * Hpz + _P[2][6] * Hx + _P[2][7] * Hy;
        HP[3] = _P[2][3] * Hpz + _P[3][6] * Hx + _P[3][7] * Hy;
        HP[4] = _P[2][4] * Hpz + _P[4][6] * Hx + _P[4][7] * Hy;
        HP[5] = _P[2][5] * Hpz + _P[5][6] * Hx + _P[5][7] * Hy;
        HP[6] = _P[2][6] * Hpz + _P[6][6] * Hx + _P[6][7] * Hy;
        HP[7] = _P[2][7] * Hpz + _P[6][7] * Hx + _P[7][7] * Hy;
        HP[8] = _P[2][8] * Hpz + _P[6][8] * Hx + _P[7][8] * Hy;
        HP[9] = _P[2][9] * Hpz + _P[6][9] * Hx + _P[7][9] * Hy;
        HP[10] = _P[2][10] * Hpz + _P[6][10] * Hx + _P[7][10] * Hy;
        HP[11] = _P[2][11] * Hpz + _P[6][11] * Hx + _P[7][11] * Hy;

        if (_control_status.flags.acc_x_bias) {
            HP[12] = _P[2][12] * Hpz + _P[6][12] * Hx + _P[7][12] * Hy;
        } else {
            HP[12] = 0.f;
        }

        if (_control_status.flags.acc_y_bias) {
            HP[13] = _P[2][13] * Hpz + _P[6][13] * Hx + _P[7][13] * Hy;
        } else {
            HP[13] = 0.f;
        }

        if (_control_status.flags.acc_z_bias) {
            HP[14] = _P[2][14] * Hpz + _P[6][14] * Hx + _P[7][14] * Hy;
        } else {
            HP[14] = 0.f;
        }

        if (_control_status.flags.grav) {
            HP[15] = _P[2][15] * Hpz + _P[6][15] * Hx + _P[7][15] * Hy;
        } else {
            HP[15] = 0.f;
        }

        if (_control_status.flags.mag_norm) {
            HP[16] = _P[2][16] * Hpz + _P[6][16] * Hx + _P[7][16] * Hy;
        } else {
            HP[16] = 0.f;
        }

        if (_control_status.flags.mag_ang) {
            HP[17] = _P[2][17] * Hpz + _P[6][17] * Hx + _P[7][17] * Hy;
            HP[18] = _P[2][18] * Hpz + _P[6][18] * Hx + _P[7][18] * Hy;
        } else {
            HP[17] = 0.f;
            HP[18] = 0.f;
        }

        if (_control_status.flags.mag_bias) {
            HP[19] = _P[2][19] * Hpz + _P[6][19] * Hx + _P[7][19] * Hy;
            HP[20] = _P[2][20] * Hpz + _P[6][20] * Hx + _P[7][20] * Hy;
            HP[21] = _P[2][21] * Hpz + _P[6][21] * Hx + _P[7][21] * Hy;
        } else {
            HP[19] = 0.f;
            HP[20] = 0.f;
            HP[21] = 0.f;
        }

        if (_control_status.flags.wind) {
            HP[22] = _P[2][22] * Hpz + _P[6][22] * Hx + _P[7][22] * Hy;
            HP[23] = _P[2][23] * Hpz + _P[6][23] * Hx + _P[7][23] * Hy;
        } else {
            HP[22] = 0.f;
            HP[23] = 0.f;
        }

        fuse_data.innov_var(0) = HP[2] * Hpz + HP[6]*Hx + HP[7]*Hy + sq(noise_std);

        fuse_data.innov(0) = range - _range;

        fuse_data.test_ratio(0) = sq(fuse_data.innov(0) / gate) / fuse_data.innov_var(0);
        if (fuse_data.test_ratio(0) > 1.f) {
            flag |= 1;
        } else {
            if (!posterior_estimate(HP, fuse_data.innov_var(0), fuse_data.innov(0))) {
                flag |= 2;
            }
        }

        regular_covariance_to_symmetric<DIM>(0);

        return flag;
    }

    void GESKF::correct_state() {
        _state.pos += _error_state.pos;
        _state.vel += _error_state.vel;
        _state.delta_ang_bias += _error_state.delta_ang_bias;
        _state.delta_vel_bias += _error_state.delta_vel_bias;
        _state.grav += _error_state.grav;
        _state.mag_norm += _error_state.mag_norm;
        _state.mag_ang += _error_state.mag_ang;
        _state.mag_bias += _error_state.mag_bias;
        _state.wind += _error_state.wind;

        // q = Exp(δθ) * q
        Quatf delta_q;
        quaternion_from_axis_angle(delta_q, _error_state.ang);
        const Quatf q = _state.quat_nominal;
        _state.quat_nominal = delta_q * q;
        _state.quat_nominal.normalize();
        _Rnb = _state.quat_nominal;

//        std::cout << _error_state.ang(0) << ", " << _error_state.ang(1) << ", " << _error_state.ang(2) << std::endl;

        reset_error_state();
    }

    void GESKF::correct_covariance() {

    }
}