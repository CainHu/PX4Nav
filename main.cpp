//
// Created by Cain on 2022/12/26.
//

#include <iostream>
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

using namespace std;
using namespace matrix;

generator::Generator env;
vector<unsigned long> time_us_seq;

int main() {
    // 飞行数据生成
    float ts = 0.001;
    double ts_sim = 0.001;

    // 参数
    eskf::Parameters params;
    Vector3f gps_offset_left {-0.2f, -0.15f, -0.02f};
    Vector3f gps_offset_right {-0.2f, 0.15f, -0.02f};

    eskf::GESKF eskf_rtk(params);

    Vector3d euler0(0., 0., 0.), vn0(0., 0., 0.), pos0(23.1659394 * arcdeg, 113.4522718 * arcdeg, 20.);
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
    const Vector3d bias_acc(0.000, 0.000, 0.000);
    const Vector3d std_gyro(0.01, 0.01, 0.01);
    const Vector3d std_acc(0.1, 0.1, 0.1);
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

        pl_meas[i] = Vector3f(float(pn[i](0)), float(pn[i](1)), float(pn[i](2))) + R * dl + npl;
        vl_meas[i] = Vector3f(float(vn[i](0)), float(vn[i](1)), float(vn[i](2))) - R * (dl.cross(Vector3f(float(wm[i](0)), float(wm[i](1)), float(wm[i](2))) / ts)) + nvl;
        pr_meas[i] = Vector3f(float(pn[i](0)), float(pn[i](1)), float(pn[i](2))) + R * dr + npr;
        vr_meas[i] = Vector3f(float(vn[i](0)), float(vn[i](1)), float(vn[i](2))) - R * (dr.cross(Vector3f(float(wm[i](0)), float(wm[i](1)), float(wm[i](2))) / ts)) + nvr;
        w_meas[i] = Vector3f(float(wm_meas[i](0)), float(wm_meas[i](1)), float(wm_meas[i](2))) / ts;
        a_meas[i] = Vector3f(float(vm_meas[i](0)), float(vm_meas[i](1)), float(vm_meas[i](2))) / ts;
    }

    // 数据融合
    eskf_rtk.enable_estimation_acc_bias();
//    eskf_rtk.enable_estimation_gravity();
    // eskf_rtk.enable_estimation_magnet();
    // eskf_rtk.enable_estimation_declination();
    // eskf_rtk.enable_estimation_magnet_bias();
    eskf_rtk.disable_estimation_gravity();
    eskf_rtk.disable_estimation_mag_norm();
    eskf_rtk.disable_estimation_mag_ang();
    eskf_rtk.disable_estimation_mag_bias();
    eskf_rtk.disable_estimation_wind();
    eskf_rtk.initialize();

//    Quatd tmp(double(eskf_rtk.get_quaternion()(0)), double(eskf_rtk.get_quaternion()(1)), double(eskf_rtk.get_quaternion()(2)), double(eskf_rtk.get_quaternion()(3)));
//    Vector3d tmp1 = generator::quat2euler(tmp);
//
////    clock_t t1 = clock();
//    for (unsigned i = 0; i < pos.size(); ++i) {
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
//        imu_sample.delta_ang_dt = ts;
//        imu_sample.delta_vel_dt = ts;
//
//        eskf_rtk.predict_state(imu_sample);
//        eskf_rtk.predict_covariance(imu_sample);
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
//        cout << "pos_horz: " << pn[i](0) << ", " << pn[i](1) << ", " << pn[i](2) << ", ";
//        cout << p_hat[i](0) << ", " << p_hat[i](1) << ", " << p_hat[i](2) << endl;
////        cout << "bias_acc: " << bias_acc(0) << ", " << bias_acc(1) << ", " << bias_acc(2) << ", ";
////        cout << ba_hat[i](0) << ", " << ba_hat[i](1) << ", " << ba_hat[i](2) << endl;
////        cout << "bias_gyro: " << bias_gyro(0) << ", " << bias_gyro(1) << ", " << bias_gyro(2) << ", ";
////        cout << bg_hat[i](0) << ", " << bg_hat[i](1) << ", " << bg_hat[i](2) << endl;
////        cout << "ang_var: " << eskf_rtk.get_covariance_matrix()[6][6] << ", " << eskf_rtk.get_covariance_matrix()[7][7] << ", " << eskf_rtk.get_covariance_matrix()[8][8] << endl;
//
//    }
////    clock_t t2 = clock();
////    cout << t2 - t1 << endl;


    ahrs::AHRS nonCF(ts);

    eskf::ESKFRunner<1> eskf_runner(eskf_rtk);
    eskf_runner._NED_origin_initialised = true;
    eskf_runner._control_status.flags.tilt_align = true;
    eskf_runner._control_status.flags.yaw_align = true;
    eskf_runner._control_status.flags.gps_hgt = true;
    eskf_runner._control_status.flags.gps_horz = true;
    eskf_runner._control_status.flags.gps_vel = true;

    eskf::ImuSample imu_sample;
    eskf::GpsSample gps_sample[2];
    for (unsigned i = 0; i < pos.size(); ++i) {
        imu_sample.delta_ang = {float(wm_meas[i](0)), float(wm_meas[i](1)), float(wm_meas[i](2))};
        imu_sample.delta_vel = {float(vm_meas[i](0)), float(vm_meas[i](1)), float(vm_meas[i](2))};
        imu_sample.delta_ang_dt = ts;
        imu_sample.delta_vel_dt = ts;
        imu_sample.time_us += (uint64_t)(1e6f * ts);

        eskf_runner.set_imu_data(imu_sample);

//        std::cout << "step of eskf_runner = " << int(i) << std::endl;

        if (i % 10 == 0) {
            gps_sample[0].pos_horz = pl_meas[i].xy();
            gps_sample[0].hgt = -pl_meas[i](2);
            gps_sample[0].vel = vl_meas[i];
            gps_sample[0].fix_type = 5;
            gps_sample[0].time_us = imu_sample.time_us;

            gps_sample[1].pos_horz = pr_meas[i].xy();
            gps_sample[1].hgt = -pr_meas[i](2);
            gps_sample[1].vel = vr_meas[i];
            gps_sample[1].fix_type = 5;
            gps_sample[1].time_us = imu_sample.time_us;

//            if (i == 1000) {
//                gps_sample[0].hgt += 10.f;
//            }

            eskf_runner.set_gps_data(gps_sample[0], 0);
            eskf_runner.set_gps_data(gps_sample[1], 1);

//            std::cout << "set gps data" << std::endl;
        }

        eskf_runner.update();

        nonCF.feed_imu(imu_sample.delta_ang, imu_sample.delta_vel, imu_sample.delta_ang_dt);
        Vector3f cf_vel(vn[i](0), vn[i](1), vn[i](2));
        nonCF.feed_vel(cf_vel);
        Vector3f diff_nav = pl_meas[i] - pr_meas[i];
        Vector3f diff_body = dl - dr;
        nonCF.feed_aux_meas(diff_nav, diff_body, 4.f);
        nonCF.fit();

        Quatd tmp(double(eskf_rtk.get_quaternion()(0)), double(eskf_rtk.get_quaternion()(1)), double(eskf_rtk.get_quaternion()(2)), double(eskf_rtk.get_quaternion()(3)));
        Vector3d tmp1 = generator::quat2euler(tmp);

        euler_hat[i] = {float(tmp1(0)), float(tmp1(1)), float(tmp1(2))};
        p_hat[i] = eskf_rtk.get_position();
        v_hat[i] = eskf_rtk.get_velocity();
        bg_hat[i] = eskf_rtk.get_gyro_bias();
        ba_hat[i] = eskf_rtk.get_acc_bias();
        g_hat[i] = eskf_rtk.get_gravity();

        tmp(0) = double(nonCF.q()(0));
        tmp(1) = double(nonCF.q()(1));
        tmp(2) = double(nonCF.q()(2));
        tmp(3) = double(nonCF.q()(3));
        tmp1 = generator::quat2euler(tmp);

        cout << "euler: " << euler[i](0) << ", " << euler[i](1) << ", " << euler[i](2) << ", ";
//        cout << euler_hat[i](0) << ", " << euler_hat[i](1) << ", " << euler_hat[i](2) << ", ";
        cout << tmp1(0) << ", " << tmp1(1) << ", " << tmp1(2) << endl;

//        cout << "pos: " << pn[i](0) << ", " << pn[i](1) << ", " << pn[i](2) << ", ";
//        cout << p_hat[i](0) << ", " << p_hat[i](1) << ", " << p_hat[i](2) << endl;

    }

    return 0;
}

