#include "../include/split_mekf.h"

#include "BasicLinearAlgebra.h"

#include <Arduino.h>

#include "../include/QuaternionUtils.h"

#include "../include/kfConsts.h"

#include <array>

#if defined(USBCON)
#define DBG SerialUSB
#else
#define DBG Serial
#endif

using namespace QuaternionUtils;
using namespace std;
using namespace vimu_const;

void SplitStateEstimator::init(float curr_time, BLA::Matrix<3, 1> accel, BLA::Matrix<3, 1> mag) {
    
    // Prop
    for (int i = 0; i < 2; i++) {
        lastCalcTimes(i) = curr_time;
    }
    // Updates
    for (int i = 0; i < 6; i++) {
        lastUpdateTimes(i) = curr_time;
    }
    
    set_dcmned2ecef(launch_dcmned2ecef);

    pv_x.Submatrix<3, 1>(2, 5) = launch_ecef;


    gyro_prev.Fill(0);
    accel_prev.Fill(0);
    vel_prev.Fill(0);
    mag_prev.Fill(0);
    baro_prev.Fill(0);

    computeInitialOrientation(accel, mag, curr_time);
}

BLA::Matrix<13, 1> SplitStateEstimator::getAttState() {
    return att_x;
}

BLA::Matrix<12, 12> SplitStateEstimator::getAttP() {
    return att_P;
}

BLA::Matrix<12, 1> SplitStateEstimator::getAttPDiag() {
    return QuaternionUtils::extractDiag(getAttP());
}

BLA::Matrix<10, 1> SplitStateEstimator::getPVState() {
    return pv_x;
}

BLA::Matrix<10, 10> SplitStateEstimator::getPVP() {
    return pv_P;
}

BLA::Matrix<10, 1> SplitStateEstimator::getPVPDiag() {
    return QuaternionUtils::extractDiag(getPVP());
}

BLA::Matrix<4, 1> SplitStateEstimator::get_quat_ecef() {
    return extractSub(att_x, SplitMEKFInds::quat);
}

BLA::Matrix<4, 1> SplitStateEstimator::get_quat_ned() {
    // Lowkey could be more efficient, but shouldn't be called much. Actually it will but idc rn
    return dcm2quat(~get_dcmned2ecef() * quat2DCM(extractSub(att_x, SplitMEKFInds::quat)));
}

BLA::Matrix<3, 1> SplitStateEstimator::get_rpy_ned() {
    // Too lazy to make more efficient. Sue me. Prob should have kept functions in rotm
    return quat2RPY(dcm2quat(~get_dcmned2ecef() * quat2DCM(extractSub(att_x, SplitMEKFInds::quat))));
}

BLA::Matrix<3, 1> SplitStateEstimator::get_vel_ecef() {
    
    return extractSub(pv_x, SplitMEKFInds::vel);
}

BLA::Matrix<3, 1> SplitStateEstimator::get_vel_ned() {
    return ~get_dcmned2ecef() * get_vel_ecef();
}

BLA::Matrix<3, 1> SplitStateEstimator::get_vel_body() {
    return quat2DCMInv(extractSub(att_x, SplitMEKFInds::quat)) * get_vel_ecef();
}

BLA::Matrix<3, 1> SplitStateEstimator::get_pos_ecef() {
    return extractSub(pv_x, SplitMEKFInds::pos);
}

BLA::Matrix<3, 1> SplitStateEstimator::get_pos_ned() {
    return ~get_dcmned2ecef() * (get_pos_ecef() - launch_ecef);
}

BLA::Matrix<3, 1> SplitStateEstimator::get_gyro_bias() {
    return extractSub(att_x, SplitMEKFInds::gyroBias);
}

BLA::Matrix<3, 1> SplitStateEstimator::get_gyro_prev() {
    return gyro_prev;
}

BLA::Matrix<3, 1> SplitStateEstimator::get_accel_prev() {
    return accel_prev;
}

BLA::Matrix<3, 1> SplitStateEstimator::get_mag_prev() {
    return mag_prev;
}

BLA::Matrix<1, 1> SplitStateEstimator::get_baro_prev() {
    return baro_prev;
}

void SplitStateEstimator::set_curr_temp(float new_temp) {
    curr_temp = new_temp;
}

float SplitStateEstimator::get_curr_temp() {
    return curr_temp;
}

BLA::Matrix<3, 3> SplitStateEstimator::get_dcmned2ecef() {
    return dcmned2ecef;
}

void SplitStateEstimator::set_dcmned2ecef(BLA::Matrix<3, 3> new_dcmned2ecef) {
    dcmned2ecef = new_dcmned2ecef;
}

float SplitStateEstimator::getGs() {
    return BLA::Norm(get_accel_prev());
}

// TODO Fix this shit
float SplitStateEstimator::getLastAttProp() {
    return lastCalcTimes(0);
}

float SplitStateEstimator::getLastAttUpdate() {
    BLA::Matrix<3, 1> last_relevant_times = {2, 3};
    return vecMax(extractSub(lastCalcTimes, last_relevant_times));
}

float SplitStateEstimator::getLastPVProp() {
    return lastCalcTimes(1);
}

float SplitStateEstimator::getLastPVUpdate() {
    BLA::Matrix<2, 1> last_relevant_times = {4, 5};
    return vecMax(extractSub(lastCalcTimes, last_relevant_times));
}

BLA::Matrix<3, 1> SplitStateEstimator::reorient_asm(BLA::Matrix<3, 1> value) {
    // Note: For the ASM
    BLA::Matrix<3, 3> reorient_matrix = {-1.0f, 0, 0,
                                    0, -1.0f, 0,
                                    0, 0, 1};

    return  reorient_matrix * value;
}

BLA::Matrix<3, 1> SplitStateEstimator::reorient_lis(BLA::Matrix<3, 1> value) {
    // Note: For the LIS
    // y multiply by -1
    BLA::Matrix<3, 3> reorient_matrix = {0, 1, 0,
                                    1, 0, 0,
                                    0, 0, 1};

    return  reorient_matrix * value;
}


// TODO
void SplitStateEstimator::computeInitialOrientation(BLA::Matrix<3, 1> accel, BLA::Matrix<3, 1> mag, float curr_time) {
    BLA::Matrix<3, 3> orientation_mat = triad_EB(accel, mag, normal_i_ecef(get_dcmned2ecef()), m_i_ecef(get_dcmned2ecef()));


    setSub(att_x, SplitMEKFInds::quat, dcm2quat(orientation_mat));

    // SerialUSB.println(dcm2quat(orientation_mat));

    lastCalcTimes(0, 0) = curr_time;
    lastCalcTimes(2, 0) = curr_time;
    lastCalcTimes(3, 0) = curr_time;

}




BLA::Matrix<13, 1> SplitStateEstimator::fastGyroProp(BLA::Matrix<3,1> gyro, float curr_time) {
    BLA::Matrix<4, 1> last_relevant_times = {0, 2, 3, 4}; // Gyro prop, accel+mag+gps update
    float dt = curr_time - lastCalcTimes(0);
    
    float dt2 = dt/2.0f;

    BLA::Matrix<4, 1> p_q = extractSub(att_x, SplitMEKFInds::quat);

    BLA::Matrix<3, 1> unbiased_gyro = gyro - extractSub(att_x, SplitMEKFInds::gyroBias);

    BLA::Matrix<4, 1> new_q = {
        (p_q(0) - dt2*(unbiased_gyro(0))*p_q(1) - dt2*(unbiased_gyro(1))*p_q(2) - dt2*(unbiased_gyro(2))*p_q(3)),
        (p_q(1) + dt2*(unbiased_gyro(0))*p_q(0) - dt2*(unbiased_gyro(1))*p_q(3) + dt2*(unbiased_gyro(2))*p_q(2)),
        (p_q(2) + dt2*(unbiased_gyro(0))*p_q(3) + dt2*(unbiased_gyro(1))*p_q(0) - dt2*(unbiased_gyro(2))*p_q(1)),
        (p_q(3) - dt2*(unbiased_gyro(0))*p_q(2) + dt2*(unbiased_gyro(1))*p_q(1) + dt2*(unbiased_gyro(2))*p_q(0))
    };

    new_q = new_q / BLA::Norm(new_q);
    att_x = setSub(att_x, SplitMEKFInds::quat, new_q);
    gyro_prev = unbiased_gyro;
    lastCalcTimes(0) = curr_time;

    // DBG.println(unbiased_gyro);
    // DBG.print(new_q(0)); DBG.print(", "); DBG.print(new_q(1)); DBG.print(", "); DBG.print(new_q(2)); DBG.print(", "); DBG.println(new_q(3));
    
    return att_x;
}

BLA::Matrix<10, 1> SplitStateEstimator::fastAccelProp(BLA::Matrix<3, 1> accel, float curr_time) {
    BLA::Matrix<4, 1> last_relevant_times = {1, 4, 5}; // accel prop, accel+gps update
    float dt = curr_time - vecMax(extractSub(lastCalcTimes, last_relevant_times));

    BLA::Matrix<3, 1> f_b = accel - extractSub(pv_x, SplitMEKFInds::accelBias);
    
    //DBG.print("f_b: ");
    //DBG.print(f_b(0)); DBG.print(", "); DBG.print(f_b(1)); DBG.print(", "); DBG.println(f_b(2));
    
    BLA::Matrix<3, 1> g_i = normal_i_ecef(get_dcmned2ecef());
    
    //DBG.print("g_i: ");
    //DBG.print(g_i(0)); DBG.print(", "); DBG.print(g_i(1)); DBG.print(", "); DBG.println(g_i(2));

    BLA::Matrix<4,1> q = extractSub(att_x, SplitMEKFInds::quat);
    BLA::Matrix<3,3> C_bi = quat2DCM(q);
    BLA::Matrix<3,1> f_i  = C_bi * f_b;

    
    //DBG.print("f_i: ");
    //DBG.print(f_i(0)); DBG.print(", "); DBG.print(f_i(1)); DBG.print(", "); DBG.println(f_i(2));
    
    BLA::Matrix<3,1> a_i  = f_i - g_i;

    BLA::Matrix<3, 1> v = get_vel_ecef() + 0.5f * (a_i + get_accel_prev()) * dt;
    BLA::Matrix<3, 1> p = get_pos_ecef() + 0.5f * (v + get_vel_ecef()) * dt;


    accel_prev = f_b;
    lastCalcTimes(1) = curr_time;

    // DBG.print(v(0)); DBG.print(", "); DBG.print(v(1)); DBG.print(", "); DBG.print(v(2)); DBG.print(",");
    // DBG.print(p(0)); DBG.print(", "); DBG.print(p(1)); DBG.print(", "); DBG.println(p(2));

    pv_x = setSub(pv_x, SplitMEKFInds::vel, v);
    pv_x = setSub(pv_x, SplitMEKFInds::pos, p);
    return pv_x;
}

void SplitStateEstimator::falseUpdateVectors(float curr_time) {
    lastCalcTimes(2, 0) = curr_time;
    lastCalcTimes(3, 0) = curr_time;
}

// TODO test
BLA::Matrix<12, 12> SplitStateEstimator::AttekfPredict(float curr_time) {
    BLA::Matrix<2, 1> att_update_relevant_times = {2, 3};
    // accel, mag


    float att_dt = curr_time - vecMax(extractSub(lastCalcTimes, att_update_relevant_times));

    //DBG.print(att_dt, 7);

    BLA::Matrix<3,3> gyroSkew = skewSymmetric(gyro_prev);

    BLA::Matrix<3,3> q_conj_dcm = quat2DCMInv(extractSub(att_x, SplitMEKFInds::quat));

    BLA::Matrix<12, 12> F;
    F.Fill(0);

    // TODO: Fix indices
    //Row 1 - 3
    F.Submatrix<3, 3>(0, SplitMEKFInds::q_w) = -1.0f * gyroSkew;
    F.Submatrix<3, 3>(SplitMEKFInds::gb_x - 1, SplitMEKFInds::gb_x - 1) = -1.0f * I_3;

    // printMatHighDef(F);

    
    BLA::Matrix<12, 12> phi;
    phi.Fill(0);

    phi = I_12 + (F * att_dt) + (0.5f * F * F * float(pow(att_dt, 2)));

    BLA::Matrix<12, 12> phi_t = ~phi;

    // P = phi * P * phi_t + Q_d;
    att_P = phi * att_P * phi_t;

    BLA::Matrix<12, 12> Q_d;

    Q_d.Fill(0);

    BLA::Matrix<3, 1> gyro_var = {0.00015761, 0.00012345, 0.00010394};

    BLA::Matrix<3, 3> gyro_var_diag = toDiag(gyro_var);

    BLA::Matrix<3, 1> gyro_bias_var = {0.002, 0.002, 0.002};
    BLA::Matrix<3, 3> gyro_bias_var_diag = toDiag(gyro_bias_var);

    BLA::Matrix<3, 1> acc_bias_var = {0, 0, 0};
    BLA::Matrix<3, 3> acc_bias_var_diag = toDiag(acc_bias_var);

    BLA::Matrix<3, 1> mag_bias_var = {0, 0, 0};
    BLA::Matrix<3, 3> mag_bias_var_diag = toDiag(mag_bias_var);

    // BLA::Matrix<3, 1> gyro_bias_var = {1e-6f, 1e-6f, 1e-6f};
    // BLA::Matrix<3, 3> gyro_bias_var_diag = toDiag(gyro_bias_var);

    // BLA::Matrix<3, 1> acc_bias_var = {1e-5f, 1e-5f, 1e-5f};
    // BLA::Matrix<3, 3> acc_bias_var_diag = toDiag(acc_bias_var);

    // BLA::Matrix<3, 1> mag_bias_var = {1e-4f, 1e-4f, 1e-4f};
    // BLA::Matrix<3, 3> mag_bias_var_diag = toDiag(mag_bias_var);

    // SerialUSB.print("att_dt: ");
    //SerialUSB.println(att_dt);

    Q_d.Submatrix<3, 3>(SplitMEKFInds::q_w, SplitMEKFInds::q_w) = gyro_var_diag * att_dt + gyro_bias_var_diag * (float) (std::pow(att_dt, 3) / 3.0);
    Q_d.Submatrix<3, 3>(0, SplitMEKFInds::gb_x - 1) = -1.0f * gyro_bias_var_diag * (float) (std::pow(att_dt, 2) / 2.0);
    Q_d.Submatrix<3, 3>(SplitMEKFInds::gb_x - 1, 0) = -1.0f * gyro_bias_var_diag * (float) (std::pow(att_dt, 2) / 2.0);
    Q_d.Submatrix<3, 3>(SplitMEKFInds::gb_x - 1, SplitMEKFInds::gb_x - 1) = -1.0f * gyro_bias_var_diag * (float) (std::pow(att_dt, 2) / 2.0);
    Q_d.Submatrix<3, 3>(SplitMEKFInds::ab_x - 1, SplitMEKFInds::ab_x - 1) = acc_bias_var_diag * att_dt;
    Q_d.Submatrix<3, 3>(SplitMEKFInds::mb_x - 1, SplitMEKFInds::mb_x - 1) = mag_bias_var_diag * att_dt;
    // SerialUSB.println("att_P: ");
    //printMatHighDef(Q_d);

    att_P = att_P + Q_d;

    // printMatHighDef(att_P);

    return att_P;

    // lastCalcTimes(2) = curr_time;
    // lastCalcTimes(3) = curr_time;
/*
    // Process noise
    BLA::Matrix<19, 19> Q_d;
    Q_d.Fill(0);


    //Sorry random numbers theyre in the output.txt file from the allan variance tests
    BLA::Matrix<3, 3> gyro_var_diag;
    gyro_var_diag.Fill(0);
    gyro_var_diag(0, 0) = 0.00015761;
    gyro_var_diag(1, 1) = 0.00012345;
    gyro_var_diag(2, 2) = 0.00010394;

    BLA::Matrix<3, 3> gyro_bias_var_diag;
    gyro_bias_var_diag.Fill(0);
    gyro_bias_var_diag(0, 0) = 0.0f; // 0.1152665 / 1000.0f * (PI / 180.0f);
    gyro_bias_var_diag(1, 1) = 0.0f; // 0.1621635 / 1000.0f * (PI / 180.0f);
    gyro_bias_var_diag(2, 2) = 0.0f; // 0.09894897 / 1000.0f * (PI / 180.0f);

    BLA::Matrix<3, 3> accel_var_diag;
    accel_var_diag.Fill(0);
    accel_var_diag(0, 0) = 0.0013;
    accel_var_diag(1, 1) = 0.0013;
    accel_var_diag(2, 2) = 0.0026;

    BLA::Matrix<3, 3> accel_bias_var_diag;
    accel_bias_var_diag.Fill(0);
    accel_bias_var_diag(0, 0) = 0.0f; //0.01474176 * 0.00980665f;
    accel_bias_var_diag(1, 1) = 0.0f; // 0.03503381 * 0.00980665f;
    accel_bias_var_diag(2, 2) = 0.0f; // 0.0043195624 * 0.00980665f;

    BLA::Matrix<3, 3> mag_bias_var_diag;
    mag_bias_var_diag.Fill(0);
    mag_bias_var_diag(0, 0) = 0.008301463f;
    mag_bias_var_diag(1, 1) = 0.003226824f;
    mag_bias_var_diag(2, 2) = 0.01745155f;

    BLA::Matrix<1, 1> baro_bias_var_diag;
    baro_bias_var_diag.Fill(0);
    baro_bias_var_diag(0, 0) = 0.01243823f;

    Q_d.Submatrix<3, 3>(SplitMEKFInds::q_w, SplitMEKFInds::q_w) = (gyro_var_diag * dt) + (gyro_bias_var_diag * float((pow(dt, 3) / 3.0)));
    Q_d.Submatrix<3, 3>(SplitMEKFInds::q_w, 9) = -1.0f * gyro_bias_var_diag * float((pow(dt, 2) / 2));

    Q_d.Submatrix<3 ,3>(3, 3) = accel_var_diag * dt + accel_bias_var_diag * float((pow(dt, 3) / 3));
    Q_d.Submatrix<3, 3>(3, 6) = accel_bias_var_diag * float((pow(dt ,4) / 8.0)) + accel_var_diag * float((pow(dt, 2) / 2.0));
    Q_d.Submatrix<3, 3>(3, 10) = -1.0f * accel_bias_var_diag * float((pow(dt, 2) / 2.0));

    Q_d.Submatrix<3, 3>(6, 3) = accel_var_diag * float((pow(dt, 2) / 2)) + accel_bias_var_diag * float((pow(dt, 4) / 8.0));
    Q_d.Submatrix<3, 3>(6, 6) = accel_var_diag * float((pow(dt, 3) / 3.0)) + accel_bias_var_diag * float((pow(dt, 5) / 20.0));
    Q_d.Submatrix<3, 3>(6, 10) = -1.0f * accel_bias_var_diag * float((pow(dt, 3) / 6.0));

    Q_d.Submatrix<3, 3>(9, 0) = -1.0f * gyro_bias_var_diag * float((pow(dt, 2) / 2.0));
    Q_d.Submatrix<3, 3>(9, 9) = gyro_bias_var_diag * float((pow(dt, 2) / 2.0));

    Q_d.Submatrix<3, 3>(12, 3) = -1.0f * accel_bias_var_diag * float((pow(dt, 2) / 2.0));
    Q_d.Submatrix<3, 3>(12, 6) = -1.0f * accel_bias_var_diag * float((pow(dt, 3) / 6.0));
    Q_d.Submatrix<3, 3>(12, 12) = accel_bias_var_diag * dt;

    Q_d.Submatrix<3, 3>(15, 15) = mag_bias_var_diag * dt;
    Q_d.Submatrix<1, 1>(18, 18) = baro_bias_var_diag * dt;


    DBG.print(P(0, 0), 7);
    DBG.print(", ");
    DBG.print(P(1, 1), 7);
    DBG.print(", ");
    DBG.println(P(2, 2), 7);
    */

}

BLA::Matrix<10, 10> SplitStateEstimator::PVekfPredict(float curr_time) {
    BLA::Matrix<1, 1> pv_update_relevant_times = {4};
    // GPS and one day baro too (5)

    float pv_dt = curr_time - vecMax(extractSub(lastCalcTimes, pv_update_relevant_times));

    // DBG.print(pv_dt, 7);

    BLA::Matrix<10, 10> F;
    F.Fill(0);

    // TODO: Fix indices
    //Row 1 - 3
    F.Submatrix<3, 3>(SplitMEKFInds::p_x, 0) = I_3;

    // printMatHighDef(F);

    
    BLA::Matrix<10, 10> phi;
    phi.Fill(0);

    phi = I_10 + (F * pv_dt) + (0.5f * F * F * float(pow(pv_dt, 2)));

    BLA::Matrix<10, 10> phi_t = ~phi;

    pv_P = phi * pv_P * phi_t;

    BLA::Matrix<10, 10> Q_d;

    Q_d.Fill(0);

    BLA::Matrix<3, 1> acc_var = {0.01, 0.01, 0.01};
    BLA::Matrix<3, 3> acc_var_diag = toDiag(acc_var);

    BLA::Matrix<3, 1> acc_bias_var = {0, 0, 0};
    BLA::Matrix<3, 3> acc_bias_var_diag = toDiag(acc_bias_var);

    Q_d.Submatrix<3, 3>(SplitMEKFInds::v_x, SplitMEKFInds::v_x) = acc_var_diag * pv_dt + acc_bias_var_diag * (float) (std::pow(pv_dt, 3) / 3.0);
    Q_d.Submatrix<3, 3>(SplitMEKFInds::v_x, SplitMEKFInds::p_x) = acc_bias_var_diag * (float) (std::pow(pv_dt, 4) / 8.0) + acc_var_diag * (float) (std::pow(pv_dt, 2) / 2.0);
    Q_d.Submatrix<3, 3>(SplitMEKFInds::v_x, SplitMEKFInds::ab_x) = -1.0f * acc_bias_var_diag * (float) (std::pow(pv_dt, 2) / 2.0);

    Q_d.Submatrix<3, 3>(SplitMEKFInds::p_x, SplitMEKFInds::v_x) = acc_var_diag * (float) (std::pow(pv_dt, 2) / 2.0) + acc_bias_var_diag * (float) (std::pow(pv_dt, 4) / 8.0);
    Q_d.Submatrix<3, 3>(SplitMEKFInds::p_x, SplitMEKFInds::p_x) = acc_var_diag * (float) (std::pow(pv_dt, 3) / 3.0) + acc_bias_var_diag * (float) (std::pow(pv_dt, 5) / 20.0);
    Q_d.Submatrix<3, 3>(SplitMEKFInds::p_x, SplitMEKFInds::ab_x) = -1.0f * acc_bias_var_diag * (float) (std::pow(pv_dt, 3) / 6.0);

    Q_d.Submatrix<3, 3>(SplitMEKFInds::ab_x - 1, SplitMEKFInds::ab_x - 1) = acc_bias_var_diag * pv_dt;

    // Q_d.Submatrix<3, 3>(0, SplitMEKFInds::gb_x - 1) = -1.0f * gyro_bias_var_diag * (float) (std::pow(att_dt, 2) / 2.0);
    // Q_d.Submatrix<3, 3>(SplitMEKFInds::gb_x - 1, 0) = -1.0f * gyro_bias_var_diag * (float) (std::pow(att_dt, 2) / 2.0);
    // Q_d.Submatrix<3, 3>(SplitMEKFInds::gb_x - 1, SplitMEKFInds::gb_x - 1) = -1.0f * gyro_bias_var_diag * (float) (std::pow(att_dt, 2) / 2.0);
    // Q_d.Submatrix<3, 3>(SplitMEKFInds::ab_x - 1, SplitMEKFInds::ab_x - 1) = acc_bias_var_diag * att_dt;
    // Q_d.Submatrix<3, 3>(SplitMEKFInds::mb_x - 1, SplitMEKFInds::mb_x - 1) = mag_bias_var_diag * att_dt;
    // printMatHighDef(Q_d);

    pv_P = pv_P + Q_d;

    return pv_P;
}

template<int R, int C>
void printMatrix(const BLA::Matrix<R, C>& M, const char* name = "") {
    if (name && name[0] != '\0') {
        DBG.println(name);
    }

    for (int i = 0; i < R; i++) {
        DBG.print("[ ");
        for (int j = 0; j < C; j++) {
            DBG.print(M(i, j), 6);  // 6 decimal places
            if (j < C - 1) DBG.print(", ");
        }
        DBG.println(" ]");
    }
    DBG.println();
}

BLA::Matrix<13, 1> SplitStateEstimator::runAccelUpdate(BLA::Matrix<3, 1> a_b, float curr_time) {

    BLA::Matrix<3, 1> unbiased_accel = a_b - extractSub(att_x, SplitMEKFInds::accelBias);
    // float u_a_n = BLA::Norm(unbiased_accel);
    // unbiased_accel = (unbiased_accel / u_a_n);
    BLA::Matrix<4,1> q = extractSub(att_x, SplitMEKFInds::quat);

    BLA::Matrix<3, 1> h_accel = quat2DCMInv(q) * normal_i_ecef(launch_dcmned2ecef);
    // float h_a_n = BLA::Norm(h_accel);
    // h_accel = h_accel / h_a_n;


    BLA::Matrix<3, 12> H_accel;
    H_accel.Fill(0);
    H_accel.Submatrix<3, 3>(0, 0) = skewSymmetric(h_accel);
    H_accel.Submatrix<3, 3>(0, SplitMEKFInds::ab_x - 1) = I_3;

    BLA::Matrix<3, 3> R;
    R.Fill(0);
    //tune ts
    float sigma_accel = 0.01f;
    float sigma_n = sigma_accel;
    //why wont diag wrk ugh
    R(0, 0) = sigma_n * sigma_n;
    R(1, 1) = sigma_n * sigma_n;
    R(2, 2) = sigma_n * sigma_n;

    lastCalcTimes(2) = curr_time;
    // return ekfAttCalcErrorInject(unbiased_accel, H_accel, h_accel, R);
}

BLA::Matrix<13, 1> SplitStateEstimator::runGPSAttUpdate(BLA::Matrix<3, 1> gpsVel, float curr_time) {
    BLA::Matrix<3, 1> v_b = {BLA::Norm(gpsVel), 0, 0};

    BLA::Matrix<4,1> q = extractSub(att_x, SplitMEKFInds::quat);

    BLA::Matrix<3, 1> h_gps = quat2DCM(q) * v_b;


    BLA::Matrix<3, 12> H_gps;
    H_gps.Fill(0);
    H_gps.Submatrix<3, 3>(0, 0) = -1.0f * quat2DCM(q) * skewSymmetric(v_b); // -1.0 * quat2rotm(att_state(1:4)') * skewSymmetric(v_b)

    BLA::Matrix<3, 3> R;
    R.Fill(0);
    //tune ts
    float sigma_vel = 0.05f;
    //why wont diag wrk ugh
    R(0, 0) = sigma_vel * sigma_vel;
    R(1, 1) = sigma_vel * sigma_vel;
    R(2, 2) = sigma_vel * sigma_vel;

    lastCalcTimes(4) = curr_time;
    return ekfAttCalcErrorInject(gpsVel, H_gps, h_gps, R);
}

BLA::Matrix<13, 1> SplitStateEstimator::runGPSMagAttUpdate(BLA::Matrix<3, 1> gpsVel, BLA::Matrix<3, 1> m_b, float curr_time) {
    BLA::Matrix<3, 1> v_b = {BLA::Norm(gpsVel), 0, 0};
    BLA::Matrix<3, 1> unbiased_mag = m_b - extractSub(att_x, SplitMEKFInds::magBias);
    BLA::Matrix<6, 1> unbiased_sens = vstack(v_b, m_b);

    BLA::Matrix<4,1> q = extractSub(att_x, SplitMEKFInds::quat);

    BLA::Matrix<3, 1> h_gps = quat2DCM(q) * v_b;
    BLA::Matrix<3, 1> h_mag = quat2DCMInv(q) * m_i_ecef(launch_dcmned2ecef);
    BLA::Matrix<6, 1> h_gps_mag = vstack(h_gps, h_mag);


    BLA::Matrix<6, 12> H_gps_mag;
    H_gps_mag.Fill(0);
    H_gps_mag.Submatrix<3, 3>(0, 0) = -1.0f * quat2DCM(q) * skewSymmetric(v_b);
    H_gps_mag.Submatrix<3, 3>(3, 6) = skewSymmetric(h_mag);
    H_gps_mag.Submatrix<3, 3>(3, SplitMEKFInds::mb_x - 1) = I_3;

    BLA::Matrix<6, 6> R;
    R.Fill(0);
    //tune ts
    float sigma_mag = 0.01f;
    float sigma_gps = 0.05f;
    //why wont diag wrk ugh
    R(0, 0) = sigma_gps * sigma_gps;
    R(1, 1) = sigma_gps * sigma_gps;
    R(2, 2) = sigma_gps * sigma_gps;
    R(3, 3) = sigma_mag * sigma_mag;
    R(4, 4) = sigma_mag * sigma_mag;
    R(5, 5) = sigma_mag * sigma_mag;

    lastCalcTimes(3) = curr_time;
    lastCalcTimes(4) = curr_time;

    return ekfAttCalcErrorInject(unbiased_sens, H_gps_mag, h_gps_mag, R);

}

BLA::Matrix<13, 1> SplitStateEstimator::runMagUpdate(BLA::Matrix<3, 1> m_b, float curr_time) {

    BLA::Matrix<3, 1> unbiased_mag = m_b - extractSub(att_x, SplitMEKFInds::magBias);

    float u_m_n = BLA::Norm(m_b);
    m_b = (m_b / u_m_n);

    BLA::Matrix<4,1> q = extractSub(att_x, SplitMEKFInds::quat);

    float u_mag_n = BLA::Norm(m_i_ecef(launch_dcmned2ecef));
    BLA::Matrix<3, 1> m_i = (m_i_ecef(launch_dcmned2ecef) / u_mag_n);

    BLA::Matrix<3, 1> h_mag = quat2DCMInv(q) * m_i;
    h_mag   /= BLA::Norm(h_mag);

    BLA::Matrix<3, 12> H_mag;
    H_mag.Fill(0);
    H_mag.Submatrix<3, 3>(0, 0) = skewSymmetric(h_mag);
    //H_mag.Submatrix<3, 3>(0, SplitMEKFInds::mb_x - 1) = I_3;

    BLA::Matrix<3, 3> R;
    R.Fill(0);
    float sigma_mag = 1.0f;
    float sigma_n = sigma_mag;
    //why wont diag wrk ugh
    R(0, 0) = sigma_n * sigma_n;
    R(1, 1) = sigma_n * sigma_n;
    R(2, 2) = sigma_n * sigma_n;

    lastCalcTimes(2) = curr_time;
    lastCalcTimes(3) = curr_time;
    return ekfAttCalcErrorInject(unbiased_mag, H_mag, h_mag, R);
}

BLA::Matrix<13, 1> SplitStateEstimator::runAccelMagUpdate(BLA::Matrix<3, 1> a_b, BLA::Matrix<3, 1> m_b, float curr_time) {
    BLA::Matrix<3, 1> unbiased_accel = a_b - extractSub(att_x, SplitMEKFInds::accelBias);
    BLA::Matrix<3, 1> unbiased_mag = m_b - extractSub(att_x, SplitMEKFInds::magBias);
    float u_a_n = BLA::Norm(a_b);
    a_b = (a_b / u_a_n);
    float u_m_n = BLA::Norm(m_b);
    m_b = (m_b / u_m_n);
    BLA::Matrix<6, 1> unbiased_sens = vstack(a_b, m_b);
    
    BLA::Matrix<4,1> q = extractSub(att_x, SplitMEKFInds::quat);


    float u_n_n = BLA::Norm(normal_i_ecef(launch_dcmned2ecef));
    BLA::Matrix<3, 1> normal_i = (normal_i_ecef(launch_dcmned2ecef) / u_n_n);
    float u_mag_n = BLA::Norm(m_i_ecef(launch_dcmned2ecef));
    BLA::Matrix<3, 1> m_i = (m_i_ecef(launch_dcmned2ecef) / u_mag_n);

    BLA::Matrix<3, 1> h_accel = quat2DCMInv(q) * normal_i;
    BLA::Matrix<3, 1> h_mag = quat2DCMInv(q) * m_i;

    h_accel /= BLA::Norm(h_accel);
    h_mag   /= BLA::Norm(h_mag);

    BLA::Matrix<6, 1> h_accel_mag = vstack(h_accel, h_mag);
    // float h_a_n = BLA::Norm(h_accel);
    // h_accel = h_accel / h_a_n;
    //printMatrix(h_accel_mag, "h_accel_mag:");
    //printMatrix(unbiased_sens, "unbiased_sens:");

    BLA::Matrix<6, 12> H_accel_mag;
    H_accel_mag.Fill(0);
    H_accel_mag.Submatrix<3, 3>(0, 0) = skewSymmetric(h_accel);
    // H_accel_mag.Submatrix<3, 3>(0, SplitMEKFInds::ab_x - 1) = I_3;
    H_accel_mag.Submatrix<3, 3>(3, 0) = skewSymmetric(h_mag);
    // H_accel_mag.Submatrix<3, 3>(3, SplitMEKFInds::mb_x - 1) = I_3;

    BLA::Matrix<6, 6> R;
    R.Fill(0);
    //tune ts
    float sigma_mag = 0.08f;
    float sigma_accel = 0.05f;
    //why wont diag wrk ugh
    R(0, 0) = sigma_accel * sigma_accel;
    R(1, 1) = sigma_accel * sigma_accel;
    R(2, 2) = sigma_accel * sigma_accel;
    R(3, 3) = sigma_mag * sigma_mag;
    R(4, 4) = sigma_mag * sigma_mag;
    R(5, 5) = sigma_mag * sigma_mag;

    lastCalcTimes(2) = curr_time;
    lastCalcTimes(3) = curr_time;

    //printMatrix(H_accel_mag, "H_accel_mag:");

    return ekfAttCalcErrorInject(unbiased_sens, H_accel_mag, h_accel_mag, R);
}


BLA::Matrix<10, 1> SplitStateEstimator::runGPSPVUpdate(BLA::Matrix<3, 1> gpsVel, BLA::Matrix<3, 1> gpsPos, float curr_time) {
    BLA::Matrix<6, 1> combined_sens = vstack(gpsVel, gpsPos);
    BLA::Matrix<6, 10> H_gps;
    H_gps.Fill(0);
    H_gps.Submatrix<3, 3>(0, SplitMEKFInds::v_x) = I_3;
    H_gps.Submatrix<3, 3>(3, SplitMEKFInds::p_x) = I_3;

    BLA::Matrix<6, 1> h_gps = extractSub(pv_x, vstack(SplitMEKFInds::vel, SplitMEKFInds::pos));

    BLA::Matrix<6, 6> R = toDiag(vstack(Max10S_const::gpsVel_var, Max10S_const::gpsPos_var));

    lastCalcTimes(4) = curr_time;
    
    return ekfPVCalcErrorInject(combined_sens, H_gps, h_gps, R);
}

template<int rows>
BLA::Matrix<13, 1> SplitStateEstimator::ekfAttCalcErrorInject(BLA::Matrix<rows, 1> &sens_reading, BLA::Matrix<rows, 12> H, BLA::Matrix<rows, 1> h, BLA::Matrix<rows, rows> R) {
    BLA::Matrix<rows, 1> residual = sens_reading - h;
    
    BLA::Matrix<rows, rows> S;
    BLA::Matrix<12, rows> K;
    BLA::Matrix<12, rows> H_t = ~H;
    
    S = H * att_P * H_t + R;
    K = (att_P * H_t) * BLA::Inverse(S);
    //joseph stabalize on it
    BLA::Matrix<12, 12> IKH = I_12 - K * H;
    att_P = IKH * att_P * ~IKH + K * R * ~K;
    BLA::Matrix<12, 1> postErrorState = K * residual;
    
    // Inject error angles into quat
    BLA::Matrix<3, 1> alpha = extractSub(postErrorState, SplitMEKFInds::smallAngle);

    BLA::Matrix old_q = extractSub(att_x, SplitMEKFInds::quat);
    BLA::Matrix<4, 1> q = quatMultiply(old_q, smallAnglerotVec2dQuat(alpha));
    float q_norm = BLA::Norm(q);
    q = q / q_norm;
    /* q = q / q_norm;    trying left mult 
    BLA::Matrix old_q = extractSub(x, SplitMEKFInds::quat);
    BLA::Matrix<4, 1> dq = smallAnglerotVec2dQuat(alpha);
    BLA::Matrix<4, 1> q  = quatMultiply(dq, old_q);
    q = q / BLA::Norm(q);
    */
    att_x(0) = q(0);
    att_x(1) = q(1);
    att_x(2) = q(2);
    att_x(3) = q(3);
    att_x(4) += postErrorState(3);
    att_x(5) += postErrorState(4);
    att_x(6) += postErrorState(5);
    att_x(7) += postErrorState(6);
    att_x(8) += postErrorState(7);
    att_x(9) += postErrorState(8);
    att_x(10) += postErrorState(9);
    att_x(11) += postErrorState(10);
    att_x(12) += postErrorState(11);

    
    return att_x;
}

template<int rows>
BLA::Matrix<10, 1> SplitStateEstimator::ekfPVCalcErrorInject(BLA::Matrix<rows, 1> &sens_reading, BLA::Matrix<rows, 10> H, BLA::Matrix<rows, 1> h, BLA::Matrix<rows, rows> R) {
    BLA::Matrix<rows, 1> residual = sens_reading - h;
    
    BLA::Matrix<rows, rows> S;
    BLA::Matrix<10, rows> K;
    BLA::Matrix<10, rows> H_t = ~H;
    
    S = H * pv_P * H_t + R;
    K = (pv_P * H_t) * BLA::Inverse(S);
    //joseph stabalize on it
    BLA::Matrix<10, 10> IKH = I_10 - K * H;
    pv_P = IKH * pv_P * ~IKH + K * R * ~K;
    BLA::Matrix<10, 1> postErrorState = K * residual;

    pv_x(0) += postErrorState(0);
    pv_x(1) += postErrorState(1);
    pv_x(2) += postErrorState(2);
    pv_x(3) += postErrorState(3);
    pv_x(4) += postErrorState(4);
    pv_x(5) += postErrorState(5);
    pv_x(6) += postErrorState(6);
    pv_x(7) += postErrorState(7);
    pv_x(8) += postErrorState(8);
    pv_x(9) += postErrorState(9);

    
    return pv_x;
}


int SplitStateEstimator::shouldKill(BLA::Matrix<3, 1> angular_vels, float angle_from_vert) {
    // Two parts: 1. Angular vel too great 2. Deviance from vertical is too high

    BLA::Matrix<3, 1> gyro_avg_rates = computeAverageAngularRates();

    for (int i = 0; i < 3; i++) {
        if (std::abs(gyro_avg_rates(i, 0)) > angular_vels(i, 0)) {
            return i + 1;
        }
    }



    BLA::Matrix<4, 1> orientation = get_quat_ned();
    BLA::Matrix<3, 1> roll_axis = {1, 0, 0};
    BLA::Matrix<4, 1> nominal = nominal_rocket_ned_orientation;
    BLA::Matrix<3, 1> est_x = qRot(orientation, roll_axis);
    BLA::Matrix<3, 1> expected_x = qRot(nominal, roll_axis);
    // Angle between two vectors formula
    float theta = std::acos(vecDot(est_x, expected_x));

    // SerialUSB.print(gb(0, 0), 7); SerialUSB.print(", "); SerialUSB.print(gb(1, 0), 7); SerialUSB.print(", "); SerialUSB.print(gb(2, 0), 7);

    // The axis for which it is off. Good to have, but not used here
    BLA::Matrix<3, 1> axis = BLA::CrossProduct(roll_axis, est_x);
    axis = normalizeVector(axis);

    BLA::Matrix<12, 1> P = getAttPDiag();
    BLA::Matrix<3, 1> inds = {0, 1, 2};
    BLA::Matrix<3, 1> P_body = get_dcmned2ecef() * extractSub(P, inds); // For our MEKF, covariance defined in inertial frame, kinda L
    float P_added_theta = std::hypotf(P_body(1, 0), P_body(2, 0));
    // SerialUSB.print(". P_added_theta: "); SerialUSB.print(P_added_theta);
    // Linearized. Kinda works. Should do hypot or max, or other distance metric, idk
    float sigma = 1.0f; // idk, tbd
    float total_theta = theta + sigma * P_added_theta;

    // if (total_theta > angle_from_vert) {
    //     return 0;
    // }

    return -1;
}

float SplitStateEstimator::solveAltitude(float pressure) {
    // physical parameters for model
    const float pb = 101325;  // [Pa] pressure at sea level
    const float Tb = 288.15;  // [K] temperature at seal level
    const float Lb = -0.0065; // [K/m] standard temperature lapse rate
    const float hb = 0; // [m] height at bottom of atmospheric layer (sea level)
    const float R = 8.31432;   // [N*m/mol*K] universal gas constant
    const float g0 = 9.80665;  // [m/s^2] Earth standard gravity
    const float M = 0.0289644; // [kg/mol] molar mass of Earth's air

    float pressure_Pa = pressure * 100;

    return hb +
            (Tb / Lb) * (pow((pressure_Pa / pb), (-R * Lb / (g0 * M))) - 1);
}

BLA::Matrix<3, 1> SplitStateEstimator::computeAverageAngularRates() {
    BLA::Matrix<3, 1> angular_rates = {0.0f, 0.0f, 0.0f};
    if (r_buffer.isEmpty()) return angular_rates;

    float r_sum = 0.0f;
    for (int i = 0; i < r_buffer.size(); i++) {
        r_sum += r_buffer[i];
    }
    angular_rates(0, 0) = r_sum / r_buffer.size();

    float p_sum = 0.0f;
    for (int i = 0; i < p_buffer.size(); i++) {
        p_sum += p_buffer[i];
    }
    angular_rates(1, 0) = p_sum / p_buffer.size();

    float y_sum = 0.0f;
    for (int i = 0; i < y_buffer.size(); i++) {
        y_sum += y_buffer[i];
    }
    angular_rates(2, 0) = y_sum / y_buffer.size();

    return angular_rates;
}


// BLA::Matrix<20, 1> SplitStateEstimator::runBaroUpdate(BLA::Matrix<1, 1> baro, float curr_time) {

//     BLA::Matrix<3, 1> lla = ecef2lla(extractSub(x, SplitMEKFInds::pos));

//     BLA::Matrix<1, 1> h_baro = {std::pow(lps22_const::P0 * (1.0f + (lps22_const::L * lla(2))) / lps22_const::T0, -1.0f * lps22_const::g_e * lps22_const::M / (lps22_const::R * lps22_const::L))};

//     float dP_dh = (-1.0f * lps22_const::g_e * h_baro(0, 0) * lps22_const::M) / (lps22_const::R * (lps22_const::T0 + lps22_const::L * lla(2)));
//     BLA::Matrix<3, 1> dh_decef = {cosd(lla(0)) * cosd(lla(1)), cosd(lla(0)) * sind(lla(1)), sind(lla(0))};
//     BLA::Matrix<3, 1> dP_decef = dP_dh * dh_decef;

//     BLA::Matrix<1, 19> H_baro;
//         H_baro.Fill(0);
//         H_baro.Submatrix<1, 3>(0, SplitMEKFInds::bb_p - 1) = ~dP_decef;
//         H_baro(0, 18) = 1.0f;

//     BLA::Matrix<1, 1> R = lps22_const::baro_var;

//     lastCalcTimes(5) = curr_time;
//     return ekfCalcErrorInject(baro, H_baro, h_baro, R);
// }

// template<int rows>
// BLA::Matrix<20, 1> SplitStateEstimator::ekfCalcErrorInject(BLA::Matrix<rows, 1> &sens_reading, BLA::Matrix<rows, 19> H, BLA::Matrix<rows, 1> h, BLA::Matrix<rows, rows> R) {
//     BLA::Matrix<rows, 1> residual = sens_reading - h;
    
//     BLA::Matrix<rows, rows> S;
//     BLA::Matrix<19, rows> K;
//     BLA::Matrix<19, rows> H_t = ~H;
    
//     S = H * P * H_t + R;
//     K = (P * H_t) * BLA::Inverse(S);
//     //joseph stabalize on it
//     BLA::Matrix<19, 19> IKH = I_19 - K * H;
//     P = IKH * P * ~IKH + K * R * ~K;
//     BLA::Matrix<19, 1> postErrorState = K * residual;
    
//     // Inject error angles into quat
//     BLA::Matrix<3, 1> alpha = extractSub(postErrorState, SplitMEKFInds::smallAngle);

//     BLA::Matrix old_q = extractSub(x, SplitMEKFInds::quat);
//     BLA::Matrix<4, 1> q = quatMultiply(old_q, smallAnglerotVec2dQuat(alpha));
//     float q_norm = BLA::Norm(q);
//     q = q / q_norm;
//     /* q = q / q_norm;    trying left mult 
//     BLA::Matrix old_q = extractSub(x, SplitMEKFInds::quat);
//     BLA::Matrix<4, 1> dq = smallAnglerotVec2dQuat(alpha);
//     BLA::Matrix<4, 1> q  = quatMultiply(dq, old_q);
//     q = q / BLA::Norm(q);
//     */
//     x(0) = q(0);
//     x(1) = q(1);
//     x(2) = q(2);
//     x(3) = q(3);
//     x(4) += postErrorState(3);
//     x(5) += postErrorState(4);
//     x(6) += postErrorState(5);
//     x(7) += postErrorState(6);
//     x(8) += postErrorState(7);
//     x(9) += postErrorState(8);
//     x(10) += postErrorState(9);
//     x(11) += postErrorState(10);
//     x(12) += postErrorState(11);
//     x(13) += postErrorState(12);
//     x(14) += postErrorState(13);
//     x(15) += postErrorState(14);
//     x(16) += postErrorState(15);
//     x(17) += postErrorState(16);
//     x(18) += postErrorState(17);
//     x(19) += postErrorState(18);

    
//     return x;
// }

// BLA::Matrix<4, 1> SplitStateEstimator::getNEDOrientation(BLA::Matrix<3, 3> &dcm_ned2ecef) {
//     // TODO inefficient, use quat mult
//     return dcm2quat(~dcm_ned2ecef * quat2DCM(extractSub(x, SplitMEKFInds::quat)));
// }

// BLA::Matrix<3, 1> SplitStateEstimator::getNEDPositionBody(BLA::Matrix<3, 3> &dcm_ned2ecef, BLA::Matrix<3, 1> launch_ecef) {
//     // Returns the distance from 
//     return ecef2nedVec(extractSub(x, SplitMEKFInds::pos) - qRot(extractSub(x, SplitMEKFInds::quat), mars_const::bodyToVIMUDis), launch_ecef, dcm_ned2ecef);
// }

// BLA::Matrix<3, 1> SplitStateEstimator::getBodyAngularVel() {
//     return gyro_prev;
// }

// BLA::Matrix<3, 1> SplitStateEstimator::getVIMUAccel() {
//     return accel_prev;
// }

// float SplitStateEstimator::getGs() {
//     return BLA::Norm(getVIMUAccel());
// }

// // TODO for all of these
// float getLastAttProp() {
//     // TODO
//     return 0.0f;
// }

// float getLastAttUpdate() {
//     // TODO
//     return 0.0f;
// }

// float getLastPVProp() {
//     // TODO
//     return 0.0f;
// }

// float getLastPVUpdate() {
//     // TODO
//     return 0.0f;
// }

// // GPS is less accurate in D direction, so account for that here. Vel then pos
// BLA::Matrix<6, 1> getGPSVar(BLA::Matrix<3, 3> dcm_ned2ecef) {
//     BLA::Matrix<6, 1> tmp = {0, 1, 2};
//     return tmp;
//     // return dcm_ned2ecef * gps_var;

// }
