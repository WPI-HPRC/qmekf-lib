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
    
    for (int i = 0; i < 6; i++) {
        lastCalcTimes(i) = curr_time;
    }
    for (int i = 0; i < 2; i++) {
        lastUpdateTimes(i) = curr_time;
    }
    

    gyro_prev.Fill(0);
    accel_prev.Fill(0);
    vel_prev.Fill(0);
    mag_prev.Fill(0);
    baro_prev.Fill(0);

    computeInitialOrientation(accel, mag);
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
    // Lowkey could be more efficient, but shouldn't be called much
    return dcm2quat(get_dcmned2ecef() * quat2DCM(extractSub(att_x, SplitMEKFInds::quat)));
}

BLA::Matrix<3, 1> SplitStateEstimator::get_vel_ecef() {
    
    return extractSub(pv_x, SplitMEKFInds::vel);
}

BLA::Matrix<3, 1> SplitStateEstimator::get_vel_ned() {
    return get_dcmned2ecef() * get_vel_ecef();
}

BLA::Matrix<3, 1> SplitStateEstimator::get_pos_ecef() {
    return extractSub(pv_x, SplitMEKFInds::pos);
}

BLA::Matrix<3, 1> SplitStateEstimator::get_pos_ned() {
    return get_dcmned2ecef() * (get_pos_ecef() - launch_ecef);
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

void SplitStateEstimator::set_dcmned2ecef(BLA::Matrix<3, 3> new_dcm_ned2ecef) {
    dcmned2ecef = new_dcm_ned2ecef;
}

float SplitStateEstimator::getGs() {
    return BLA::Norm(get_accel_prev());
}

float SplitStateEstimator::getLastAttProp() {
    return lastCalcTimes(0);
}

float SplitStateEstimator::getLastAttUpdate() {
    return lastUpdateTimes(0);
}

float SplitStateEstimator::getLastPVProp() {
    return lastCalcTimes(1);
}

float SplitStateEstimator::getLastPVUpdate() {
    return lastUpdateTimes(1);
}

// TODO
void computeInitialOrientation() {
    // TRIAD_EB()
    // TODO triad and set to the att_x to that
}




BLA::Matrix<13, 1> SplitStateEstimator::fastGyroProp(BLA::Matrix<3,1> gyro, float curr_time) {
    gyro = vimu_const::asm_to_board * gyro;

    BLA::Matrix<4, 1> last_relevant_times = {0, 2, 3, 4}; // Gyro prop, accel+mag+gps update
    float dt = curr_time - vecMax(extractSub(lastCalcTimes, last_relevant_times));
    
    float dt2 = dt/2.0f;

    BLA::Matrix<4, 1> p_q = extractSub(att_x, SplitMEKFInds::quat);

    BLA::Matrix<3, 1> unbiased_gyro = gyro - extractSub(att_x, SplitMEKFInds::gyroBias);
    // OLD METHOD BROKEN BLA::Matrix<4, 1> new_q = quatMultiply(extractSub(x, SplitMEKFInds::quat), rotVec2dQuat(unbiased_gyro * dt));
    // TODO: Need to test if can use euler-rod formula or first order approx

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
    accel = vimu_const::asm_to_board * accel;
    BLA::Matrix<4, 1> last_relevant_times = {0, 4, 5}; // accel prop, accel+mag+gps update
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

// BLA::Matrix<19, 19> SplitStateEstimator::ekfPredict(float curr_time) {
//     // TODO finish this. too lazy
//     /*
//     BLA::Matrix<4, 1> att_last_relevant_times = {0, 1, 3, 4};
//     BLA::Matrix<4, 1> pv_last_relevant_times = {1, 4, 5};
//     float att_dt = curr_time - vecMax(extractSub(lastCalcTimes, att_last_relevant_times));
//     float pv_dt = curr_time - vecMax(extractSub(lastCalcTimes, pv_last_relevant_times));

//     DBG.print("att_dt: "); DBG.println(att_dt, 6);
//     DBG.print("pv_dt: "); DBG.println(pv_dt, 6);
//     */

//     float dt = curr_time - lastPredictTime;
    
//     lastPredictTime = curr_time;


//     BLA::Matrix<3,3> gyroSkew = skewSymmetric(gyro_prev);
//     BLA::Matrix<3,3> accelSkew = skewSymmetric(accel_prev);

//     BLA::Matrix<3, 3> q_conj_dcm = quat2DCM(quatConjugate(extractSub(x, SplitMEKFInds::quat)));

//     BLA::Matrix<19, 19> F;
//     F.Fill(0);

//     //Row 1 - 3
//     F.Submatrix<3, 3>(0, SplitMEKFInds::q_w) = -1.0f * gyroSkew; 
//     F.Submatrix<3, 3>(0, SplitMEKFInds::gb_x - 1) = -1.0f * I_3;

//     //Row 4 - 6
//     F.Submatrix<3, 3>(SplitMEKFInds::v_x - 1, SplitMEKFInds::q_w) = -1.0f * q_conj_dcm * accelSkew;
//     F.Submatrix<3, 3>(SplitMEKFInds::v_x - 1, SplitMEKFInds::ab_x - 1) = -1.0f * q_conj_dcm;

//     //Row 7 - 9
//     F.Submatrix<3, 3>(SplitMEKFInds::p_x - 1, SplitMEKFInds::v_x - 1) = I_3;
    
//     BLA::Matrix<19, 19> phi;
//     phi.Fill(0);

//     phi = I_19 + (F * dt) + (0.5f * F * F * float(pow(dt, 2))); // I was hoping to split up the dt. Not that deep tho

//     BLA::Matrix<19, 19> phi_t = ~phi;

//     // Process noise
//     BLA::Matrix<19, 19> Q_d;
//     Q_d.Fill(0);


//     //Sorry random numbers theyre in the output.txt file from the allan variance tests
//     BLA::Matrix<3, 3> gyro_var_diag;
//     gyro_var_diag.Fill(0);
//     gyro_var_diag(0, 0) = 0.00015761;
//     gyro_var_diag(1, 1) = 0.00012345;
//     gyro_var_diag(2, 2) = 0.00010394;

//     BLA::Matrix<3, 3> gyro_bias_var_diag;
//     gyro_bias_var_diag.Fill(0);
//     gyro_bias_var_diag(0, 0) = 0.0f; // 0.1152665 / 1000.0f * (PI / 180.0f);
//     gyro_bias_var_diag(1, 1) = 0.0f; // 0.1621635 / 1000.0f * (PI / 180.0f);
//     gyro_bias_var_diag(2, 2) = 0.0f; // 0.09894897 / 1000.0f * (PI / 180.0f);

//     BLA::Matrix<3, 3> accel_var_diag;
//     accel_var_diag.Fill(0);
//     accel_var_diag(0, 0) = 0.0013;
//     accel_var_diag(1, 1) = 0.0013;
//     accel_var_diag(2, 2) = 0.0026;

//     BLA::Matrix<3, 3> accel_bias_var_diag;
//     accel_bias_var_diag.Fill(0);
//     accel_bias_var_diag(0, 0) = 0.0f; //0.01474176 * 0.00980665f;
//     accel_bias_var_diag(1, 1) = 0.0f; // 0.03503381 * 0.00980665f;
//     accel_bias_var_diag(2, 2) = 0.0f; // 0.0043195624 * 0.00980665f;

//     BLA::Matrix<3, 3> mag_bias_var_diag;
//     mag_bias_var_diag.Fill(0);
//     mag_bias_var_diag(0, 0) = 0.008301463f;
//     mag_bias_var_diag(1, 1) = 0.003226824f;
//     mag_bias_var_diag(2, 2) = 0.01745155f;

//     BLA::Matrix<1, 1> baro_bias_var_diag;
//     baro_bias_var_diag.Fill(0);
//     baro_bias_var_diag(0, 0) = 0.01243823f;

//     Q_d.Submatrix<3, 3>(SplitMEKFInds::q_w, SplitMEKFInds::q_w) = (gyro_var_diag * dt) + (gyro_bias_var_diag * float((pow(dt, 3) / 3.0)));
//     Q_d.Submatrix<3, 3>(SplitMEKFInds::q_w, 9) = -1.0f * gyro_bias_var_diag * float((pow(dt, 2) / 2));

//     Q_d.Submatrix<3 ,3>(3, 3) = accel_var_diag * dt + accel_bias_var_diag * float((pow(dt, 3) / 3));
//     Q_d.Submatrix<3, 3>(3, 6) = accel_bias_var_diag * float((pow(dt ,4) / 8.0)) + accel_var_diag * float((pow(dt, 2) / 2.0));
//     Q_d.Submatrix<3, 3>(3, 10) = -1.0f * accel_bias_var_diag * float((pow(dt, 2) / 2.0));

//     Q_d.Submatrix<3, 3>(6, 3) = accel_var_diag * float((pow(dt, 2) / 2)) + accel_bias_var_diag * float((pow(dt, 4) / 8.0));
//     Q_d.Submatrix<3, 3>(6, 6) = accel_var_diag * float((pow(dt, 3) / 3.0)) + accel_bias_var_diag * float((pow(dt, 5) / 20.0));
//     Q_d.Submatrix<3, 3>(6, 10) = -1.0f * accel_bias_var_diag * float((pow(dt, 3) / 6.0));

//     Q_d.Submatrix<3, 3>(9, 0) = -1.0f * gyro_bias_var_diag * float((pow(dt, 2) / 2.0));
//     Q_d.Submatrix<3, 3>(9, 9) = gyro_bias_var_diag * float((pow(dt, 2) / 2.0));

//     Q_d.Submatrix<3, 3>(12, 3) = -1.0f * accel_bias_var_diag * float((pow(dt, 2) / 2.0));
//     Q_d.Submatrix<3, 3>(12, 6) = -1.0f * accel_bias_var_diag * float((pow(dt, 3) / 6.0));
//     Q_d.Submatrix<3, 3>(12, 12) = accel_bias_var_diag * dt;

//     Q_d.Submatrix<3, 3>(15, 15) = mag_bias_var_diag * dt;
//     Q_d.Submatrix<1, 1>(18, 18) = baro_bias_var_diag * dt;

//     P = phi * P * phi_t + Q_d;

//     DBG.print(P(0, 0), 7);
//     DBG.print(", ");
//     DBG.print(P(1, 1), 7);
//     DBG.print(", ");
//     DBG.println(P(2, 2), 7);

//     return P;
    
// }

// BLA::Matrix<20, 1> SplitStateEstimator::runAccelUpdate(BLA::Matrix<3, 1> a_b, float curr_time) {
//     a_b = vimu_const::asm_to_board * a_b;

//     BLA::Matrix<3, 1> unbiased_accel = a_b - extractSub(x, SplitMEKFInds::accelBias);
//     float u_a_n = BLA::Norm(unbiased_accel);
//     unbiased_accel = (unbiased_accel / u_a_n);
//     BLA::Matrix<4,1> q = extractSub(x, SplitMEKFInds::quat);

//     BLA::Matrix<3, 1> h_accel = quat2DCM(q) * normal_i_ecef(launch_dcmned2ecef);
//     float h_a_n = BLA::Norm(h_accel);
//     h_accel = h_accel / h_a_n;


//     BLA::Matrix<3, 19> H_accel;
//     H_accel.Fill(0);
//     H_accel.Submatrix<3, 3>(0, 0) = -1.0f * skewSymmetric(h_accel);
//     H_accel.Submatrix<3, 3>(0, SplitMEKFInds::ab_x - 1) = I_3;

//     BLA::Matrix<3, 3> R;
//     R.Fill(0);
//     //tune ts
//     float sigma_accel = 0.1f; 
//     float sigma_n = sigma_accel / u_a_n;
//     //why wont diag wrk ugh
//     R(0, 0) = sigma_n * sigma_n;
//     R(1, 1) = sigma_n * sigma_n;
//     R(2, 2) = sigma_n * sigma_n;

//     lastCalcTimes(2) = curr_time;
//     return ekfCalcErrorInject(unbiased_accel, H_accel, h_accel, R);
// }

// BLA::Matrix<20, 1> SplitStateEstimator::runMagUpdate(BLA::Matrix<3, 1> m_b, float curr_time) {
//     m_b = vimu_const::mag_to_board * m_b;

//     BLA::Matrix<3, 1> unbiased_mag = m_b - extractSub(x, SplitMEKFInds::magBias);
//     float u_m_n = BLA::Norm(unbiased_mag);
//     unbiased_mag = unbiased_mag / u_m_n;
//     BLA::Matrix<4,1> q = extractSub(x, SplitMEKFInds::quat);

//     BLA::Matrix<3, 1> h_mag = quat2DCM(q) * m_i_ecef(launch_dcmned2ecef);

//     BLA::Matrix<3, 19> H_mag;
//     H_mag.Fill(0);
//     H_mag.Submatrix<3, 3>(0, 0) = -1.0f * skewSymmetric(h_mag);
//     H_mag.Submatrix<3, 3>(0, SplitMEKFInds::mb_x - 1) = I_3;

//     float sigma_mag = 0.10f;
//     float sigma_dir = sigma_mag / u_m_n;
//     float var = sigma_dir * sigma_dir;

//     BLA::Matrix<3, 3> R;
//     R.Fill(0);
//     R(0, 0) = var;
//     R(1, 1) = var;
//     R(2, 2) = var;

//     lastCalcTimes(3) = curr_time;
//     return ekfCalcErrorInject(unbiased_mag, H_mag, h_mag, R);
// }

// BLA::Matrix<20, 1> SplitStateEstimator::runGPSUpdate(BLA::Matrix<3, 1> gpsPos, BLA::Matrix<3, 1> gpsVel, bool velOrientation, float curr_time) {
//     // TODO something is wrong with gps vel
//     lastCalcTimes(4) = curr_time;
//     if (velOrientation) {
//         BLA::Matrix<9, 1> combined_sens = vstack(vstack(gpsVel, gpsPos), gpsVel);
//         BLA::Matrix<3, 1> v_b = {BLA::Norm(gpsVel), 0, 0};
//         BLA::Matrix<4, 1> q = extractSub(x, SplitMEKFInds::quat);
//         BLA::Matrix<3, 1> h_vi = quat2DCM(quatConjugate(q)) * v_b;
//         BLA::Matrix<9, 1> h_gps = vstack(extractSub(x, vstack(SplitMEKFInds::vel, SplitMEKFInds::pos)), extractSub(x, SplitMEKFInds::vel));
        
//         BLA::Matrix<9, 19> H_gps;
//         H_gps.Fill(0);
//         H_gps.Submatrix<3, 3>(0, SplitMEKFInds::v_x - 1) = I_3;
//         H_gps.Submatrix<3, 3>(3, SplitMEKFInds::p_x - 1) = I_3;
//         H_gps.Submatrix<3, 3>(6, 0) = -1.0f * quat2DCM(quatConjugate(q)) * skewSymmetric(v_b);

//         BLA::Matrix<9, 9> R = toDiag(vstack(vstack(Max10S_const::gpsVel_var, Max10S_const::gpsPos_var), Max10S_const::gpsVel_var));

//         return ekfCalcErrorInject(combined_sens, H_gps, h_gps, R);
//     } else {
//         BLA::Matrix<6, 1> combined_sens = vstack(gpsVel, gpsPos);
//         BLA::Matrix<6, 19> H_gps;
//         H_gps.Fill(0);
//         H_gps.Submatrix<3, 3>(0, SplitMEKFInds::v_x - 1) = I_3;
//         H_gps.Submatrix<3, 3>(3, SplitMEKFInds::p_x - 1) = I_3;

//         BLA::Matrix<6, 1> h_gps = extractSub(x, vstack(SplitMEKFInds::vel, SplitMEKFInds::pos));

//         BLA::Matrix<6, 6> R = toDiag(vstack(Max10S_const::gpsVel_var, Max10S_const::gpsPos_var));
//         return ekfCalcErrorInject(combined_sens, H_gps, h_gps, R);
//     }
    
// }

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