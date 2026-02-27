#pragma once

#include "BasicLinearAlgebra.h"

#include "kfConsts.h"
#include <cstdint>
#include "QuaternionUtils.h"

/**
 * @name QMEKfInds
 * @brief Struct holding the indices of the total QMEKF
 */
namespace QMEKFInds {
constexpr int q_w = 0;
constexpr int q_x = 1;
constexpr int q_y = 2;
constexpr int q_z = 3;
inline BLA::Matrix<4, 1> quat = {q_w, q_x, q_y, q_z};
inline BLA::Matrix<3, 1> smallAngle = {q_w, q_x, q_y};

constexpr int v_x = 4;
constexpr int v_y = 5;
constexpr int v_z = 6;
inline BLA::Matrix<3, 1> vel = {v_x, v_y, v_z};

constexpr int p_x = 7;
constexpr int p_y = 8;
constexpr int p_z = 9;
inline BLA::Matrix<3, 1> pos = {p_x, p_y, p_z};

constexpr int gb_x = 10;
constexpr int gb_y = 11;
constexpr int gb_z = 12;
inline BLA::Matrix<3, 1> gyroBias = {gb_x, gb_y, gb_z};

constexpr int ab_x = 13;
constexpr int ab_y = 14;
constexpr int ab_z = 15;
inline BLA::Matrix<3, 1> accelBias = {ab_x, ab_y, ab_z};

constexpr int mb_x = 16;
constexpr int mb_y = 17;
constexpr int mb_z = 18;
inline BLA::Matrix<3, 1> magBias = {mb_x, mb_y, mb_z};

constexpr int bb_p = 19;
inline BLA::Matrix<1, 1> baroBias = {bb_p};



}; // namespace QMEKFInds


/**
 * @name QMEKFStateEstimator
 * @author QMEKF team
 * @brief Integrated Attitude and Position/Velocity estimation. See matlab simulation for details
 */
class StateEstimator {

  public:
    // Quat, gyroBias, accelBias, magBias
    BLA::Matrix<13, 1> att_x;
    BLA::Matrix<10, 1> pv_x;

    // Vel, pos, accelBias, baroBias
    BLA::Matrix<12, 12> att_P;
    BLA::Matrix<10, 10> pv_P;


    BLA::Matrix<13, 1> getAttState();
    BLA::Matrix<12, 12> getAttP();
    BLA::Matrix<12, 1> getAttPDiag();

    BLA::Matrix<10, 1> getPVState();
    BLA::Matrix<10, 10> getPVP();
    BLA::Matrix<10, 1> getPVPDiag();

    BLA::Matrix<3, 1> get_gyro_prev();
    BLA::Matrix<3, 1> get_accel_prev();
    BLA::Matrix<3,1> get_vel_prev();
    BLA::Matrix<3,1> get_vel_prev_ned();
    BLA::Matrix<3,1> get_pos_prev();
    BLA::Matrix<3,1> get_mag_prev();
    BLA::Matrix<1,1> get_baro_prev();
    BLA::Matrix<1,1> get_curr_temp();
    void set_curr_temp(float curr_temp);
    float getGs();



    void init(BLA::Matrix<3, 1> ECEF, float curr_time);
    void computeInitialOrientation();

    // Priori Functions
    BLA::Matrix<13, 1> fastGyroProp(BLA::Matrix<3,1> gyro, float curr_time);
    BLA::Matrix<10, 1> fastAccelProp(BLA::Matrix<3,1> accel, float curr_time);

    // EKF update functions
    BLA::Matrix<12, 12> AttekfPredict(float curr_time);
    BLA::Matrix<10, 10> PVekfPredict(float curr_time);

    // Att Update Functions
    BLA::Matrix<13, 1> runAccelUpdate(BLA::Matrix<3, 1> a_b, float curr_time);
    BLA::Matrix<13, 1> runMagUpdate(BLA::Matrix<3, 1> m_b, float curr_time);
    BLA::Matrix<13, 1> runAccelMagUpdate(BLA::Matrix<3, 1> a_b, BLA::Matrix<3, 1> m_b, float curr_time);
    BLA::Matrix<13, 1> runGPSAttUpdate(BLA::Matrix<3, 1> gpsVel, float curr_time);
    BLA::Matrix<13, 1> runGPSMagAttUpdate(BLA::Matrix<3, 1> gpsVel, BLA::Matrix<3, 1> m_b, float curr_time);

    // PV Update Functions
    BLA::Matrix<10, 1> runGPSPVUpdate(BLA::Matrix<3, 1> gpsVel, BLA::Matrix<3, 1> gpsPos, float curr_time);
    BLA::Matrix<10, 1> runBaroUpdate(BLA::Matrix<1, 1> baro, float curr_time);


    // Error Inject functions
    template<int rows>
    BLA::Matrix<13, 1> ekfAttCalcErrorInject(BLA::Matrix<rows, 1> &sens_reading, BLA::Matrix<rows, 13> H, BLA::Matrix<rows, 1> h, BLA::Matrix<rows, rows> R);
    
    template<int rows>
    BLA::Matrix<10, 1> ekfPVCalcErrorInject(BLA::Matrix<rows, 1> &sens_reading, BLA::Matrix<rows, 13> H, BLA::Matrix<rows, 1> h, BLA::Matrix<rows, rows> R);
    
    
    
    BLA::Matrix<4, 1> getNEDOrientation(BLA::Matrix<3, 3> &dcm_ned2ecef);
    
    // Return the position in the body frame
    BLA::Matrix<3, 1> getNEDPositionBody(BLA::Matrix<3, 3> &dcm_ned2ecef, BLA::Matrix<3, 1> launch_ecef);


    float getGs();

    BLA::Matrix<3, 3> launch_dcmned2ecef;
    BLA::Matrix<3, 1> launch_ecef = {1475354.0f, -4490428.0f, 4268181.0f};
    
  private:
    // Identity Matrices
    BLA::Matrix<20, 20> I_20 = BLA::Eye<20, 20>();
	  BLA::Matrix<19, 19> I_19 = BLA::Eye<19, 19>();
    BLA::Matrix<3, 3> I_3 = BLA::Eye<3, 3>();

    BLA::Matrix<6, 1> lastTimes = {0, 0, 0, 0, 0, 0};
    // Gyro prop, accel prop, accel update, mag update, gps update, baro update
    BLA::Matrix<3,1> gyro_prev = {0, 0, 0};
    BLA::Matrix<3,1> accel_prev = {0, 0, 0}; // TODO wouldn't it be normal force
    BLA::Matrix<3,1> vel_prev = {0, 0, 0};
    BLA::Matrix<3,1> pos_prev = launch_ecef;
    BLA::Matrix<3,1> mag_prev = {0, 0, 0};
    BLA::Matrix<1,1> baro_prev = {0};


    float curr_temp;


    // padLoop vars:
    float numLoop;
    BLA::Matrix<3, 1> sumAccel;
    BLA::Matrix<3, 1> sumMag;


    float getLastAttProp();
    float getLastAttUpdate();
    float getLastPVProp();
    float getLastPVUpdate();

    BLA::Matrix<6, 1> getGPSVar(BLA::Matrix<3, 3> dcm_ned2ecef);


    // //R matricies
    // BLA::Matrix<3, 3> R_accel = {accel_var, 0, 0,
    //                             0, accel_var, 0,
    //                             0, 0, accel_var};

    // BLA::Matrix<3, 3> R_mag = {mag_var, 0, 0,
    //                           0, mag_var, 0,
    //                           0, 0, mag_var};

    // BLA::Matrix<3, 3> R_gps= {gps_var, 0, 0,
    //                          0, gps_var, 0,
    //                          0, 0, gps_var};

    // BLA::Matrix<10, 1> R_all = {
    //   pow(sqrt(asm330_const::accelXY_var) * 9.8, 2),
    //   pow(sqrt(asm330_const::accelXY_var) * 9.8, 2),
    //   pow(sqrt(asm330_const::accelZ_var) * 9.8, 2),
    //   icm20948_const::magXYZ_var,
    //   icm20948_const::magXYZ_var,
    //   icm20948_const::magXYZ_var,
    //   Max10S_const::gpsXYZ_var,
    //   Max10S_const::gpsXYZ_var,
    //   Max10S_const::gpsXYZ_var,
    //   Max10S_const::baro_var
    // };



};
