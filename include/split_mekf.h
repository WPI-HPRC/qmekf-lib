#pragma once

#include "BasicLinearAlgebra.h"

#include "kfConsts.h"
#include <cstdint>
#include "QuaternionUtils.h"

/**
 * @name SplitMEKfInds
 * @brief Struct holding the indices of the total QMEKF
 */
namespace SplitMEKFInds {
constexpr int q_w = 0;
constexpr int q_x = 1;
constexpr int q_y = 2;
constexpr int q_z = 3;
inline BLA::Matrix<4, 1> quat = {q_w, q_x, q_y, q_z};
inline BLA::Matrix<3, 1> smallAngle = {q_w, q_x, q_y};

constexpr int v_x = 0;
constexpr int v_y = 1;
constexpr int v_z = 2;
inline BLA::Matrix<3, 1> vel = {v_x, v_y, v_z};

constexpr int p_x = 3;
constexpr int p_y = 4;
constexpr int p_z = 5;
inline BLA::Matrix<3, 1> pos = {p_x, p_y, p_z};

constexpr int gb_x = 4;
constexpr int gb_y = 5;
constexpr int gb_z = 6;
inline BLA::Matrix<3, 1> gyroBias = {gb_x, gb_y, gb_z};

constexpr int ab_x = 7;
constexpr int ab_y = 8;
constexpr int ab_z = 9;
inline BLA::Matrix<3, 1> accelBias = {ab_x, ab_y, ab_z};

constexpr int mb_x = 10;
constexpr int mb_y = 11;
constexpr int mb_z = 12;
inline BLA::Matrix<3, 1> magBias = {mb_x, mb_y, mb_z};

constexpr int bb_p = 9;
inline BLA::Matrix<1, 1> baroBias = {bb_p};



}; // namespace SplitMEKFInds


/**
 * @name SplitStateEstimator
 * @author QMEKF team
 * @brief Integrated Attitude and Position/Velocity estimation. See matlab simulation for details
 */
class SplitStateEstimator {

  public:
    void init(float curr_time, BLA::Matrix<3, 1> accel, BLA::Matrix<3, 1> mag);

    BLA::Matrix<13, 1> getAttState();
    BLA::Matrix<12, 12> getAttP();
    BLA::Matrix<12, 1> getAttPDiag();

    BLA::Matrix<10, 1> getPVState();
    BLA::Matrix<10, 10> getPVP();
    BLA::Matrix<10, 1> getPVPDiag();

    BLA::Matrix<4, 1> get_quat_ecef();
    BLA::Matrix<4, 1> get_quat_ned();
    BLA::Matrix<3,1> get_vel_ecef();
    BLA::Matrix<3,1> get_vel_ned();
    BLA::Matrix<3,1> get_pos_ecef();
    BLA::Matrix<3,1> get_pos_ned();

    BLA::Matrix<3, 1> get_gyro_prev();
    BLA::Matrix<3, 1> get_accel_prev();
    BLA::Matrix<3,1> get_mag_prev();
    BLA::Matrix<1,1> get_baro_prev();
    float get_curr_temp();
    void set_curr_temp(float curr_temp);
    BLA::Matrix<3, 3> get_dcmned2ecef();
    void set_dcmned2ecef(BLA::Matrix<3, 3> new_dcm_ned2ecef);
    float getGs();

    float getLastAttProp();
    float getLastAttUpdate();
    float getLastPVProp();
    float getLastPVUpdate();

    void computeInitialOrientation(BLA::Matrix<3, 1> accel, BLA::Matrix<3, 1> mag);

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
    
    


    
    
  private:
    // School stats
    BLA::Matrix<3, 3> launch_dcmned2ecef = {-0.2100, 0.9500, -0.2310, 0.6391, 0.3121, 0.7030, 0.7399, 0, -0.6727};
    BLA::Matrix<3, 3> dcmned2ecef = launch_dcmned2ecef;
    BLA::Matrix<3, 1> launch_ecef = {1475354.0f, -4490428.0f, 4268181.0f};
    BLA::Matrix<3, 1> launch_lla = {42.27405, -71.81174, 10};


    // TODO replace these values with actual values someday
    // Quat, gyroBias, accelBias, magBias
    BLA::Matrix<13, 1> att_x = {1, 0, 0, 0,
                                  0, 0, 0,
                                  0, 0, 0,
                                  0, 0, 0};

    // Vel, pos, accelBias, baroBias                              
    BLA::Matrix<10, 1> pv_x = {0, 0, 0,
                                1475354.0f, -4490428.0f, 4268181.0f,
                                0, 0, 0,
                                0};

    
    BLA::Matrix<12, 1> init_att_P_diag = {0.1, 0.1, 0.1, 0.1745, 0.1745, 0.1745, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
    BLA::Matrix<12, 12> att_P = toDiag(init_att_P_diag);
    BLA::Matrix<10, 1> init_pv_P_diag = {0.1, 0.1, 0.1, 5, 5, 5, 1, 1, 1, 50};
    BLA::Matrix<10, 10> pv_P = toDiag(init_pv_P_diag);



    // Identity Matrices
    BLA::Matrix<20, 20> I_20 = BLA::Eye<20, 20>();
	  BLA::Matrix<19, 19> I_19 = BLA::Eye<19, 19>();
    BLA::Matrix<10, 10> I_10 = BLA::Eye<10, 10>();
	  BLA::Matrix<13, 13> I_13 = BLA::Eye<13, 13>();
    BLA::Matrix<3, 3> I_3 = BLA::Eye<3, 3>();
    

    BLA::Matrix<6, 1> lastCalcTimes = {0, 0, 0, 0, 0, 0};
    // Gyro prop, accel prop, accel update, mag update, gps update, baro update
    BLA::Matrix<2, 1> lastUpdateTimes = {0, 0};
    // Att, PV

    BLA::Matrix<3,1> gyro_prev = {0, 0, 0};
    BLA::Matrix<3,1> accel_prev = {0, 0, 0};
    BLA::Matrix<3,1> vel_prev = {0, 0, 0};
    BLA::Matrix<3,1> pos_prev = launch_ecef;
    BLA::Matrix<3,1> mag_prev = {0, 0, 0};
    BLA::Matrix<1,1> baro_prev = {0};


    float curr_temp;


    //R matricies
    BLA::Matrix<3, 1> R_accel_diag = {0.00013924, 0.00013924, 0.00013924};

    BLA::Matrix<3, 1> R_mag_diag = {0.09, 0.09, 0.09};

    BLA::Matrix<3, 1> R_gps_vel_diag_ned = {0.0025, 0.0025, 0.0225};
    BLA::Matrix<3, 1> R_gps_pos_diag_ned = {0.5625, 0.5625, 1.7161};


};
