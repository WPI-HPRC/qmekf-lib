#include "../include/QuaternionUtils.h"
#include "BasicLinearAlgebra.h"
#include <cmath>
#include <iostream>


BLA::Matrix<3, 3> QuaternionUtils::quat2DCM(const BLA::Matrix<4, 1> &quat) {
    float w = quat(0);
    float x = quat(1);
    float y = quat(2);
    float z = quat(3);

    BLA::Matrix<3, 3> rot;

    // Row1
    rot(0, 0) = w * w + x * x - y * y - z * z;
    rot(0, 1) = 2 * x * y + 2 * w * z;
    rot(0, 2) = 2 * x * z - 2 * w * y;

    // Row2
    rot(1, 0) = 2 * x * y - 2 * w * z;
    rot(1, 1) = w * w - x * x + y * y - z * z;
    rot(1, 2) = 2 * y * z + 2 * w * x;

    // Row3
    rot(2, 0) = 2 * x * z + 2 * w * y;
    rot(2, 1) = 2 * y * z - 2 * w * x;
    rot(2, 2) = w * w - x * x - y * y + z * z;

    return ~rot;
}


BLA::Matrix<3, 3> QuaternionUtils::quat2DCMInv(const BLA::Matrix<4, 1> &quat) {
    // TODO one day rewrite it to make it more efficient
    return ~quat2DCM(quat);
}


BLA::Matrix<3, 3> QuaternionUtils::skewSymmetric(const BLA::Matrix<3, 1> &vec) {
    // Can rewrite as hstack
    BLA::Matrix<3, 3> mat;

    mat(0, 0) = 0;
    mat(0, 1) = -1 * vec(2, 0);
    mat(0, 2) = vec(1, 0);

    mat(1, 0) = vec(2, 0);
    mat(1, 1) = 0;
    mat(1, 2) = -1 * vec(0, 0);

    mat(2, 0) = -1 * vec(1, 0);
    mat(2, 1) = vec(0, 0);
    mat(2, 2) = 0;

    return mat;

}


BLA::Matrix<4, 1> QuaternionUtils::rotVec2dQuat(const BLA::Matrix<3, 1> &vec) {
    BLA::Matrix<4, 1> quat;

    float norm = BLA::Norm(vec);
    BLA::Matrix<3, 1> vec_rot_normed = vec / norm;
    float norm_over_two = norm / 2.0f;

    quat(0, 0) = cos(norm_over_two);
    quat(1, 0) = vec_rot_normed(0, 0) * sin(norm_over_two);
    quat(2, 0) = vec_rot_normed(1, 0) * sin(norm_over_two);
    quat(3, 0) = vec_rot_normed(2, 0) * sin(norm_over_two);

    return quat;
}


BLA::Matrix<4, 1> QuaternionUtils::smallAnglerotVec2dQuat(const BLA::Matrix<3, 1> &vec) {
    BLA::Matrix<4, 1> quat;
    quat(0, 0) = 1.0;
    quat(1, 0) = 0.5 * vec(0, 0);
    quat(2, 0) = 0.5 * vec(1, 0);
    quat(3, 0) = 0.5 * vec(2, 0);

    return quat;
}


BLA::Matrix<4, 1> QuaternionUtils::dcm2quat(const BLA::Matrix<3, 3> &dcm) {
    BLA::Matrix<3, 3> dcm_t = ~dcm;
    BLA::Matrix<4, 1> q_sq;
    q_sq(0, 0) = 0.25f * (1.0f + dcm_t(0, 0) - dcm_t(1, 1) - dcm_t(2, 2));
    q_sq(1, 0) = 0.25f * (1.0f - dcm_t(0, 0) + dcm_t(1, 1) - dcm_t(2, 2));
    q_sq(2, 0) = 0.25f * (1.0f - dcm_t(0, 0) - dcm_t(1, 1) + dcm_t(2, 2));
    q_sq(3, 0) = 0.25f * (1.0f + dcm_t(0, 0) + dcm_t(1, 1) + dcm_t(2, 2));

    int max_ind = vecMaxInd(q_sq);
    BLA::Matrix<4, 1> q_ret;

    switch (max_ind) {
        case 0:
            q_ret(1, 0) = 4.0 * q_sq(0, 0);
            q_ret(2, 0) = dcm_t(0, 1) + dcm_t(1, 0);
            q_ret(3, 0) = dcm_t(2, 0) + dcm_t(0, 2);
            q_ret(0, 0) = dcm_t(1, 2) - dcm_t(2, 1);
            q_ret = q_ret / (4.0f * sqrt(q_sq(0, 0)));
            break;
        case 1:
            q_ret(1, 0) = dcm_t(0, 1) + dcm_t(1, 0);
            q_ret(2, 0) = 4.0 * q_sq(1, 0);
            q_ret(3, 0) = dcm_t(1, 2) + dcm_t(2, 1);
            q_ret(0, 0) = dcm_t(2, 0) - dcm_t(0, 2);
            q_ret = q_ret / (4.0f * sqrt(q_sq(1, 0)));
            break;
        case 2:
            q_ret(1, 0) = dcm_t(2, 0) + dcm_t(0, 2);
            q_ret(2, 0) = dcm_t(1, 2) + dcm_t(2, 1);
            q_ret(3, 0) = 4.0 * q_sq(2, 0);
            q_ret(0, 0) = dcm_t(0, 1) - dcm_t(1, 0);
            q_ret = q_ret / (4.0f * sqrt(q_sq(2, 0)));
            break;
        case 3:
            q_ret(1, 0) = dcm_t(1, 2) - dcm_t(2, 1);
            q_ret(2, 0) = dcm_t(2, 0) - dcm_t(0, 2);
            q_ret(3, 0) = dcm_t(0, 1) - dcm_t(1, 0);
            q_ret(0, 0) = 4.0 * q_sq(3, 0);
            q_ret = q_ret / (4.0f * sqrt(q_sq(3, 0)));
            break;
    }

    return q_ret;
}


BLA::Matrix<4, 1> QuaternionUtils::quatMultiply(const BLA::Matrix<4, 1> &p, const BLA::Matrix<4, 1> &q) {
    BLA::Matrix<4, 1> quat;

    quat(0, 0) = p(0, 0) * q(0, 0) - p(1, 0) * q(1, 0) - p(2, 0) * q(2, 0) - p(3, 0) * q(3, 0);
    quat(1, 0) = p(0, 0) * q(1, 0) + p(1, 0) * q(0, 0) + p(2, 0) * q(3, 0) - p(3, 0) * q(2, 0);
    quat(2, 0) = p(0, 0) * q(2, 0)  - p(1, 0) * q(3, 0) + p(2, 0) * q(1, 0) + p(3, 0) * q(1, 0);
    quat(3, 0) = p(0, 0) * q(3, 0) + p(1, 0) * q(2, 0) - p(2, 0) * q(1, 0) + p(3, 0) * q(3, 0);

    // quat(0, 0) = p(0, 0) * q(0, 0) - p(1, 0) * q(1, 0) - p(2, 0) * q(2, 0) - p(3, 0) * q(3, 0);
    // quat(1, 0) = p(0, 0) * q(1, 0) + p(1, 0) * q(0, 0) - p(2, 0) * q(3, 0) + p(3, 0) * q(2, 0);
    // quat(2, 0) = p(0, 0) * q(2, 0) + p(1, 0) * q(3, 0) + p(2, 0) * q(0, 0) - p(3, 0) * q(1, 0);
    // quat(3, 0) = p(0, 0) * q(3, 0) - p(1, 0) * q(2, 0) + p(2, 0) * q(1, 0) + p(3, 0) * q(0, 0);

    return quat;

}


BLA::Matrix<3, 3> QuaternionUtils::dcm_ned2ecef(const BLA::Matrix<3, 1> &lla) {
    BLA::Matrix<3, 3> R_ET = {
        -1.0f * sind(lla(0)) * cosd(lla(1)), -1.0f * sind(lla(1)), -1.0f * cosd(lla(0)) * cosd(lla(1)),
        -1.0f * sind(lla(0)) * sind(lla(1)), cosd(lla(1)), -1.0f * cosd(lla(0)) * sind(lla(1)),
        cosd(lla(0)), 0, -1.0f * sind(lla(0))
    };
    return R_ET;
}


BLA::Matrix<3, 1> QuaternionUtils::ecef2nedVec(const BLA::Matrix<3, 1> &ecef_meas, const BLA::Matrix<3, 1> &launch_ecef, const BLA::Matrix<3, 3> &R_ET) {
    return ~R_ET * (ecef_meas - launch_ecef);

}


BLA::Matrix<3, 1> QuaternionUtils::ned2ecefVec(const BLA::Matrix<3, 1> &ned_meas, const BLA::Matrix<3, 1> &launch_ecef, const BLA::Matrix<3, 3> &R_ET) {
    return R_ET * ned_meas + launch_ecef;
}


BLA::Matrix<3, 1> QuaternionUtils::quat2RPY(const BLA::Matrix<4, 1> &p) {
    // Not matlab, but from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // XYZ RPY but converted for ZYX order. TODO eventually rewrite to be more efficient
    BLA::Matrix<3, 1> RPY = {0, 0, 0};

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (p(0) * p(1) + p(2) * p(3));
    double cosr_cosp = 1 - 2 * (p(1) * p(1) + p(2) * p(2));
    RPY(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (p(0) * p(2) - p(1) * p(3)));
    double cosp = std::sqrt(1 - 2 * (p(0) * p(2) - p(1) * p(3)));
    RPY(1) = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (p(0) * p(3) + p(1) * p(2));
    double cosy_cosp = 1 - 2 * (p(2) * p(2) + p(3) * p(3));
    RPY(2) = std::atan2(siny_cosp, cosy_cosp);

    return RPY;
}


BLA::Matrix<4, 1> QuaternionUtils::quatConjugate(const BLA::Matrix<4, 1> &p){
    BLA::Matrix<4, 1> quat;
    quat(0, 0) = p(0);
    quat(1, 0) = -1 * p(1);
    quat(2, 0) = -1 * p(2);
    quat(3, 0) = -1 * p(3);

    return quat;
}


BLA::Matrix<3, 1> QuaternionUtils::qRot(const BLA::Matrix<4, 1> &q, BLA::Matrix<3, 1> vec) {
    return QuaternionUtils::quat2DCM(q) * vec;
}

BLA::Matrix<3, 1> QuaternionUtils::qInvRot(const BLA::Matrix<4, 1> &q, BLA::Matrix<3, 1> vec) {
    return QuaternionUtils::quat2DCMInv(q) * vec;
}


BLA::Matrix<3, 1> QuaternionUtils::g_i_ecef(const BLA::Matrix<3, 3> dcm_ned2ecef) {
    // TODO one day put in the world model and take in position
    BLA::Matrix<3, 1> grav_ned = {0, 0, 9.8};
    return dcm_ned2ecef * grav_ned;
}


BLA::Matrix<3, 1> QuaternionUtils::m_i_ecef(const BLA::Matrix<3, 3> dcm_ned2ecef) {
    // TODO one day put in the world model and take in position. School vals for now
    BLA::Matrix<3, 1> mag_ned = {19.983111, -4.8716, 46.9986145};
    return dcm_ned2ecef * mag_ned;
}


BLA::Matrix<3, 1> QuaternionUtils::normal_i_ecef(const BLA::Matrix<3, 3> dcm_ned2ecef) {
    // TODO one day put in the world model. See g_i notes
    BLA::Matrix<3, 1> normal_ned = {0, 0, -9.8};
    return dcm_ned2ecef * normal_ned;
}


BLA::Matrix<3, 3> QuaternionUtils::vecs2mat(const BLA::Matrix<3, 1> v1, const BLA::Matrix<3, 1> v2, const BLA::Matrix<3, 1> v3) {
    // TODO eventually replace with just using hstack
    BLA::Matrix<3, 3> ret = {v1(0, 0), v2(0, 0), v3(0, 0),
                            v1(1, 0), v2(1, 0), v3(1, 0),
                            v1(2, 0), v2(2, 0), v3(2, 0)};

    return ret;
}


BLA::Matrix<3, 3> QuaternionUtils::triad_EB(const BLA::Matrix<3, 1> v1_b, const BLA::Matrix<3, 1> v2_b, const BLA::Matrix<3, 1> v1_i, const BLA::Matrix<3, 1> v2_i) {
    // 1 is important, 2 is secondary
    BLA::Matrix<3, 1> v1_b_norm = v1_b / BLA::Norm(v1_b);
    BLA::Matrix<3, 1> v2_b_norm = v2_b / BLA::Norm(v2_b);
    BLA::Matrix<3, 1> v1_i_norm = v1_i / BLA::Norm(v1_i);
    BLA::Matrix<3, 1> v2_i_norm = v2_i / BLA::Norm(v2_i);

    // Inertial
    BLA::Matrix<3, 1> q_i = v1_i_norm;
    BLA::Matrix<3, 1> r_i = BLA::CrossProduct(v1_i_norm, v2_i_norm) / BLA::Norm(BLA::CrossProduct(v1_i_norm, v2_i_norm));
    BLA::Matrix<3, 1> s_i = BLA::CrossProduct(q_i, r_i);

    BLA::Matrix<3, 3> M_i = vecs2mat(q_i, r_i, s_i);


    // Body
    BLA::Matrix<3, 1> q_b = v1_b_norm;
    BLA::Matrix<3, 1> r_b = BLA::CrossProduct(v1_b_norm, v2_b_norm) / BLA::Norm(BLA::CrossProduct(v1_b_norm, v2_b_norm));
    BLA::Matrix<3, 1> s_b = BLA::CrossProduct(q_b, r_b);

    BLA::Matrix<3, 3> M_b = vecs2mat(q_b, r_b, s_b);
    BLA::Matrix<3, 3> M_b_t = ~M_b;

    BLA::Matrix<3, 3> R_EB = M_i * M_b_t;

    return R_EB;

}

// CHATGPT BABY LESSSS GOOOOOOOOO. I'm not tryna translate ts myself. Thanks Nic for sending: https://danceswithcode.net/engineeringnotes/geodetic_to_ecef/geodetic_to_ecef.html
BLA::Matrix<3,1> QuaternionUtils::lla2ecef(BLA::Matrix<3,1> lla) {
    const double a = 6378137.0;
    const double e2 = 6.6943799901377997e-3;
    double lat = lla(0);
    double lon = lla(1);
    double alt = lla(2);
    double n = a / std::sqrt(1.0f - e2 * sind(lat) * sind(lat));
    BLA::Matrix<3,1> ecef;
    ecef(0) = (n + alt) * cosd(lat) * cosd(lon);
    ecef(1) = (n + alt) * cosd(lat) * sind(lon);
    ecef(2) = (n * (1 - e2) + alt) * sind(lat);
    return ecef;
}

// Chat baby lessssgoooooo
BLA::Matrix<3, 1> QuaternionUtils::ecef2lla(BLA::Matrix<3, 1> ecef) {
    const double a = 6378137.0;
    const double e2 = 6.6943799901377997e-3;
    const double a1 = 4.2697672707157535e+4;
    const double a2 = 1.8230912546075455e+9;
    const double a3 = 1.4291722289812413e+2;
    const double a4 = 4.5577281365188637e+9;
    const double a5 = 4.2840589930055659e+4;
    const double a6 = 9.9330562000986220e-1;
    double x = ecef(0);
    double y = ecef(1);
    double z = ecef(2);
    double zp = std::abs(z);
    double w2 = x * x + y * y;
    double w = std::sqrt(w2);
    double r2 = w2 + z * z;
    double r = std::sqrt(r2);
    BLA::Matrix<3, 1> lla;
    lla(1) = std::atan2(y, x);
    double s2 = z * z / r2;
    double c2 = w2 / r2;
    double u = a2 / r;
    double v = a3 - a4 / r;
    double s, c, ss;
    if (c2 > 0.3) {
        s = (zp / r) * (1.0 + c2 * (a1 + u + s2 * v) / r);
        lla(0) = std::asin(s);
        ss = s * s;
        c = std::sqrt(1.0 - ss);
    } else {
        c = (w / r) * (1.0 - s2 * (a5 - u - c2 * v) / r);
        lla(0) = std::acos(c);
        ss = 1.0 - c * c;
        s = std::sqrt(ss);
    }
    double g = 1.0 - e2 * ss;
    double rg = a / std::sqrt(g);
    double rf = a6 * rg;
    u = w - rg * c;
    v = zp - rf * s;
    double f = c * u + s * v;
    double m = c * v - s * u;
    double p = m / (rf / g + f);
    lla(0) += p;
    lla(2) = f + m * p / 2.0;
    if (z < 0.0) {
        lla(0) *= -1.0;
    }

    lla(0) = lla(0) * (180.0 / M_PI);
    lla(1) = lla(1) * (180.0 / M_PI);
    return lla;
}


BLA::Matrix<4, 1> QuaternionUtils::normalizeQuaternion(BLA::Matrix<4, 1> quat) {
    BLA::Matrix<4, 1> new_quat = quat / BLA::Norm(quat);
    return new_quat;
}

float QuaternionUtils::cosd(float degs) {
    float pi = 3.141592653589;
    return cos(degs * (pi / 180.0f));
}

float QuaternionUtils::sind(float degs) {
    float pi = 3.141592653589;
    return sin(degs * (pi / 180.0f));
}

BLA::Matrix<3, 1> QuaternionUtils::combine_variances(const BLA::Matrix<3, 2> &A, const BLA::Matrix<2, 1> w) {
    BLA::Matrix<3, 1> output_variance;
    
    for (int i = 0; i < 3; i++) {
        output_variance(i) = w(0) * w(0) * A(i, 0) + w(1) * w(1) * A(i, 1);
    }
    return output_variance;
}

BLA::Matrix<3, 1> QuaternionUtils::fuse_measurements(const BLA::Matrix<3, 2> &A, const BLA::Matrix<2, 1> w) {
    BLA::Matrix<3, 1> output_val;
        
    for (int i = 0; i < 3; i++) {
        output_val(i) = w(0) * A(i, 0) + w(1) * A(i, 1);
    }
    return output_val;
}





