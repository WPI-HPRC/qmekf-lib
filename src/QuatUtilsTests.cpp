#include <Arduino.h>

#include "BasicLinearAlgebra.h"

#include "QuaternionUtils.h"

using namespace QuaternionUtils;
// This line is a game changer

void setup() {


}

void loop() {
        BLA::Matrix<4, 1> random_quat = {0.9344, 0.2435, 0.2435, 0.0907};
        BLA::Matrix<3, 3> assoc_dcm = {0.8650, 0.2880, -0.4109, -0.0508, 0.8649, 0.4993, 0.4992, -0.4110, 0.7628};
        BLA::Matrix<4, 1> random_quat2 = {0.3666, 0.4564, 0.7857, 0.1998};
        BLA::Matrix<3, 3> random_mat = {0.8662, 0.0206,0.2123, 0.6011, 0.9699, 0.1818, 0.7081, 0.8324, 0.1834};
        BLA::Matrix<3, 1> random_vec = {0.3042, 0.5248, 0.4319};
        BLA::Matrix<3, 1> school_lla = {42.274027, -71.811788, 10};
        BLA::Matrix<3, 1> school_ecef = {1475354, -4490428, 4268181};
        BLA::Matrix<3, 1> some_ecef = {1475390, -4490445, 4268161};
        BLA::Matrix<3, 1> some_ned = {20, -10, 15};
        BLA::Matrix<3, 1> v1_b = {-8.4870, -4.7330, -1.2682};
        BLA::Matrix<3, 1> v2_b = {-0.1710, 0.0429, 0.9843};
        BLA::Matrix<3, 1> v1_i = {0, 0, 9.8};
        BLA::Matrix<3, 1> v2_i = {1, 0, 0};
        BLA::Matrix<4, 1> random_unnormed_quat = {21.232, 40.243, 50.233, 19.232};
        
        Serial.println("Test quat2dcm: Quat: {0.9344, 0.2435, 0.2435, 0.0907}");
        printMatHighDef(quat2DCM(random_quat));

        Serial.println("Test quat2dcmInv: Quat: {0.9344, 0.2435, 0.2435, 0.0907}");
        printMatHighDef(quat2DCMInv(random_quat));


        Serial.println("Test skewSymmetric: Random vec: {0.3042, 0.5248, 0.4319}");
        printMatHighDef(skewSymmetric(random_vec));


        Serial.println("Test rotVec2dQuat: Random vec: {0.3042, 0.5248, 0.4319}. Expected Quat: {0.9315, 0.1486, 0.2564, 0.2110} ");

        printMatHighDef(rotVec2dQuat(random_vec));


        Serial.println("Test smallAnglerotVec2dQuat: Random vec: {0.3042, 0.5248, 0.4319}. Expected Quat: {1.0000, 0.1521, 0.2624, 0.2160}");

        printMatHighDef(smallAnglerotVec2dQuat(random_vec));


        Serial.println("Test dcm2quat: Dcm: 0.8650    0.2880   -0.4109 \
        -0.0508    0.8649    0.4993 \
        0.4992   -0.4110    0.7628. Expected quat: {0.9344, 0.2435, 0.2435, 0.0907}");

        printMatHighDef(dcm2quat(assoc_dcm));


        Serial.println("Test quatMultiply: Random Quats: {0.9344    0.2435    0.2435    0.0907} and {0.3666    0.4564    0.7857    0.1998} \
                Expected Quat: {0.0220    0.4932    0.8162    0.3002}");

        printMatHighDef(quatMultiply(random_quat, random_quat2));

        Serial.println("Test dcm_ned2ecef: School LLA: {42.274027, -71.811788, 10}. Expected DCM: \
                -0.2100    0.9500   -0.2310 \
                0.6391    0.3121    0.7030 \
                0.7399         0   -0.6727");
        printMatHighDef(dcm_ned2ecef(school_lla));
        
        
        Serial.println("Test ecef2nedVec:");
        printMatHighDef(ecef2nedVec(some_ecef, school_ecef, dcm_ned2ecef(school_lla)));
        
        Serial.println("Test ned2ecefVec:");
        printMatHighDef(ned2ecefVec(some_ned, school_ecef, dcm_ned2ecef(school_lla)));

        Serial.println("Test quat2RPY: Quat: {0.9344, 0.2435, 0.2435, 0.0907}");
        printMatHighDef(quat2RPY(random_quat));

        Serial.println("Test quatConj: Test quat: {0.9344    0.2435    0.2435    0.0907}. Expected quat: 0.9344   -0.2435   -0.2435   -0.0907");
        printMatHighDef(quatConjugate(random_quat));

        Serial.println("Test qRot: Quat: {0.9344, 0.2435, 0.2435, 0.0907} Vec: {0.3042, 0.5248, 0.4319}. Expected: {0.4319, 0.3042, 0.5248}");
        printMatHighDef(qRot(random_quat, random_vec));

        Serial.println("Test qInvRot: Quat: {0.9344, 0.2435, 0.2435, 0.0907} Vec: {0.3042, 0.5248, 0.4319}. Expected: {0.5248, 0.4319, 0.3042}");
        printMatHighDef(qInvRot(random_quat, random_vec));

        Serial.println("Test g_i_ecef using school coords.");
        printMatHighDef(g_i_ecef(dcm_ned2ecef(school_lla)));

        Serial.println("Test m_i_ecef using school coords.");
        printMatHighDef(m_i_ecef(dcm_ned2ecef(school_lla)));

        Serial.println("Test normal_i_ecef using school coords.");
        printMatHighDef(normal_i_ecef(dcm_ned2ecef(school_lla)));
        
        Serial.println("Test vecs2mat.");
        printMatHighDef(vecs2mat(v1_b, v2_b, v1_i));

        Serial.println("Test triad_EB.");
        printMatHighDef(triad_EB(v1_b, v2_b, v1_i, v2_i));

        Serial.println("Test lla2ecef: School LLA");

        printMatHighDef(lla2ecef(school_lla));

        Serial.println("Test ecef2lla: School ECEF");
        printMatHighDef(ecef2lla(school_ecef));

        Serial.println("Test normalizeQuaterion. Initial quat: {21.232, 40.243, 50.233, 19.232}\n \
                Expected: {0.3014, 0.5712, 0.7130, 0.2730}");
        printMatHighDef(normalizeQuaternion(random_unnormed_quat));

        Serial.println("Test cosd. Initial angle: -270. Expected: 0");
        Serial.println(cosd(-270));

        Serial.println("Test sind. Initial angle: -270. Expected: 1");
        Serial.println(sind(-270));

        // TODO combine variances
        // TODO fuse measurements

        // TODO pinv


        Serial.println("Test extractSub. Expected {42.274027, 10}");
        BLA::Matrix<2, 1> inds = {0, 2};
        printMatHighDef(extractSub(school_lla, inds));

        Serial.println("Test extractDiag. Expected: {0.8662, 0.9699, 0.1834}");
        printMatHighDef(extractDiag(random_mat));

        Serial.println("Test BLA cross prod");
        printMatHighDef(BLA::Cross(random_vec, v1_b));

        Serial.println("Test norm");
        printMatHighDef(v1_b / BLA::Norm(v1_b));







        // TODO toDiag
        // TODO vecMax
        // TODO vecMaxInd
        // TODO setSub
        // TODO vstack
        // TODO vstackList
        // TODO hstack
        // TODO sum
        // TODO matToVec



        Serial.println("\n\n\n\n\n");
        delay(10000);
}
