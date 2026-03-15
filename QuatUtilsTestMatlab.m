%% Define vars


random_quat = [0.9344, 0.2435, 0.2435, 0.0907];
assoc_dcm = [0.8650, 0.2880, -0.4109; -0.0508, 0.8649, 0.4993; 0.4992, -0.4110, 0.7628];

random_quat2 = [0.3666, 0.4564, 0.7857, 0.1998];
random_mat = [0.8662, 0.0206, 0.2123; 0.6011, 0.9699, 0.1818; 0.7081, 0.8324, 0.1834];
random_vec = [0.3042, 0.5248, 0.4319]';

school_lla = [42.274027, -71.811788, 10]';
school_ecef = [1475354, -4490428, 4268181]';
some_ecef = [1475390, -4490445, 4268161]';
some_ned = [20, -10, 15]';

v1_b = [-8.4870, -4.7330, -1.2682]';
v2_b = [-0.1710, 0.0429, 0.9843]';
v1_i = [0, 0, 9.8]';
v2_i = [1, 0, 0]';

random_unnormed_quat = [21.232, 40.243, 50.233, 19.232];


%% Tests

disp("Test quat2dcm: random_quat: {0.9344, 0.2435, 0.2435, 0.0907}. DCM:");
quat2rotm(random_quat)

disp("Test quat2dcmInv: random_quat: {0.9344, 0.2435, 0.2435, 0.0907}. DCM:");
quat2dcm(random_quat)


disp("Test skewSymmetric: random_vec: {0.3042, 0.5248, 0.4319}")
skewSymmetric(random_vec)

disp("Test rotVec2dQuat: Random vec: {0.3042, 0.5248, 0.4319}");
rot_vec = random_vec;
rot_vec_norm = norm(rot_vec);
axis = rot_vec / rot_vec_norm;

if min(rot_vec_norm) < 1e-9 % Small angle approx if small angle
    dq = [1; 0.5*rot_vec];
else
    dq = [cos(rot_vec_norm / 2.0); (axis * sin(rot_vec_norm / 2.0))]';
end
dq

disp("Test smallAnglerotVec2dQuat: Random vec: {0.3042, 0.5248, 0.4319}. Expected Quat: {1.0000, 0.1521, 0.2624, 0.2160}")
quat = [1; 0.5 * random_vec];
quat

disp("Test dcm2quat: Dcm: 0.8650    0.2880   -0.4109 -0.0508    0.8649    0.4993 0.4992   -0.4110    0.7628. Expected quat: {0.9344, 0.2435, 0.2435, 0.0907}")
rotm2quat(assoc_dcm)

disp("Test quatMultiply: Random Quats: {0.9344    0.2435    0.2435    0.0907} and {0.3666    0.4564    0.7857    0.1998} Expected Quat: {0.0220    0.4932    0.8162    0.3002}")
quatMultiply = quatmultiply(random_quat, random_quat2)

%% TODO: LLA2ECEF, ECEF2LLA

disp("Test dcm_ned2ecef: School LLA: {42.274027, -71.811788, 10}. Expected DCM: -0.2100    0.9500   -0.2310 0.6391    0.3121    0.7030 0.7399         0   -0.6727")
dcmecef2ned(school_lla(1), school_lla(2))'

disp("Test ecef2nedVec");
dcmecef2ned(school_lla(1), school_lla(2)) * (some_ecef - school_ecef)

disp("Test ned2ecefVec");
dcmecef2ned(school_lla(1), school_lla(2)) * some_ned + school_ecef

disp("Test quat2RPY: Quat: {0.9344, 0.2435, 0.2435, 0.0907}")
quat2eul(random_quat);

disp("Test quatConj: Test quat: {0.9344    0.2435    0.2435    0.0907}. Expected quat: 0.9344   -0.2435   -0.2435   -0.0907")
quatconj(random_quat)

disp("Test qRot: Quat: {0.9344, 0.2435, 0.2435, 0.0907} Vec: {0.3042, 0.5248, 0.4319}. Expected: {0.4319, 0.3042, 0.5248}")
qRot = quatrotate(quatconj(random_quat), random_vec')

disp("Test qInvRot: Quat: {0.9344, 0.2435, 0.2435, 0.0907} Vec: {0.3042, 0.5248, 0.4319}. Result:");
qInvRot = quatrotate(quatconj(random_quat), random_vec')

disp("Test g_i_ecef using school coords. Expected: TODO")
dcmecef2ned(school_lla(1), school_lla(2))' * [0; 0; -1.0 * gravitywgs84(school_lla(3), school_lla(1))]

disp("Test m_i_ecef using school coords. Expected: TODO")
dcmecef2ned(school_lla(1), school_lla(2))' * (igrfmagm(school_lla(3), school_lla(1), school_lla(2), 2026) / 1000.0)'

disp("Test normal_i_ecef using school coords. Expected: TODO")
dcmecef2ned(school_lla(1), school_lla(2))' * [0; 0; gravitywgs84(school_lla(3), school_lla(1))]

disp("Test triad_EB: Test vecs displayed here in code. Expected: -0.1710   -0.4698   -0.8660 0.0429    0.8746   -0.4830 0.9843   -0.1197   -0.1294")
initialOrientationTRIAD(v1_b, v2_b, v1_i, v2_i)


disp("Test lla2ecef: School LLA")
lla2ecef(school_lla')

disp("Test ecef2lla: School ECEF")
ecef2lla(school_ecef')


disp("Test normalizeQuaterion. Initial quat: {21.232, 40.243, 50.233, 19.232}\n Expected: {0.3014, 0.5712, 0.7130, 0.2730}")
random_unnormed_quat / norm(random_unnormed_quat)

