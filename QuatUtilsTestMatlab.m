%% Define vars


random_quat = [0.9344, 0.2435, 0.2435, 0.0907];
assoc_dcm = [0.8650, 0.2880, -0.4109; -0.0508, 0.8649, 0.4993; 0.4992, -0.4110, 0.7628];

random_quat2 = [0.3666, 0.4564, 0.7857, 0.1998];
random_mat = [0.8662, 0.0206, 0.2123; 0.6011, 0.9699, 0.1818; 0.7081, 0.8324, 0.1834];
random_vec = [0.3042, 0.5248, 0.4319]';

school_lla = [42.274027, -71.811788, 10]';
test_launch_ecef = [1, 2, 3]';
test_end_ecef = [1, 2, 3]';

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

rotVec2dQuat(random_vec);

 rot_vec_norm = norm(rot_vec);
    axis = rot_vec / rot_vec_norm;
    
    if min(rot_vec_norm) < 1e-9 % Small angle approx if small angle
        dq = [1; 0.5*rot_vec];
    else
        dq = [cos(rot_vec_norm / 2.0); (axis * sin(rot_vec_norm / 2.0))]';
    end
    
    q = quatmultiply(q, dq);
    q = q / norm(q);
