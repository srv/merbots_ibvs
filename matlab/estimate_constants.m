% This script can be used to approximate the constants for visual servoing
% according to a set of predefined params and velocities

% These values are an approximation and should be tuned later

% Calibration params
im_width = 480;
im_height = 360;
fs = 370.0;
fs_2 = fs^2;
ppoint = [240, 180.0];

% Maximum velocities
max_vx = 0.6;
max_vy = 0.2;
max_wz = 0.25;

% Points
% -- Current points
pt_tl = [0.0, 0.0];
pt_tr = [im_width, 0.0];
pt_bl = [0.0, im_height];

% -- Desired positions
im_width_step = im_width / 4.0;
im_height_step = im_height / 4.0;
des_pt_tl = [im_width_step, im_height_step];
des_pt_tr = [im_width - im_width_step, im_height_step];
des_pt_bl = [im_width_step, im_height - im_height_step];

% Heights
% Let's suppose that all points are at 1 m of distance
z = 1.0;
z1 = z;
z2 = z;
z3 = z;

% Defining common variables
% Principal point
u0 = ppoint(1);
v0 = ppoint(2);
% Point coords
u1 = pt_tl(1);
v1 = pt_tl(2);
u2 = pt_tr(1);
v2 = pt_tr(2);
u3 = pt_bl(1);
v3 = pt_bl(2);
% Desired coords
u1_d = des_pt_tl(1);
v1_d = des_pt_tl(2);
u2_d = des_pt_tr(1);
v2_d = des_pt_tr(2);
u3_d = des_pt_bl(1);
v3_d = des_pt_bl(2);

% Computing the interaction matrix
L = zeros(6);

% Getting primes
up1 = u1 - u0;
vp1 = v1 - v0;
up2 = u2 - u0;
vp2 = v2 - v0;
up3 = u3 - u0;
vp3 = v3 - v0; 

% Squared values
up1_2 = up1^2;
vp1_2 = vp1^2;
up2_2 = up2^2;
vp2_2 = vp2^2;
up3_2 = up3^2;
vp3_2 = vp3^2;

% L Column 1
L(1, 1) = -(fs / z1);
L(3, 1) = -(fs / z2);
L(5, 1) = -(fs / z3);

% L Column 2
L(2, 2) = -(fs / z1);
L(4, 2) = -(fs / z2);
L(6, 2) = -(fs / z3);

% L Column 3
L(1, 3) = up1 / z1;
L(2, 3) = vp1 / z1;
L(3, 3) = up2 / z2;
L(4, 3) = vp2 / z2;
L(5, 3) = up3 / z3;
L(6, 3) = vp3 / z3;

% L Column 4
L(1, 4) = (up1 * vp1) / fs;
L(2, 4) = (fs_2 + vp1_2) / fs;
L(3, 4) = (up2 * vp2) / fs;
L(4, 4) = (fs_2 + vp2_2) / fs;
L(5, 4) = (up3 * vp3) / fs;
L(6, 4) = (fs_2 + vp3_2) / fs;

% L Column 5
L(1, 5) = -(fs_2 + up1_2) / fs;
L(2, 5) = -(up1 * vp1) / fs;
L(3, 5) = -(fs_2 + up2_2) / fs;
L(4, 5) = -(up2 * vp2) / fs;
L(5, 5) = -(fs_2 + up3_2) / fs;
L(6, 5) = -(up3 * vp3) / fs;

% L Column 6
L(1, 6) = vp1;
L(2, 6) = -up1;
L(3, 6) = vp2;
L(4, 6) = -up2;
L(5, 6) = vp3;
L(6, 6) = -up3;

% Filling the error vector
s = zeros(6, 1);
s(1,1) = u1 - u1_d;
s(2,1) = v1 - v1_d;
s(3,1) = u2 - u2_d;
s(4,1) = v2 - v2_d;
s(5,1) = u3 - u3_d;
s(6,1) = v3 - v3_d;

% Computing the error
v_t = inv(L) * s;

% Computing vx
lamb_vx = max_vx / v_t(3)

% Computing vy
lamb_vy = max_vy / v_t(1)

% Computing wz
lamb_wz = max_wz / v_t(5)
