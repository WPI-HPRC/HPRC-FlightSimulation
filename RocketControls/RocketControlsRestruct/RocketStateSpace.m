clear variables; close all; clc;

%% Rocket Constants
S_r = 0.0192898; % [m^2] Cross Sectional area of body tube and fins
m = 22.720; % [kg] Rocket Mass
C_d = 0.4; % Average assumed C_D

% Motor Constants (CTI M3400)
mFuelRate = 1.632; % [kg/s] Mass flow rate of motor burn
T_motor = 3414; % [N] Average Motor Thrust

% Moment of Inertia
I_x = 26.413;
I_y = 26.413;
I_z = 0.107;



%% Global Constants
g = 9.81; % [m/s^2] Gravitational Acceleration of Earth

% Dynamic Model
A = [
    0 -0.005 -0.005 -0.005 0 0 0 0 0 0 0         0         0        0;
    0.005 0 0.005 -0.005 0 0 0 0.5 0 0 0         0         0        0;
    0.005 -0.005 0 0.005 0 0 0 0 0.5 0 0         0         0        0;
    0.005 0.005 -0.005 0 0 0 0 0 0 0.5 0         0         0        0;
    0     0      0     0 0 0 0 0 0 0   1         0         0        0;
    0     0      0     0 0 0 0 0 0 0   0         1         0        0;
    0     0      0     0 0 0 0 0 0 0   0         0         1        0;
    0     0      0     0 0 0 0 0 0 0   0         0         0        0;
    0     0      0     0 0 0 0 0 0 0   0         0         0        0;
    0     0      0     0 0 0 0 0 0 0   0         0         0        0;
    0     0      0     0 0 0 0 0 0 0   5.7837e3 -2.8918e3 -2.8918e3 0;
    0     0      0     0 0 0 0 0 0 0  -2.8918e3  5.7837e3 -2.8918e3 0;
    0     0      0     0 0 0 0 0 0 0  -2.8918e3 -2.8918e3  5.7837e3 0;
    0     0      0     0 0 0 0 0 0 0 0 0 0 0;
];

B = [
    0  0;
    0  0;
    0  0;
    0  0;
    0  0;
    0  0;
    0  0;
    0  0;
    0  0;
    0  0;
    0  86.7550;
    0  86.7550;
    0  86.7550;
    0 -1.6320;
];

C = eye(length(A));

D = 0;

states = {"q_w", "q_i", "q_j", "q_k", "p_x", "p_y", "p_z", "\omega_x", "\omega_y", "\omega_z", "v_x", "v_y", "v_z", "mass"};
inputs = {"A_b", "T_h"};

sys = ss(A,B,C,D,'statename', states, 'inputname', inputs);

TFs = tf(sys);   