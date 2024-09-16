clear variables; close all; clc;

%% Time Configuration
navRate = 100; % [Hz] Navigation rate
dt = 1/navRate;
t = 0;

% Configure RNG Seed
rng(1);

%% Load Simulation Data
load("flightSimulations\RocketSimulation.mat");
tRecord = zeros(1, length(sRecord));

%% Configure IMU Errors
% Accel Biases, Noise, Scale Factors
sigmaAccel = 70e-6; % [g/sqrt(hz)]
accelBias = 0.02 * randn(3,1)';
accelNoise = sigmaAccel * randn(3,1)';
accelScaleFactor = [1.02, 0.98, 1.01];
accelErrors = [accelBias; accelNoise; accelScaleFactor];

% Gyro Biases, Noise, Scale Factors
gyroBias = [0.05, -0.03, 0.02];
gyroNoise = [0.01, 0.01, 0.01];
gyroScaleFactor = [1.02, 0.98, 1.01];
gyroErrors = [gyroBias; gyroNoise; gyroScaleFactor];

IMUErrors = [accelErrors; gyroErrors];

%% Initialize EKF
QUAT_0 = [0.0189; -0.9493; 0.3084; -0.0581];
POS_0 = [0; 0; -1400];
VEL_0 = [0; 0; 0.1];
ACC_BIAS = [0; 0; 0;];
ACC_NOISE = [0; 0; 0];
ACC_SCALE = [1; 1; 1];
GYRO_BIAS = [0; 0; 0;];
GYRO_NOISE = [0; 0; 0];
GYRO_SCALE = [1; 1; 1];

x_whole = [QUAT_0; VEL_0; POS_0; ACC_BIAS; ACC_NOISE; ACC_SCALE; GYRO_BIAS; GYRO_NOISE; GYRO_SCALE];

P0 = zeros(28,28);

P0(1:4, 1:4) = diag(1e-6*ones(4,1));
P0(5:7, 5:7) = diag(0.5*ones(3,1));
P0(8:10, 8:10) = diag(2*ones(3,1));
% Accel Errors
P0(11:13, 11:13) = (500e-6*eye(3)).^2; % [500 micro-g]
P0(14:16, 14:16) = (sigmaAccel*eye(3)).^2;
P0(17:19, 17:19) = (100/1e6*eye(3)).^2;
% Gyro Errors
P0(20:22, 20:22) = (8.273e-3*eye(3)).^2; % [5 dps]
P0(23:25, 23:25) = (2.8e-3*eye(3)).^2;
P0(26:28, 26:28) = (100/1e6*eye(3)).^2;

x_whole_Record = zeros(28, length(sRecord));
w_b_corrupt_Record = zeros(3, length(sRecord));

Q = eye(28) * 0.1;

P = P0;

%% Run EKF

for i = 1:length(sRecord)
    t = t + dt;

    w_b_true = [
        sRecord(4,i);
        sRecord(5,i);
        sRecord(6,i);
    ];

    sf_b_true = [
        sRecord(1, i);
        sRecord(2, i);
        sRecord(3, i);
    ];

    % Simulate IMU errors (bias, noise, scale factors)
    [sf_b_corrupt, w_b_corrupt] = addSensorNoise(sf_b_true, w_b_true, IMUErrors, dt);

    C_ib = quat2rotm(x_whole(1:4)');

    sf_i_corrupt = (C_ib' * sf_b_corrupt - [0; 0; 1]) * 9.81;

    % State Update
    qw = x_whole(1);
    qx = x_whole(2);
    qy = x_whole(3);
    qz = x_whole(4);
    vx = x_whole(5);
    vy = x_whole(6);
    vz = x_whole(7);
    px = x_whole(8);
    py = x_whole(9);
    pz = x_whole(10);

    wx = w_b_corrupt(1);
    wy = w_b_corrupt(2);
    wz = w_b_corrupt(3);
    sfx = sf_i_corrupt(1);
    sfy = sf_i_corrupt(2);
    sfz = sf_i_corrupt(3);

    % Calculate State Update Function
    f = [
        0.5*(-qx*wx - qy*wy - qz*wz);
        0.5*(qw*wx - qz*wy + qy*wz);
        0.5*(qz*wx + qw*wy - qx*wz);
        0.5*(-qy*wx + qx*wy + qw*wz);
        sfx;
        sfy;
        sfz;
        vx;
        vy;
        vz;
        0; 0; 0; % Accelerometer bias update (assumed constant)
        0; 0; 0; % Accelerometer noise update (assumed constant)
        0; 0; 0; % Accelerometer scale factor update (assumed constant)
        0; 0; 0; % Gyro bias update (assumed constant)
        0; 0; 0; % Gyro noise update (assumed constant)
        0; 0; 0; % Gyro scale factor update (assumed constant)
    ];

    % Update the state
    x_whole = x_whole + dt * f;

    % Normalize quaternion
    x_whole(1:4) = x_whole(1:4) / norm(x_whole(1:4));

    % Continuous State Transition Matrix
    F = zeros(28, 28);

    % F(1:4, 1:4) = [
    %     0 0.5*wx -0.5*wy -0.5*wz;
    %     -0.5*wx 0 0.5*wz -0.5*wy;
    %     -0.5*wy -0.5*wz 0 0.5*wx;
    %     -0.5*wz 0.5*wy -0.5*wx 0;
    % ];
    % 
    % F(5:7, 11:13) = diag(1 ./ x_whole(17:19));
    % F(5:7, 17:19) = -diag((sf_i_corrupt - x_whole(11:13)) ./ (x_whole(17:19).^2));
    % F(8:10, 5:7) = eye(3) * dt;
    % 
    % % Including the effect of biases and scale factors
    % F(5:7, 14:16) = eye(3);
    % F(20:22, 23:25) = -eye(3) * 0.5 * dt; % Example, adapt as needed

    phi = eye(28) + F * dt;

    % State transition
    x_whole = phi * x_whole;

    % Process covariance propagation
    P = phi * P * phi' + Q * dt;

    % Record the state
    x_whole_Record(:,i) = x_whole;
    tRecord(1,i) = t;
    w_b_corrupt_Record(:,i) = w_b_corrupt;
end

% Plotting the results
figure;
hold on;
plot(tRecord, x_whole_Record(10,:), "Color", "red"); % Plotting position z
plot(tRecord, xRecord(7,:), "Color", "blue"); % Plotting position z from the simulation data
hold off;
xlabel('Time (s)');
ylabel('Position Z (m)');
legend('Estimated', 'Actual');
title('Position Z Estimation vs Actual');
