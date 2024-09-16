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
gyroErrors = [accelBias; accelNoise; accelScaleFactor];

IMUErrors = [accelErrors; gyroErrors];

[sf_b, w_b] = addSensorNoise([1;1;1], [0,0,0], IMUErrors, dt);

%% Initialize EKF
QUAT_0 = [0.0189; -0.9493; 0.3084; -0.0581];

% Initialize Position with spaceport america launch location
launchPosLLA = [32.94065483216892, -106.9217294526612, 1400];
launchPosNED = lla2ned(launchPosLLA, [0,0,0], 'ellipsoid');

POS_0 = launchPosNED;
VEL_0 = [0; 0; 0.1];
ACC_BIAS = [0; 0; 0;];
ACC_NOISE = [0; 0; 0];
ACC_SCALE = [0; 0; 0];
GYRO_BIAS = [0; 0; 0;];
GYRO_NOISE = [0; 0; 0];
GYRO_SCALE = [0; 0; 0];

x_whole = [QUAT_0; POS_0; VEL_0; ACC_BIAS; ACC_NOISE; ACC_SCALE; GYRO_BIAS; GYRO_NOISE; GYRO_SCALE];

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

Q = eye(28) * 0.1;

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

    [sf_b_corrupt, w_b_corrupt] = addSensorNoise(sf_b_true, w_b_true, IMUErrors, dt);
    % sf_b_corrupt = sf_b_true;
    % w_b_corrupt = w_b_true;

    C_ib = quat2rotm([x_whole(1), x_whole(2), x_whole(3), x_whole(4)]);

    sf_i_corrupt = ((C_ib * sf_b_corrupt) - [0; 0; 1]) * 9.81;
    w_b_corrupt = w_b_corrupt * (pi/180);

    % Position update
    x_whole(5:7) = x_whole(5:7) + x_whole(8:10) * dt + (sf_i_corrupt + x_whole(17:19)) * (dt^2 / 2);

    % Velocity update
    x_whole(8:10) = x_whole(8:10) + dt * (sf_i_corrupt + x_whole(17:19));

    %% Jacobian of State Propagation
    F = zeros(28);
    Qt = zeros(28);

    GM = 3.986e14; % [m^3/s^2] Gravatation Constant
    r = norm(x_whole(8:10)); % Radius of flight path
    GravGradNED = (GM / r^3) * [-1 0 0; 0 -1 0; 0 0 2];

    % Pos WRT Vel
    F(8:10, 5:7) = eye(3);

    % Vel WRT Pos
    F(5:7, 8:10) = GravGradNED;

    % Process covariance propagation
    % P = phi * P0 * phi' + Q;

    % Measurement update - Here you would typically update based on sensor measurements
    % For simplicity, let's assume we don't have any measurements to update with
    x_whole_Record(:,i) = x_whole;
    tRecord(1,i) = t;
end

% Plotting the results
figure;
hold on;
plot(tRecord, x_whole_Record(7,:), "Color", "red"); % Plotting position x
plot(tRecord, xRecord(7,:), "Color", "blue"); % Plotting position x from the simulation data
xlabel('Time (s)');
ylabel('Position Z (m)');
legend('Estimated', 'Actual');
title('Position x Estimation vs Actual');
