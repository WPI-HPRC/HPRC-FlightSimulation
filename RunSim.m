%% WPI High Power Rocketry Club - Flight Simulation
% Author: Daniel Pearson (djpearson@wpi.edu)
% Version: 1.28.2025

clear variables; close all; clc;

%% Configure constants and model data
const = setupConstants();
kins = HPRC_RocketKinematics();

% Kinematics 
inds = getMissileInds(); % Control State Indices

% Aerodynamics Model
AeroModel = initRocketAeroModel();

% Motor Model
MotorModel = initMotorModel();

ImuModel = getASM330Params();

%% Simulator Config **FOR SIMULINK USE LATER**
% Time Configuration
time.dt = 0.01; % [s] Time Step
time.t0 = 0; % [s] Initial Time
% time.tf = 60*3; % [s] Final Time
time.tf = 200;

simCfg.time = time;

%% Launch Site Initialization
% [launchLat, launchLon, launchAlt] = selectLaunchLocation();
launchLat =  42.2738703; % [deg] Latitude
launchLon = -71.8098593; % [deg] Longitude
launchAlt = 180; % [m] Altitude MSL

launchLLA = [launchLat, launchLon, launchAlt];
currLLA = launchLLA;

launch_ECEF_m = lla2ecef(launchLLA);

%% Attitude Initialization
roll_0 = deg2rad(0);
pitch_0 = deg2rad(85);
yaw_0 = deg2rad(0);

q_0 = eul2quat(roll_0, pitch_0, yaw_0);

%$ Angular Rate Initialization
w_ib_x = 0.00; % [rad/s]
w_ib_y = 0.00; % [rad/s]
w_ib_z = 0.00; % [rad/s]

%% Velocity Initialization
Vx_E_0 = 1e-2; % [m/s]
Vy_E_0 = 1e-2; % [m/s]
Vz_E_0 = 1e-2; % [m/s]

%% Initial Mass
m_0 = kins.m_0 + MotorModel.emptyWt + MotorModel.propWt;

%% State Initialization
x_0 = [
    q_0';
    launch_ECEF_m';
    Vx_E_0;
    Vy_E_0;
    Vz_E_0;
    w_ib_x;
    w_ib_y;
    w_ib_z;
    m_0;
];

x_t = x_0;

%% State Data Storage
t = time.t0;

numTimePts = time.tf / time.dt+1;

tRecord = nan(1, numTimePts);
tRecord(1,1) = t;

xRecord = nan(length(x_0), numTimePts);
xRecord(:,1) = x_t;

tSpan = [0, time.tf];  % Start time and end time
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);  % Tolerances for ode45

% Guidance Storage
cmdHist = zeros(4, numTimePts);
% Initialize buffer and max actuation rate for canards (e.g., 0.1 rad/s)
stateBuffer = nan(size(x_t, 1), 2);

%% Fill steady state data for set period of time (Simulate Launch Pad)
steadyStateDuration = 5; % [s]
numSteadyPts = steadyStateDuration / time.dt;

% Pre-Fill steady state values
tRecord(1:numSteadyPts) = linspace(-steadyStateDuration+time.dt, time.t0, numSteadyPts);
xRecord(:, 1:numSteadyPts) = repmat(x_0, 1, numSteadyPts); % Repeat initial state
cmdHist(:, 1:numSteadyPts) = zeros(4, numSteadyPts); % Zero canard deflections
colNum = numSteadyPts;

while(currLLA(3) >= -5)
    % Iterate
    colNum = colNum + 1;

    % Update buffer with the latest state; shift older states
    stateBuffer(:, 2) = stateBuffer(:, 1);
    stateBuffer(:, 1) = x_t;

    % Wrap dynamic model in anonymous function for ODE solver
    rocketModelODE = @(t, x_t) RocketDynamicModel(x_t, t, AeroModel, MotorModel, const, kins, inds);

    % Call ODE solver for small time step from current t to t + dt
    [t_out, x_out] = ode45(rocketModelODE, [t, t + time.dt], x_t, options);

    % Update time and state variables with the last output from ode45
    t = t_out(end);
    x_t = x_out(end, :)';  % Transpose to maintain consistency with your original state vector format

    % Convert ECEF position to LLA for altitude check
    currLLA = ecef2lla([x_t(inds.px_ecef)', x_t(inds.py_ecef)', x_t(inds.pz_ecef)']);

    % Record the results for future analysis
    xRecord(:, colNum) = x_t;
    tRecord(1, colNum) = t;

    % Extract Acceleration Readings
    [~, accel_ecef] = RocketDynamicModel(x_out(end, :)', t, AeroModel, MotorModel, const, kins, inds);

    sensorReading = generateIMU_Readings(x_t, accel_ecef, ImuModel, inds, const);
end

%% Plot Vehicle Trajectory
lla = ecef2lla([xRecord(inds.px_ecef, :)', xRecord(inds.py_ecef, :)', xRecord(inds.pz_ecef, :)']);

% Create a geoglobe
uif = uifigure('Name', 'Vehicle Trajectory');
g = geoglobe(uif);

geoplot3(g, lla(:, 1), lla(:,2), lla(:,3),"y");

%% Euler Angles
eulHist = quat2eul(xRecord(1:4, :)', 'ZYX');

yawHist   = rad2deg(eulHist(:,1));
pitchHist = rad2deg(eulHist(:,2));
rollHist  = rad2deg(eulHist(:,3));

% Plot
figure('Name', 'Orientation');
plot(tRecord(:), yawHist);
hold on;
plot(tRecord(:), pitchHist);
plot(tRecord(:), rollHist);
hold off;
title("Euler Angles");
legend('Yaw', 'Pitch', 'Roll');

figure('Name', 'Angular Velocity');
plot(tRecord(:), xRecord(inds.w_ib_x,:));
hold on;
plot(tRecord(:), xRecord(inds.w_ib_y,:));
plot(tRecord(:), xRecord(inds.w_ib_z,:));
hold off;
title("Angular Velocity");
legend('P', 'Q', 'R');

%% Vehicle State

figure('Name', 'Altitude');
plot(tRecord(:), lla(:,3))
title("Altitude Vs. Time");
ylabel("Altitude (m)");
xlabel("Time (s)");
grid on;

velocityHist = vecnorm(xRecord(inds.vel, :));

figure('Name', 'Velocity');
plot(tRecord(:), velocityHist);
title('Velocity Vs. Time');
ylabel("Velocity (m/s)");
xlabel("Time (s)");
grid on;

% Altitude Vs Downrange
downrange = getHaversine(launchLLA(1), launchLLA(2), lla(:,1), lla(:,2), const);
figure('Name', 'Conops');
plot(downrange, lla(:,3));
title("Mission Conops");
ylabel("Altitude (m)");
xlabel("Downrange (m)");
grid on;