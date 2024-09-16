%% RK4 Rocket Control Simulation
% Author: Daniel Pearson (djpearson@wpi.edu) 
% Version: 4/16/2024

clear variables; close all; clc;

%% Rocket Drag Polar

% RasAero
machNumber = [0.01; 0.1; 0.2; 0.3; 0.4; 0.5; 0.6; 0.7; 0.8; 0.9; 1; 1.1];
cDVal = [0.444; 0.392; 0.374; 0.364; 0.358; 0.353; 0.352; 0.356; 0.360; 0.363; 0.497; 0.575];

% Use least squares estimate to solve for C
X = [machNumber.^5 machNumber.^4 machNumber.^3, machNumber.^2, machNumber, ones(size(machNumber))];

C = X \ cDVal;
cdPolar = @(x) C(1)*x^5 + C(2)*x^4 + C(3)*x^3 + C(4)*x^2 + C(5)*x + C(6);

%% Rocket CL Interpolation
clData = readtable('CL_Data.CSV');

AOA = clData.Alpha;
Mach = clData.Mach;
CL = clData.CL;

F = scatteredInterpolant(AOA, Mach, CL, 'linear');

calculateCL = @(aoa, mach) F(aoa, mach);

%% RK4 Propagation
dt = 0.01;
t_0 = 0;
t = t_0;
t_f = 50;
timeSlice = t_0:dt:t_f;
numTimePts = t_f / dt+1;
tRecord = zeros(1, numTimePts);
tRecord(1,1) = t;

%% Initialize RK4
v_B = [0;0;0.01]; % [m/s]
q_0 = [0, -0.9981348, 0, 0.0610485];
% q_0 = [0.10,-0.92,0.38,-0.03];

% Launch Site Initial Coords
launchSiteLLA = [32.94051076670848, -106.92168654133573, 1400];
pos_initial_NED = [0;0;0];

ecefInitial = lla2ecef(launchSiteLLA);

R_TB = quat2rotm(q_0);

initialLLArad = deg2rad(launchSiteLLA);

C_IN = [
    -sin(initialLLArad(1))*cos(initialLLArad(2)) -sin(initialLLArad(2)) -cos(initialLLArad(1))*cos(initialLLArad(2));
    -sin(initialLLArad(1))*sin(initialLLArad(2)) cos(initialLLArad(2)) -cos(initialLLArad(1))*sin(initialLLArad(2));
    cos(initialLLArad(1)) 0 -sin(initialLLArad(1));
];

rotm2eul(R_TB);

v_NED = R_TB' * v_B;

v_ECEF = C_IN * v_NED;

m_fuel = 4.776; % [kg] Solid rocket fuel mass
m_empty = 22.72 + 3.342; % [kg] Empty Rocket Mass
m_full = m_empty + m_fuel;

% qw, qi, qj, qk, px, py, pz, wx, wy, wz, vx, vy, vz, m
x_t = [q_0(1); q_0(2); q_0(3); q_0(4); ecefInitial(1); ecefInitial(2); ecefInitial(3); 0; 0; 0; v_ECEF(1); v_ECEF(2); v_ECEF(3); m_full];


xRecord = zeros(length(x_t), numTimePts);
xRecord(:, 1) = x_t;

%% Sensor Readings
% Acx, Acy, Acz, Gyx, Gyy, Gyz, Magx, Magy, Magz
a_t = [0; 0; 0; 0; 0; 0; 0; 0; 0];
sRecord = zeros(length(a_t), numTimePts-1);
aoaRecord = zeros(length(a_t), numTimePts-1);

colNum = 1;
while(t < t_f)
    colNum = colNum + 1;
    t = t+dt;

    if(x_t(14) >= m_empty)
        throttle = 1;
    else
        throttle = 0;
    end
    abPct = 0;

    u_t = [abPct; throttle];

    [sysDyn1, omegaDot, AOA] = systemDynamics(x_t, u_t, cdPolar, calculateCL);
    k1 = dt*sysDyn1;
    [sysDyn2, ~, ~] = systemDynamics(x_t + (1/2)*k1, u_t, cdPolar, calculateCL);
    k2 = dt*sysDyn2;
    [sysDyn3, ~, ~] = systemDynamics(x_t + (1/2)*k2, u_t, cdPolar, calculateCL);
    k3 = dt*systemDynamics(x_t+(1/2)*k2, u_t, cdPolar, calculateCL);
    [sysDyn4, ~, ~] = systemDynamics(x_t + k3, u_t, cdPolar, calculateCL);
    k4 = dt*systemDynamics(x_t+k3, u_t, cdPolar, calculateCL);

    x_t = x_t + (1/6)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4;

    xRecord(:,colNum) = x_t;
    tRecord(1,colNum) = t;

    %% Calculate Sensor Readings
    sysDyn = k4 / dt; % Bring back to per second

    % Collect acceleration readings and rotate into body frame
    ax = sysDyn(11) / 9.81; % [G]
    ay = sysDyn(12) / 9.81; % [G]
    az = sysDyn(13) / 9.81 + 1; % [G] Add one G to negate gravity compensation
    aR = [ax; ay; az];

    aR = R_TB * aR;

    omega = (omegaDot / dt);

    sRecord(:,colNum) = [aR; omega; 0;0;0];
    aoaRecord(:,colNum) = AOA;
    
end

%% Results
llaRecord = ecef2lla(xRecord(5:7, :)');

figure();
plot(tRecord, llaRecord(:,3));
title("Altitude Vs. Time");
xlabel("Time (s)");
ylabel("Altitude (m)");
grid on;
% 
% Plot Vehicle Track
uif = uifigure();
g = geoglobe(uif);
geoplot3(g, llaRecord(:,1), llaRecord(:,2), llaRecord(:,3));
% 
% % Plot Vehicle AOA
% figure();
% plot(tRecord, aoaRecord);
% title("AOA Vs. Time");
% xlabel("Time (s)");
% ylabel("AOA (deg)");
% grid on;

% figure();
% plot(tRecord, xRecord(11,:)); % X-Velocity vs Time
% title("X-Velocity Vs. Time");
% xlabel("Time (s)");
% ylabel("X-Velocity (m/s)");
% grid on;
% 
% figure();
% plot(tRecord, xRecord(12,:)); % Y-Velocity vs Time
% title("Y-Velocity Vs. Time");
% xlabel("Time (s)");
% ylabel("Y-Velocity (m/s)");
% grid on;
% 
% figure();
% plot(tRecord, xRecord(13,:)); % Z-Velocity vs Time
% title("Z-Velocity Vs. Time");
% xlabel("Time (s)");
% ylabel("Z-Velocity (m/s)");
% grid on;

% euler_angles = rad2deg(quat2eul(xRecord(1:4,:)', 'XYZ'));
% % Plot Roll angles against time
% figure();
% plot(tRecord, euler_angles(:,1)); % Roll angle vs Time
% title("Roll Angle Vs. Time");
% xlabel("Time (s)");
% ylabel("Roll Angle (deg)");
% grid on;

%% Save Sensor Readings
save('./Data/RocketSimulation', "sRecord", "xRecord");

function [x_dot, omega_dot, AOA] = systemDynamics(x, u, cdPolar, calculateCL)
    %% Earth Constants
    g = 9.81; % [m/s/s] Gravitational Acceleration on Earth
    R = 287; % [J/kgK]

    z_cp = 0.285 - 0.233;
    x_cp = 0;
    y_cp = 0;

    curr_LLA = ecef2lla(x(5:7)');
    % curr_LLA(1:2) = deg2rad(curr_LLA(1:2)); 
    currLat = deg2rad(curr_LLA(1));
    currLon = deg2rad(curr_LLA(2));
    currAlt = curr_LLA(3);

    R_TB = quat2dcm([x(1),x(2),x(3),x(4)]);
    R_IT = [
        -sin(currLat)*cos(currLon) -sin(currLon) -cos(currLat)*cos(currLon);
        -sin(currLat)*sin(currLon) cos(currLon) -cos(currLat)*sin(currLon);
        cos(currLat) 0 -sin(currLat);
    ];

    atmo = AtmosphericModel(currAlt);

    rho_alt = atmo.getDensity(); % Density at altitude

    a = sqrt(1.4*R*atmo.getTemperature()); % Speed of sound

    %% Vehicle Constants
    S_r = 0.0192898; % [m^2] Cross Sectional area of body tube and fins
    m = 22.720; % [kg] Rocket Mass 
    
    %% Motor Constants
    mFuelRate = 1.632; % [kg/s] Mass flow rate of motor burn
    T_motor = 3414; % [N] Average motor thrust

    % Moment of Inertia
    I_x = 26.413;
    I_y = 26.413;
    I_z = 0.107;

    v_ECEF = [x(11); x(12); x(13)];

    % Calculate mach number
    M = norm(v_ECEF) / a;

    % Rotate velocity into the body frame
    v_T = R_IT' * v_ECEF;
    v_B = R_TB' * v_T;

    v_hat = v_ECEF / norm(v_ECEF);
    v_hat_b = v_B / norm(v_B);
    % v_B = R_TB'*v_N;

    % Calculate dynamic pressure
    q_inf = 0.5 * rho_alt * norm(v_B) * norm(v_B);

    % Calculate Drag in ECEF Inertial Frame
    % D = [
    %     q_inf * cdPolar(M)*S_r * v_hat(1);
    %     q_inf * cdPolar(M)*S_r * v_hat(2);
    %     q_inf * cdPolar(M)*S_r * v_hat(3);
    % ];
    D_B = q_inf * cdPolar(M)*S_r * -v_hat_b;

    D_NED = R_TB * D_B;
    D_ECEF = R_IT * D_NED;

    % Calculate Thrust in ECEF Inertial Frame
    % T = [
    %     T_motor * u(2) * v_hat(1);
    %     T_motor * u(2) * v_hat(2);
    %     T_motor * u(2) * v_hat(3);
    % ];

    T_B = T_motor * u(2) * [0; 0; 1];

    T_NED = R_TB * T_B;
    T_ECEF = R_IT * T_NED;

    % Calculate AOA
    AOA_rad = atan(v_B(3) / v_B(1));
    AOA = rad2deg(AOA_rad);
    % AOA_rad = atan2(v_B(3), v_B(1));
    % AOA = rad2deg(AOA_rad);

    C_L = calculateCL(M, AOA);

    % L = [
    %     q_inf * C_L * S_r * v_hat_b(1);
    %     q_inf * C_L * S_r * v_hat_b(2);
    %     q_inf * C_L * S_r * v_hat_b(3);
    % ];

    L_B = q_inf * C_L * S_r * [1; 0; 0];

    L_NED = R_TB * L_B;
    L_ECEF = R_IT * L_NED;

    % Calculate Acceleration in ECEF Inertial Frame
    px_dot = (D_ECEF(1) + T_ECEF(1) + L_ECEF(1)) / x(14);
    py_dot = (D_ECEF(2) + T_ECEF(2) + L_ECEF(2)) / x(14);
    pz_dot = ((-g*x(14)) + D_ECEF(3) + T_ECEF(3) + L_ECEF(3)) / x(14);

    omegax_dot = -z_cp * norm(L_B) / I_x;
    omegay_dot = 0;
    omegaz_dot = (x_cp * norm(L_B)) / I_z;

    m_dot = -mFuelRate * u(2); % [kg/s]

    qw_dot = 0.5*(-x(2)*x(8)-x(3)*x(9)-x(4)*x(10));
    qx_dot = 0.5*(x(1)*x(8)+x(3)*x(10)-x(4)*x(9));
    qy_dot = 0.5*(x(1)*x(9)-x(2)*x(10)+x(4)*x(8));
    qz_dot = 0.5*(x(1)*x(10)+x(2)*x(9)-x(3)*x(8));

    x_dot = [
      qw_dot;
      qx_dot;
      qy_dot;
      qz_dot;
      x(11);
      x(12);
      x(13);
      omegax_dot;
      omegay_dot;
      omegaz_dot;
      px_dot;
      py_dot;
      pz_dot;
      m_dot;
    ];

    omega_dot = [
        omegax_dot;
        omegay_dot;
        omegaz_dot;
    ];
end