clear variables; close all; clc;

%% Rocket Drag Polar
% Using the least squares estimation method, the drag curve outputted by OpenRocket and RasAero can be used to estimate C_D at specified mach numbers
machNumber = [0.01; 0.1; 0.2; 0.3; 0.4; 0.5; 0.6; 0.7; 0.8; 0.9; 1; 1.1];
cDVal = [0.444; 0.392; 0.374; 0.364; 0.358; 0.353; 0.352; 0.356; 0.360; 0.363; 0.497; 0.575];

X = [machNumber.^5 machNumber.^4 machNumber.^3, machNumber.^2, machNumber, ones(size(machNumber))];

% Use least squares estimate to solve for C
C = X \ cDVal;
cdPolar = @(x) C(1)*x^5 + C(2)*x^4 + C(3)*x^3 + C(4)*x^2 + C(5)*x + C(6);

figure;
scatter(machNumber, cDVal, 'o', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
hold on;
fplot(cdPolar, [min(machNumber), max(machNumber)], 'r');
title('Vehicle C_D');
xlabel('Mach Number');
ylabel('Coefficient of Drag');
legend('Data Points', 'Function');
grid on;

%% RK4 Configuration
dt = 0.01;
t_0 = 0;
t = t_0;
t_f = 50;
timeSlice = t_0:dt:t_f;
numTimePts = t_f / dt+1;
tRecord = zeros(1, numTimePts);
tRecord(1,1) = t;

v_B = [0;0;0.01]; % [m/s]
q_0 = [0.0189, -0.9493, 0.3084, -0.0581];
% q_0 = [0.10,-0.92,0.38,-0.03];

R_TB = quat2rotm(q_0);

rotm2eul(R_TB);

v_NED = R_TB' * v_B;

m_fuel = 4.776; % [kg] Solid rocket fuel mass
m_empty = 22.72 + 3.342; % [kg] Empty Rocket Mass
m_full = m_empty + m_fuel;

% qw, qi, qj, qk, px, py, pz, wx, wy, wz, vx, vy, vz, m
x_t = [q_0(1); q_0(2); q_0(3); q_0(4); 0; 0; -1400; 0; 0; 0; v_NED(1); v_NED(2); v_NED(3); m_full];


xRecord = zeros(length(x_t), numTimePts);
xRecord(:, 1) = x_t;

%% Sensor Readings
% Acx, Acy, Acz, Gyx, Gyy, Gyz, Magx, Magy, Magz
a_t = [0; 0; 0; 0; 0; 0; 0; 0; 0];
sRecord = zeros(length(a_t), numTimePts-1);

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

    k1 = dt*RocketDynamicModel(x_t, u_t, cdPolar);
    k2 = dt*RocketDynamicModel(x_t+(1/2)*k1, u_t, cdPolar);
    k3 = dt*RocketDynamicModel(x_t+(1/2)*k2, u_t, cdPolar);
    k4 = dt*RocketDynamicModel(x_t+k3, u_t, cdPolar);

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

    sRecord(:,colNum) = [aR; 0;0;0; 0;0;0];
    
end

%% Plotting Results
% figure();
% plot(tRecord, xRecord(5,:)); % X-Position vs Time
% title("X Vs. Time");
% xlabel("Time (s)");
% ylabel("X-Pos (m)");
% grid on;

% figure();
% plot(tRecord, xRecord(6,:)); % Y-Position vs Time
% title("Y Vs. Time");
% xlabel("Time (s)");
% ylabel("Y-Pos (m)");
% grid on;

% figure();
% plot(tRecord, xRecord(7,:)); % Z-Position vs Time
% title("Z Vs. Time");
% xlabel("Time (s)");
% ylabel("Z-Pos (m)");
% grid on;

% figure();
% plot(tRecord, xRecord(11,:)); % X-Velocity vs Time
% title("X-Velocity Vs. Time");
% xlabel("Time (s)");
% ylabel("X-Velocity (m/s)");
% grid on;

% figure();
% plot(tRecord, xRecord(12,:)); % Y-Velocity vs Time
% title("Y-Velocity Vs. Time");
% xlabel("Time (s)");
% ylabel("Y-Velocity (m/s)");
% grid on;

% figure();
% plot(tRecord, xRecord(13,:)); % Z-Velocity vs Time
% title("Z-Velocity Vs. Time");
% xlabel("Time (s)");
% ylabel("Z-Velocity (m/s)");
% grid on;

% figure();
% plot3(xRecord(5,:), xRecord(6,:), xRecord(7,:));
% title("Rocket Path");
% xlabel("Px (m)");
% ylabel("Py (m)");
% zlabel("Pz (m)");
% grid on;

% euler_angles = rad2deg(quat2eul(xRecord(1:4,:)', 'XYZ'));
% % Plot Euler angles against time
% figure();
% plot(tRecord, euler_angles(:,1)); % Roll angle vs Time
% title("Roll Angle Vs. Time");
% xlabel("Time (s)");
% ylabel("Roll Angle (rad)");
% grid on;

% figure();
% plot(tRecord, euler_angles(:,2)); % Pitch angle vs Time
% title("Pitch Angle Vs. Time");
% xlabel("Time (s)");
% ylabel("Pitch Angle (rad)");
% grid on;

% figure();
% plot(tRecord, euler_angles(:,3)); % Yaw angle vs Time
% title("Yaw Angle Vs. Time");
% xlabel("Time (s)");
% ylabel("Yaw Angle (rad)");
% grid on;

% figure();
% plot(tRecord, sRecord(3,:));
% title("Sensor Readings - Az");
% xlabel("Time (s)");
% ylabel("Acceleration Z (m/s^2)")
% grid on;