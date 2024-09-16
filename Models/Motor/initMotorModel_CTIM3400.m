function ModelData = initMotorModel_CTIM3400(plotThrustCurve)
%% Motor Model - CTIM3400
% Author: Daniel Pearson (djpearson@wpi.edu)
% Version: 9.16.2024
% Cesaroni M3400 OpenRocket Motor Model
% Inputs:
%   plotThrustCurve - (T/F)
% Outputs:
%   ModelData - Struct containing motor constants and thrust curve

%% Default Parameter Val
if ~exist('plotThrustCurve', 'var')
    plotThrustCurve = false;
end

%% Constants
consts.totalImpulse = 9990;      % [N-s] Total Impulse
consts.avgThrust    = 3414;      % [N] Average Thrust
consts.maxThrust    = 3898;      % [N] Max Thrust
consts.burnTime     = 2.92;      % [s] Burn Time
consts.m_i          = 8.108;     % [kg] Initial Mass
consts.m_f          = 3.342;     % [kg] Final Mass
consts.m_p          = consts.m_i - consts.m_f; % [kg] Propellant Mass
consts.diameter     = 98e-3;     % [m] Motor Diameter
consts.I_sp         = 213.74;    % [s] Specific Impulse


%% Thrust Curve
thrustCurveData = load('M3400_ThrustCurve.mat');

% % Thrust polynomial - Fourth degree polyn least squares approx **BAD**
% X = [thrustCurveData.time.^4, thrustCurveData.time.^3, thrustCurveData.time.^2, thrustCurveData.time, ones(size(thrustCurveData.time))];
% 
% C = X \ thrustCurveData.thrust;
% 
% thrustPolar = @(x) C(1)*x^4 + C(2)*x^3 + C(3)*x^2 + C(4)*x + C(5);

% Makima interpolate into anonymous function
thrustPolar = @(t) interp1(thrustCurveData.time, thrustCurveData.thrust, t, 'makima');


ModelData.thrustPolar = thrustPolar;
ModelData.consts = consts;

if(plotThrustCurve)
    figure('Name', 'Thrust Curve');
    scatter(thrustCurveData.time, thrustCurveData.thrust, 'o', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
    hold on;
    fplot(thrustPolar, [min(thrustCurveData.time), max(thrustCurveData.time)], 'r');
    hold off;
    title('CTI-M3400 Thrust Curve');
    xlabel('Thrust (N)');
    ylabel('Time (s)');
    legend('ThrustPTS', 'Function');
    grid on;
end



end