function kins = HPRC_RocketKinematics()

kins.x_cp = 2.14 - 2.53; % [m] Longitudinal center of pressure distance
kins.I_x = 0.054;  % [kg/m^2]
kins.I_y = 16.004; % [kg/m^2]
kins.I_z = 16.004; % [kg/m^2]
kins.I = diag([kins.I_x, kins.I_y, kins.I_z]);
kins.diameter = 0.131; % [m] Diameter
kins.S = pi * (kins.diameter^2 / 4);
kins.len = 3.36; % [m] Rocket Length

% Mass Properties
kins.m_0 = 17.791; % [kg] Dry Mass

% Airbrakes Properties
ab.cd = 1.2; % [] Coefficient of Drag **TO GET FROM CFD**
ab.S_max = 1.1619332e-3; % [m^2] Maximum AB surface area
kins.ab = ab;

end