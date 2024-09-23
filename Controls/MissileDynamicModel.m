function x_dot = MissileDynamicModel(x, u, AeroModel, const, kins, inds)
%% MissileDynamicModel - Nonlinear dynamic model of missile
% Returns the discrete state derivative of a generic missile model
% Inputs:
%   X - Current state of the vehicle
     % [qw, qx, qy, qz, px, py, pz, vx, vy, vz, m]
%   U - Control Inputs to the vehicle
%   AeroModel - Aerodynamic Model of the vehicle holding aerodynamic data

    r_ecef = [x(inds.px_ecef); x(inds.py_ecef); x(inds.pz_ecef)];

    r_ned = R_TE * r_ecef;

    atmo = AtmosphericModel(-r_ned(3));

    rho_alt = atmo.getDensity();

    a = sqrt(const.gamma_air*const.R_air*atmo.getTemperature()); % [m/s] Speed of sound at state

    v_ecef = [x(inds.vx_ecef); x(inds.vy_ecef); x(inds.vz_ecef)]; % [m/s] Velocity vector in ECEF

    M = norm(v_ecef) / a; % Mach Number

    v_hat_ecef = v_ecef / norm(v_ecef); % [1] Unit Vector of Velocity in ECEF

    q_inf = 0.5 * rho_alt * norm(v_ecef)^2; % Dynamic Pressure Term

    v_hat_B = R_BE * v_hat_ecef; % [1] Unit Vector of Velocity in Body

    AoA = atan2(v_hat_B(3), v_hat_B(1)); AoA = rad2deg(AoA);

    D_B = q_inf * AeroModel.cdLookup(M, AoA) * kins.S * v_hat_B;

    D_ECEF = R_EB * D_B;

    L_B = q_inf * AeroModel.clLookup(M, AoA) * kins.S * cross(v_hat_B, [1; 0; 0]);

    L_ECEF = R_EB * L_B;

    g_T = [0; 0; -const.g_e]; % **TODO** APPLY GRAVITY MODEL

    G_ECEF = R_ET * g_T;

    %% Position Dynamics
    vx_dot = (D_ECEF(1) + L_ECEF(1) + G_ECEF(1)) / x(inds.mass);
    vy_dot = (D_ECEF(2) + L_ECEF(2) + G_ECEF(2)) / x(inds.mass);
    vz_dot = (D_ECEF(3) + L_ECEF(3) + G_ECEF(3)) / x(inds.mass);

    %% Attitude Dynamics
    omegax_dot = 0;
    omegay_dot = (-kins.x_cp * norm(L_B)) / kins.I_y;
    omegaz_dot = 0;

    x_dot = [
        0;
        0;
        0;
        0;
        x(inds.vx_ecef);
        x(inds.vy_ecef);
        x(inds.vz_ecef);
        vx_dot;
        vy_dot;
        vz_dot;
    ];

end