function x_dot = RocketDynamicModel(x,u,cdPolar)
    %% Earth Constants
    g = 9.81; % [m/s/s] Gravitational Acceleration on Earth
    R = 287; % [J/kgK]

    z_cp = 0.285 - 0.233;
    x_cp = 0;
    y_cp = 0;

    atmo = AtmosphericModel(-x(7));

    rho_alt = atmo.getDensity();

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

    R_TB = quat2rotm([x(1),x(2),x(3),x(4)]);

    v_N = [x(11); x(12); x(13)];

    M = norm(v_N) / a;

    v_hat = v_N / norm(v_N);

    v_B = R_TB*v_N;

    D = [
        0.5*rho_alt*cdPolar(M)*S_r*norm(v_B)*norm(v_B) * v_hat(1);
        0.5*rho_alt*cdPolar(M)*S_r*norm(v_B)*norm(v_B) * v_hat(2);
        0.5*rho_alt*cdPolar(M)*S_r*norm(v_B)*norm(v_B) * v_hat(3)
    ];

    T = [
        T_motor * u(2) * v_hat(1);
        T_motor * u(2) * v_hat(2);
        T_motor * u(2) * v_hat(3);
    ];

    px_dot = (-D(1) + T(1)) / x(14);
    py_dot = (-D(2) + T(2)) / x(14);
    pz_dot = ((g*x(14)) - D(3) + T(3)) / x(14);

    D_B = R_TB' * D;

    omegax_dot = (D_B(1)*z_cp) / I_x * 0.1542;
    omegay_dot = (D_B(2)*x_cp) / I_y * 0.1542;
    omegaz_dot = (D_B(3)*y_cp) / I_z * 0.1542;

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
end