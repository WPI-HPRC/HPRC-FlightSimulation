function [sf_b, w_b] = addSensorNoise(sf_b_true, w_b_true, IMUErrors, dt)

    rng(1);
    %% Add Accelerometer Biases
    accelBias = IMUErrors(1,:)'; % First Index is bias
    accelWithBias = sf_b_true + accelBias;

    %% Add Noise
    accelNoise = IMUErrors(2,:)'; % RMS Noise
    noise = accelNoise * randn(1)';
    accelWithNoise = accelWithBias + noise;

    %% Add Scale Factor
    scaleFactors = IMUErrors(3,:)';
    corruptedAccel = accelWithNoise .* scaleFactors;

    %% Add Gyro Biases
    gyroBias = IMUErrors(4,:)'; % First Index is bias
    gyroWithBias = w_b_true + gyroBias;

    %% Add Noise
    gyroNoise = IMUErrors(5,:)'; % RMS Noise
    noise = gyroNoise * randn(1)';
    gyroWithNoise = gyroWithBias + noise;

    %% Add Scale Factor
    scaleFactors = IMUErrors(6,:)';
    corruptedGyro = gyroWithNoise .* scaleFactors;


    sf_b = corruptedAccel;
    w_b = corruptedGyro;
    
end