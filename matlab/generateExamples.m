%% Angular rates system
% Params
type = 'angular_rates';
frequency = 5.0; % [Hz]
accelerations = true;
% Noise on the acceleration
sigma_ddot = [1e-7 1e-7 1e-7 1e-8 1e-8 1e-8]; %[m/s^2 m/s^2 m/s^2 rad/s^2 rad/s^2 rad/s^2]
% Noise on the measurements
sigma_m = [1e-5 1e-5 1e-5 1e-6 1e-6 1e-6]; %[m m m rad rad rad]
% Expected initial error
I = ones(1,3);
sigma_p = [0.1*I 0.01*I 0.01*I 0.01*I 0.01*I 0.01*I]; %[m rad m/s rad/s m/s^2 rad/s^2]
generateModel(type, frequency, sigma_ddot, sigma_m, sigma_p, accelerations)

%% Angular velocities
% Params
type = 'angular_velocities';
frequency = 5.0; % [Hz]
accelerations = false; % Note: this system does not support accelerations
% Noise on the acceleration
sigma_ddot = [1e-7 1e-7 1e-7 1e-8 1e-8 1e-8]; %[m/s^2 m/s^2 m/s^2 rad/s^2 rad/s^2 rad/s^2]
% Noise on the measurements
sigma_m = [1e-5 1e-5 1e-5 1e-6 1e-6 1e-6]; %[m m m rad rad rad]
% Expected initial error
I = ones(1,3);
sigma_p = [0.1*I 0.01*I 0.01*I 0.01*I]; %[m rad m/s rad/s]
generateModel(type, frequency, sigma_ddot, sigma_m, sigma_p, accelerations)

%% Projectile
% Params
type = 'projectile';
frequency = 5.0; % [Hz]
accelerations = true; % Note: this system needs accelerations
% Noise on the acceleration
sigma_ddot = [1e-7 1e-7 1e-7]; %[m/s^2 m/s^2 m/s^2]
% Noise on the measurements
sigma_m = [1e-5 1e-5 1e-5]; %[m m m]
% Expected initial error
sigma_p = [0.1 0.1 0.1 0.01 0.01 0.01 0.001 0.001 0.001]; %[m m m m/s m/s m/s m/s^2 m/s^2 m/s^2]
generateModel(type, frequency, sigma_ddot, sigma_m, sigma_p, accelerations)

%% Uniformly accelerated
% Params
type = 'uniformly_accelerated';
frequency = 5.0; % [Hz]
accelerations = true; % Note: this system needs accelerations
% Noise on the acceleration
sigma_ddot = [1e-7 1e-7 1e-7]; %[m/s^2 m/s^2 m/s^2]
% Noise on the measurements
sigma_m = [1e-5 1e-5 1e-5]; %[m m m]
% Expected initial error
sigma_p = [0.1 0.1 0.1 0.01 0.01 0.01 0.001 0.001 0.001]; %[m m m m/s m/s m/s m/s^2 m/s^2 m/s^2]
generateModel(type, frequency, sigma_ddot, sigma_m, sigma_p, accelerations)
