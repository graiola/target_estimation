%% Angular rates system
% Params
type = 'angular_rates';
frequency = 250.0; % [Hz]
accelerations = true;
% Noise on the acceleration
sigma_ddot = [1e-3 1e-3 1e-3 1e-5 1e-5 1e-5]; %[m/s^2 m/s^2 m/s^2 rad/s^2 rad/s^2 rad/s^2]
% Noise on the measurements
sigma_m = [0.01 0.01 0.01 0.1 0.1 0.1]; %[m m m rad rad rad]
% Expected initial error
I = ones(1,3);
sigma_p = [0.1*I 0.01*I 0.01*I 0.01*I 0.01*I 0.01*I]; %[m rad m/s rad/s m/s^2 rad/s^2]
generateModel(type, frequency, sigma_ddot, sigma_m, sigma_p, accelerations)

%% Angular velocities
% Params
type = 'angular_velocities';
frequency = 250.0; % [Hz]
accelerations = false; % Note: this system does not support accelerations
% Noise on the acceleration
sigma_ddot = [1e-3 1e-3 1e-3 1e-5 1e-5 1e-5]; %[m/s^2 m/s^2 m/s^2 rad/s^2 rad/s^2 rad/s^2]
% Noise on the measurements
sigma_m = [0.01 0.01 0.01 0.1 0.1 0.1]; %[m m m rad rad rad]
% Expected initial error
I = ones(1,3);
sigma_p = [0.1*I 0.01*I 0.01*I 0.01*I]; %[m rad m/s rad/s]
generateModel(type, frequency, sigma_ddot, sigma_m, sigma_p, accelerations)

%% Uniform acceleration
% Params
type = 'uniform_acceleration';
frequency = 250.0; % [Hz]
accelerations = true; % Note: this system needs accelerations
% Noise on the acceleration
sigma_ddot = [1e-3 1e-3 1e-3]; %[m/s^2 m/s^2 m/s^2]
% Noise on the measurements
sigma_m = [0.01 0.01 0.01]; %[m m m]
% Expected initial error
sigma_p = [0.1 0.1 0.1 0.01 0.01 0.01 0.001 0.001 0.001]; %[m m m m/s m/s m/s m/s^2 m/s^2 m/s^2]
v0 = zeros(6,1);
a0 = zeros(6,1);
a0(3) = -9.81;
generateModel(type, frequency, sigma_ddot, sigma_m, sigma_p, accelerations, v0, a0)

%% Uniform velocity
% Params
type = 'uniform_velocity';
frequency = 250.0; % [Hz]
accelerations = false; % Note: this system does not accelerations
% Noise on the acceleration
sigma_ddot = [1e-3 1e-3 1e-3]; %[m/s^2 m/s^2 m/s^2]
% Noise on the measurements
sigma_m = [0.01 0.01 0.01]; %[m m m]
% Expected initial error
sigma_p = [0.1 0.1 0.1 0.01 0.01 0.01]; %[m m m m/s m/s m/s]
generateModel(type, frequency, sigma_ddot, sigma_m, sigma_p, accelerations)
