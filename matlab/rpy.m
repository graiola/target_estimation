f = 5.0; % [Hz]
dt = 1.0/f; % Estimated dt
type = 'rpy';

%% Compute the Q matrix (process noise) starting from the estimated std of the acceleration white noise
sigma_ddot_x        = 1e-7;  % [m/s^2]
sigma_ddot_y        = 1e-7;  % [m/s^2]
sigma_ddot_z        = 1e-7;  % [m/s^2]
sigma_dot_omega_x   = 1e-8; % [rad/s^2]
sigma_dot_omega_y   = 1e-8; % [rad/s^2]
sigma_dot_omega_z   = 1e-8; % [rad/s^2]

sigma_a = [sigma_ddot_x
    sigma_ddot_y
    sigma_ddot_z
    sigma_dot_omega_x
    sigma_dot_omega_y
    sigma_dot_omega_z];

Sigma_a = diag(sigma_a);

p = 0.5 * dt^2;
v = dt;

Gamma = [p .* eye(6); v .* eye(6)];

Q = Gamma * Sigma_a.^2 * Gamma'; %Note the sigma square

%% Compute the R matrix (measurement noise) starting from the estimated std of the measurement white noise
sigma_m_x = 1e-5; % [m]
sigma_m_y = 1e-5; % [m]
sigma_m_z = 1e-5; % [m]
sigma_m_roll  = 1e-6;   % [rad]
sigma_m_pitch = 1e-6;   % [rad]
sigma_m_yaw   = 1e-6;   % [rad]

sigma_m = [sigma_m_x
    sigma_m_y
    sigma_m_z
    sigma_m_roll
    sigma_m_pitch
    sigma_m_yaw];

Sigma_m = diag(sigma_m);

R = Sigma_m.^2;

%% Compute the P matrix starting from the expected error
simga_p_pos = 1e-1;
simga_p_rpy = 1e-2;
P = [eye(6), eye(6).*simga_p_pos;
    zeros(6), eye(6).*simga_p_rpy] ;

%% Save to yaml file 
model2yaml(f,type,Q,R,P)
