dt = 0.02; % Estimated dt
type = "projectile"

%% Compute the Q matrix (process noise) starting from the estimated std of the acceleration white noise
sigma_ddot_x        = 0.001;  % [m/s^2]
sigma_ddot_y        = 0.001;  % [m/s^2]
sigma_ddot_z        = 0.001;  % [m/s^2]

sigma_a = [sigma_ddot_x
           sigma_ddot_y
           sigma_ddot_z];

Sigma_a = diag(sigma_a);           
           
p = 0.5 * dt^2;
v = dt;
a = 1;

Gamma = [p .* eye(3); v .* eye(3); a .* eye(3)];

Q = Gamma * Sigma_a.^2 * Gamma'; %Note the sigma square

%% Compute the R matrix (measurement noise) starting from the estimated std of the measurement white noise
sigma_m_x = 0.01; % [m]
sigma_m_y = 0.01; % [m]
sigma_m_z = 0.01; % [m]

sigma_m = [sigma_m_x
           sigma_m_y
           sigma_m_z];

Sigma_m = diag(sigma_m);

R = Sigma_m.^2;

%% Compute the P matrix starting from the expected error
P = eye(9);

%% Save to yaml file 
file_name = ["kf_" type "_params_" sprintf('%1.f',floor(1/dt)) "hz.yaml"]  
file = fopen( file_name, 'w');
fprintf(file, 'type: %s\n',type);
fclose(file);
matlab2yaml(Q,'Q',file_name,'a');
matlab2yaml(R,'R',file_name,'a');
matlab2yaml(P,'P',file_name,'a');
