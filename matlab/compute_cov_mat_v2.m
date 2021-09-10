f = 5*1e0; % [Hz]
dt = 1/f; % Estimated dt
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
CLK=clock;
YR=num2str(CLK(1),'%04d');
MTH=num2str(CLK(2),'%02d');
DAY=num2str(CLK(3),'%02d');
HOUR=num2str(CLK(4),'%02d');
MIN=num2str(CLK(5),'%02d');
SEC=num2str(round(CLK(6)),'%02d');
date = [YR,'-',MTH,'-',DAY];
date_time = [YR,'-',MTH,'-',DAY,'_',HOUR,'.',MIN];

% % select destiation folder
% dir_path = [];
% dir_path = uigetdir('Select destination folder for the yaml file...');
% 
% Save_folder_path = dir_path;

Save_folder_path = ['..',filesep,'config'];
if(exist(Save_folder_path)~=7) % create new folder if it does not exist
    mkdir(Save_folder_path);
end

file_name = [];
file_name = [Save_folder_path,filesep,'kf_',type,'_params_',num2str(f),'hz_','version_', date,'.yaml'];
file = fopen( file_name, 'w');
fprintf(file, 'type: %s\n',type);
fclose(file);
matlab2yaml(Q,'Q',file_name,'a');
matlab2yaml(R,'R',file_name,'a');
matlab2yaml(P,'P',file_name,'a');