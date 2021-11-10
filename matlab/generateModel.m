function [ ] = generateModel( type, frequency, sigma_ddot_lin, sigma_ddot_ang, sigma_m, sigma_p, accelerations = false)

f = frequency; % [Hz]
dt = 1.0/f; % Estimated dt
n = length(sigma_ddot_lin);
m = length(sigma_m);

%% Compute the Q matrix (process noise) starting from the estimated std of the acceleration white noise
sigma_a = [sigma_ddot_lin
           sigma_ddot_ang];

Sigma_a = diag(sigma_a);

p = 0.5 * dt^2;
v = dt;
a = 1;

if accelerations:
  Gamma = [p .* eye(n); v .* eye(n); a .* eye(n)];
else
  Gamma = [p .* eye(n); v .* eye(n)];
end

Q = Gamma * Sigma_a.^2 * Gamma'; %Note the sigma square

%% Compute the R matrix (measurement noise) starting from the estimated std of the measurement white noise
Sigma_m = diag(sigma_m);

R = Sigma_m.^2;

%% Compute the P matrix starting from the expected error
P = diag(sigma_p);

%% Save to yaml file
model2yaml(f,type,Q,R,P)

end
