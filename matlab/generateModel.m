function [ ] = generateModel( type, frequency, sigma_ddot, sigma_m, sigma_p, accelerations, varargin)

f = frequency; % [Hz]
dt = 1.0/f; % [s]
n = length(sigma_p);
m = length(sigma_m);
v0 = []; a0 = [];

if nargin <= 6
    v0 = zeros(1,6);
    a0 = zeros(1,6);
elseif nargin == 7
    v0 = varargin{1};
    a0 = zeros(1,6);
elseif nargin == 8
    v0 = varargin{1};
    a0 = varargin{2};
end % if
%% Compute the Q matrix (process noise) starting from the estimated std of the acceleration white noise
Sigma_a = diag(sigma_ddot);

p = 0.5 * dt^2;
v = dt;
a = 1;

%% Transformation matrix to map acceleration noises into position, velocity and accelerations
dim = length(sigma_ddot);
if accelerations
  Gamma = [p .* eye(dim); v .* eye(dim); a .* eye(dim)];
else
  Gamma = [p .* eye(dim); v .* eye(dim)];
end

Q = Gamma * Sigma_a.^2 * Gamma'; %Note the sigma square

if size(Q) ~= [n,n]
    fprintf("Wrong dimensions for Q while generating %s\n",type);
    return;
end

%% Compute the R matrix (measurement noise) starting from the estimated std of the measurement white noise
Sigma_m = diag(sigma_m);

R = Sigma_m.^2;

if size(R) ~= [m,m]
    fprintf("Wrong dimensions for R while generating %s\n",type);
    return;
end

%% Compute the P matrix starting from the expected error
P = diag(sigma_p);

if size(P) ~= [n,n]
    fprintf("Wrong dimensions for P while generating %s\n",type);
    return;
end

%% Save to yaml file
model2yaml(f,type,Q,R,P,v0,a0)

end
