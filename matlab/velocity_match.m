close all
clear all

dt = 0.01;   %Sample time [s]
V_t = [-0.05, 0.0, 0.0]';   %Target constant velocity [m/s]
P_ee_i = [0.1, 0.0, 0.0]';  %Initial position EE [m]
P_t_i = [0.35, 0.0, 0.0]'; %Initial position target [m]
V_ee_i = [0.0, 0.0, 0.0]'; %Initial velocity EE [m/s]
d = 0.03; %Final relative distance [m]


Vv_t = V_t/norm(V_t);

D = Vv_t * inv(Vv_t' * Vv_t) * Vv_t';
Dt = eye(3) - D;

% Project on the direction of motion
Pd_t_i = Vv_t' * P_t_i;
Pd_ee_i = Vv_t' * P_ee_i;
Vd_ee_i = Vv_t' * V_ee_i;
Vd_t = Vv_t' * V_t;
%dd = Vv_t' * d;

Td = 2*(Pd_t_i - Pd_ee_i + d)./(Vd_ee_i - Vd_t)
Ad_ee_ref = ((Vd_t - Vd_ee_i))./Td

N = floor(abs(max(Td))/dt);

N_tot = N;

t_data = zeros(N_tot,1);
Pdr_data = zeros(N_tot,length(Td));
Pd_ee_data = zeros(N_tot,length(Td));
Pd_t_data = zeros(N_tot,length(Td));
Vd_ee_data = zeros(N_tot,length(Td));
Ad_ee_data = zeros(N_tot,length(Td));
Vd_t_data = zeros(N_tot,length(Td));
A_t_data = zeros(N_tot,length(Td));
Vdr_data = zeros(N_tot,length(Td));

% Init
Pd_ee = Pd_ee_i;
P_ee = P_ee_i;
Pd_t = Pd_t_i;
P_t = P_t_i;
Vd_ee = Vd_ee_i;
t = 0.0;
Ad_ee = zeros(size(Td))';

 
for i=1:N_tot
    
    Pdr = Pd_ee - Pd_t;
    
    Ad_ee = Ad_ee_ref;

    t = (i-1) * dt;
    
    Pd_t = Pd_t + Vd_t .* dt;
    Vd_ee = Vd_ee + Ad_ee .* dt;
    Pd_ee = Pd_ee + Vd_ee .* dt;
    
    V_ee = Vv_t * Vd_ee;
    P_ee = P_ee + V_ee .* dt;
    
    
    P_t = P_t + V_t .* dt;
    
    % Save
    t_data(i) = t;
    Pdr_data(i,:) = Pdr;
    Pd_ee_data(i,:) = Pd_ee;
    P_ee_data(i,:) = P_ee;
    Pd_t_data(i,:) = Pd_t;
    P_t_data(i,:) = P_t;
    Vd_ee_data(i,:) = Vd_ee;
    V_ee_data(i,:) = V_ee;
    
    Ad_ee_data(i,:) = Ad_ee; 
    Vd_t_data(i,:) = Vd_t;
    Vdr_data(i,:) = Vd_ee - Vd_t;
    
    Vr_data(i,:) = V_ee - V_t;
    Pr_data(i,:) = P_ee - P_t;
end

figure(1)
subplot(2,1,1)
plot(t_data,Pdr_data)
title('Relative position and velocity')
ylabel('pos [m]')
xlabel('time [s]')
subplot(2,1,2)
plot(t_data,Vdr_data)
ylabel('vel [m/s]')
xlabel('time [s]')
 
figure(2)
subplot(3,1,1)
plot(t_data,Pd_ee_data)
ylabel('pos [m]')
title('EE')
subplot(3,1,2)
plot(t_data,Vd_ee_data)
ylabel('vel [m/s]')
subplot(3,1,3)
plot(t_data,Ad_ee_data)
ylabel('acc [m/s^2]')
xlabel('time [s]')

figure(3)
subplot(3,1,1)
plot(t_data,Pd_t_data)
ylabel('pos [m]')
title('Target')
subplot(3,1,2)
plot(t_data,Vd_t_data)
ylabel('vel [m/s]')
subplot(3,1,3)
plot(t_data,A_t_data)
ylabel('acc [m/s^2]')
xlabel('time [s]')

figure(4)
plot(t_data,Pd_t_data)
hold on
plot(t_data,Pd_ee_data,'r')
title('Target and EE')
legend('Target','EE')
xlabel('time [s]')
ylabel('pos [m]')
