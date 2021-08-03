clear all
close all
clc

dt = 0.1;
T = 30.0;
t = 0.0;
n = T/dt + 1;
t_int = zeros(n,1);
R = zeros(n,3,3);
time = zeros(n,1);
topt = zeros(n,1);
theta_opt = zeros(n,1);

omegas = [1.0 3.0 0.0]'; %rad/s

period = 2*pi/(norm(omegas)) % This is a good approx

S = [0 -omegas(1) -omegas(2) -omegas(3); omegas(1) 0 -omegas(3) omegas(2); omegas(2) omegas(3) 0 -omegas(1); omegas(3) -omegas(2) omegas(1) 0];

q0 = [0, 0.9999997, 0, 0.0007963 ];
q0 = quatnormalize(q0);
q = q0;

qd = [1.0 0.0 0.0 0.0];
qd = quatnormalize(qd);

figure(1)
for i=1:n
    
     time(i) = t;

     %quatmultiply(qd,quatconj(q)) % This is equivalent to do the following
     qe_v = q(1)*qd(2:4) -qd(1)*q(2:4) - cross(q(2:4),qd(2:4));
     qe_s = q(1)*qd(1) + q(2:4)*qd(2:4)';
     qe = [qe_s qe_v];
     qe = quatnormalize(qe);
     
     
     qe_theta(i) = 2*acos(qe_s);
     qe_s_out(i) = qe_s;
     q_out(i,:) = q;
     qd_out(i,:) = qd;

     % These are some functions I tried
     %fun = @(topt)2*acos(norm((qd * (eye(4) + 0.5*S*topt) * q'))^2);
     %fun = @(topt)2*acos((1 - (qd * (cos(norm(omegas)*topt/2) * eye(4) + 2/norm(omegas) * sin(norm(omegas)*topt/2) * 0.5 * S) * q'))/2);
     
     % This is working:
%      fun = @(topt)2*acos(((qd * (cos(norm(omegas)*topt/2) * eye(4) + 2/norm(omegas) * sin(norm(omegas)*topt/2) * 0.5 * S) * q')));
%      [topt_curr, theta_opt_curr] = fminsearch(fun,0.0);
     
     % Gradient descent
     J = @(x)(qd * (cos(norm(omegas)*x/2) * S - norm(omegas) * sin(norm(omegas)*x/2) * eye(4)) * q');
     [topt_curr, J_opt] = gradientDescent(J,0.0,0.05,200);
     
    if sign(topt_curr) > 0
        topt_curr = topt_curr;
    else
        topt_curr = period + topt_curr;
    end
    
     topt(i) = topt_curr;
     %theta_opt(i) = theta_opt_curr;

    axang = quat2axang(q);  
    e = axang(1:3);

    t = t + dt;
     
    R = quat2rotm(q);
    Rd = quat2rotm(qd);
    R0 = quat2rotm(q0);
    
    trplot(R)
    hold on
    trplot(Rd,'color','r')
    hold on
    trplot(R0,'color','g')
    hold on
    quiver3(0,0,0,e(1),e(2),e(3))
    hold off
       
    topt_curr
    qe_theta(i)
    
    pause
    
    
     %q = (eye(4) + 0.5*S*dt) * q'; % Linear approx
     q = (cos(norm(omegas)*dt/2) * eye(4) + 2/norm(omegas) * sin(norm(omegas)*dt/2) * 0.5 * S) * q';
     q = quatnormalize(q');
    
end

figure(2)
plot(time,topt)
hold on
plot(time,qe_theta,'r--')

figure(3)
plot(time,theta_opt)