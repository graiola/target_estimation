clear all
close all

prefix = 'target_node_';
data = {'twist','time','sigma','pose','meas_pose','est_pose','est_twist','err_twist','err_pose'};

for i=1:length(data)
    full_data_name = [prefix data{i}];
    load(full_data_name);
    eval([data{i} '=' full_data_name ';']);
 end

 
 figure(1)
 for i=1:3
   subplot(3,1,i)
   plot(time,meas_pose(:,i),'r')
   hold on
   plot(time,est_pose(:,i))
   hold on
   plot(time,pose(:,i),'b')
 end
 
 figure(2)
 for i=1:3
   sigma_up = err_pose(:,i) + 3*sqrt(sigma(:,i));
   sigma_down = err_pose(:,i) - 3*sqrt(sigma(:,i));
   subplot(3,1,i)
   plot(time,err_pose(:,i))
   hold on
   plot(time,sigma_up,'r')
   hold on
   plot(time,sigma_down,'r')
 end
 
  figure(3)
 for i=1:3
   subplot(3,1,i)
   plot(time,est_twist(:,i))
 end