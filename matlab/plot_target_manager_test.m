clear all
close all

models = {"1","2","3","4"};
position_labels = {"x","y","z"};

for j=1:length(models)
  
  time       = load(["/tmp/time_",      models{j}]);
  meas_pose  = load(["/tmp/meas_pose_", models{j}]);
  est_pose   = load(["/tmp/est_pose_",  models{j}]);
  real_pose  = load(["/tmp/real_pose_", models{j}]);
  est_twist  = load(["/tmp/est_twist_", models{j}]);
  
  figure(j)
  for i=1:3
    subplot(3,1,i)
    hold on
    plot(time,meas_pose(:,i),'y')
    hold on
    plot(time,est_pose(:,i),'b')
    hold on
    plot(time,real_pose(:,i),'r')
    xlabel('time [s]')
    ylabel(position_labels{i})
  end
  %hold off 
end
