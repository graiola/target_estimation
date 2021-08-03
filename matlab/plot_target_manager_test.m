clear all
close all

prefixs = {'rpy_','rpy_ext_'}
data = {'meas_pose','est_pose','est_twist'};

for j=1:length(prefixs)
  for i=1:length(data)
      full_data_name = [prefixs{j} data{i}];
      load(full_data_name);
      eval([data{i} '=' full_data_name ';']);
   end
 

   figure(1)
   for i=1:3
     subplot(3,2,i)
     plot(time,meas_pose(:,i),'r')
     hold on
     plot(time,est_pose(:,i))
     hold on
     plot(time,pose(:,i),'b')
   end
   
    figure(2)
   for i=1:3
     subplot(3,2,i)
     plot(time,est_twist(:,i))
   end
 
 end