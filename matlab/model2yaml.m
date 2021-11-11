function [ ] = model2yaml( frequency, type, Q, R, P)

%% Save to yaml file
%CLK=clock;
%YR=num2str(CLK(1),'%04d');
%MTH=num2str(CLK(2),'%02d');
%DAY=num2str(CLK(3),'%02d');
%HOUR=num2str(CLK(4),'%02d');
%MIN=num2str(CLK(5),'%02d');
%SEC=num2str(round(CLK(6)),'%02d');
%date = [YR,'-',MTH,'-',DAY];
%date_time = [YR,'-',MTH,'-',DAY,'_',HOUR,'.',MIN];

% % select destiation folder
% dir_path = [];
% dir_path = uigetdir('Select destination folder for the yaml file...');
% 
% Save_folder_path = dir_path;

save_folder_path = ['..',filesep,'models'];
if(exist(save_folder_path)~=7) % create new folder if it does not exist
    mkdir(save_folder_path);
end

file_name = [];
file_name = [save_folder_path,filesep,'model_',type,'_params.yaml'];
file = fopen( file_name, 'w');
fprintf(file, 'type: %s\n',type);
fprintf(file, 'frequency: %f\n',frequency);
fclose(file);    
matlab2yaml(Q,'Q',file_name,'a');
matlab2yaml(R,'R',file_name,'a');
matlab2yaml(P,'P',file_name,'a');

end
