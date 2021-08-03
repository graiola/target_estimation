function [ ] = matlab2yaml( variable, variable_name, fileName, flag )
%matlab2yaml Save `variable` to yml/xml file 
% fileName: filename where the variable is stored
% flag: `a` for append, `w` for writing.
%   Detailed explanation goes here

[rows cols] = size(variable);

% Beware of Matlab's linear indexing
variable = variable';

% Write mode as default
if ( ~exist('flag','var') )
    flag = 'w'; 
end

if ( ~exist(fileName,'file') || flag == 'w' )
    % New file or write mode specified 
    file = fopen( fileName, 'w');
else
    % Append mode
    file = fopen( fileName, 'a');
end

% Write variable header
fprintf( file, '%s: [',variable_name);

% Write variable data
for i=1:rows*cols
    fprintf( file, '%.20f', variable(i));
    if (i == rows*cols), break, end
    fprintf( file, ', ');
    %if mod(i+1,4) == 0
     %   fprintf( file, '\n ');
   % end
end

fprintf( file, ']\n');

fclose(file);
end