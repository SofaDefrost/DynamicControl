%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                         Matlab toolbox for SOFA                         %
%                                                                         %
%				                                                          % 
% This plugin is distributed under the GNU LGPL (Lesser General           %
% Public License) license with the same conditions than SOFA.             %
%                                                                         %
% Contributors: Defrost team  (INRIA, University of Lille, CNRS,          %
%               Ecole Centrale de Lille)                                  %
%                                                                         %
% Contact information: https://project.inria.fr/softrobot/contact/        %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function runSofaFromMatlab(modelName,env_var,simuToLaunch,showSofaGUI)
%RUNSOFAFROMMATLAB runs a Sofa simulation from Matlab
%   Call:
%   sofa.runSofaFromMatlab(modelName,SofaFolder,currentDirectory{,typeOfSimulation});
%   Input arguments:
%   modelName : name of studied soft robot
%   SofaFolder : path to sofa scenes directory
%   currentDirectory : path to current directory (SoftRobot directory)
%   typeOfSimulation : [optional] type of simulation to run :
%           'modelling', 'closedLoop', ...

if nargin < 2
    error('Not enough input arguments.');
end

% by default, launch Sofa in batch mode
if nargin < 4
    showSofaGUI=0;
end

currentDirectory=pwd;

%% Go to directory of studied robot
pathToDir=fullfile(env_var.scenes,modelName);
cd(pathToDir);


%% Launch SOFA
user_platform = computer;
switch user_platform(1:3)
    case 'GLN'  
        pathToLib= strcat('LD_LIBRARY_PATH=',env_var.libQT);
    case 'PCW'  % if app is used on windows
        % THIS APP HAS NEVER BEEN TESTED ON WINDOWS, USE AT YOUR OWN RISKS
        pathToLib = [];
    case 'MAC'  % if app is used on macOS
        % THIS APP HAS NEVER BEEN TESTED ON MACOS, USE AT YOUR OWN RISKS
        pathToLib = [];
    otherwise
        error('This platform is not recognized.');
end

% default configuration = batch mode
% -c no = argument for runSofa to not use colors on stdout and stderr
if showSofaGUI
    sofaCommand='runSofa -c no';
else
    sofaCommand='runSofa -c no -g batch';
end

if nargin > 2
    sofaCall=char(strcat(sofaCommand,{' '},modelName,'.pyscn',{' '},'--argv',{' '},simuToLaunch));
else
    sofaCall=char(strcat(sofaCommand,{' '},modelName,'.pyscn'));
end

disp('---------------------------------');
disp('Sofa called using following call:');
disp(sofaCall)
disp('---------------------------------');

system(char(strcat(pathToLib,{' '},sofaCall)));

%% Get back to initial directory
cd(currentDirectory);

end

