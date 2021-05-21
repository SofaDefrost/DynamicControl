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

function env_var=startup()
%STARTUP set the environment variables.
% it defines the path to the sofa scenes and data and adds to matlab path
%the required directory.
%   Call:
%   env_var=startup();
%
%   env_path : structure containing SofaFolder.data, the path to the
%   directory where sofa text files are stored and SofaFolder.scene, the
%   path to the sofa scenes.
%
%   These paths are added to matlab path to read and write data from and to
%   sofa.
%
%   The directory savedModels is also added to matlab path to save time, it
%   contains data from previous projects.
%
% Usage :
%   - directory containg sofa scenes and sofa simulation data are set by
%   default, you do not have to change them.
%   - [optional] update following paths :
%       path to model order reduction plugin (if required)
%       path to libQT (if required)
%       path to libqxcb (if required)
%
% See also SOFTROBOT_APP, SOFTROBOT_CLASS

clc;
s=pwd;

%% path to Sofa scenes files
% path to sofa files is set by default
% change it at your own risks
env_var.scenes='SofaScenes/Projects/';

% path to Sofa data files
% path to sofa simulation data is set when choosing the name of the studied robot via the main app
% change it at your own risks
% default is SofaFolder.data='~/SofaScenes/Project/RobotName/data'; 

%% link to sofa plugins
% optional 
% mandatory to use POD reduction
env_var.pathToMOR = "/Path/To/Sofa/Plugins/ModelOrderReduction/";

%% link to required libraries
% optional
% the following may be useful if matlab does not find required libraries
env_var.libQT = '/Path/To/QT/lib'; % folder containing libQt5Gui.so.5, libQt5Sql.so.5 ...
env_var.libQXCB = '/Path/To/QT/platforms/libqxcb.so';

%% add Sofa files to matlab path
addpath(genpath(strcat(env_var.scenes))); 
addpath(genpath(strcat(s,'/savedModels/')));         

end