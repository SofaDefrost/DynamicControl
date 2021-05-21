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

function launchPODinSOFA(modelName, env_var, SimulationData)
%LAUNCHPODINSOFA uses the Model order Reduction plugin to save snapshots
%of the robot. These snapshots can then be read using readSofaSnapshots.
% To see the "shaking" simulation, go to Sofa/SimulationData directory for
% the studied robot. And launch "debug_scene" in reduction/debug
%
% Launch sofa simulation that saves snapshots
% Here is the python scripts used to save snapshots using the Model Reduction Plugin
% import os
% import sys
% sys.path.append("./SofaScenes/PythonScripts/")
% from controller import reduction
% reduction.saveSnapshots(inputArgs)
% where inputArgs are:
% absPathToSceneDir : 	absolute path to directory where the scene to reduce is stored.
% modelName 		: 	name of the robot studied 	(e.g. name of the scene without '.pyscn' extension)
% nbActuators		: 	number of actuators to use to "shake" the robot and save snapshots
% maxForceToApply   : 	maximum force to apply to each actuator
%						the force applied to each actuators goes from 0 to maxForceToApply
%						with an increment computed such that there are 50 increments
%                       between force = 0 and force = maxForceToApply

warning('[%s] %s',mfilename,'This file supposes that python scripts are located in /SoftRobots/SofaScenes/Projects');

nbActuators = SimulationData.nbActuators;
maxForceToApply = SimulationData.maxForceActuators;

% translation of python script into matlab command:

commandLine=sprintf(['python -c ',...
    '"import os ; import sys;', ...
    'sys.path.append(''./SofaScenes/PythonScripts/'');',...
    'from controller import reduction;', ...
    'reduction.saveSnapshots(''%s'',''%s'',%d,%d, ''%s'', ''%s'')"'],...
    strcat(pwd,'/',env_var.scenes,modelName,'/'),...
    modelName,...
    nbActuators,...
    maxForceToApply,...
    env_var.data,...    
    env_var.pathToMOR);

pathToLib= strcat('LD_LIBRARY_PATH=',env_var.libQT); % handle problem with matlab not findind QT lib

tstart= tic;
[result,status]=system(char(strcat(pathToLib,{' '},commandLine)));
fprintf('\nTime to save snapshots for model with %d nodes and %d actuators: %4.0d minutes.\n',...
    SimulationData.numNodes, nbActuators, toc(tstart)/60);

if result ~= 0
    error(status)
end

end