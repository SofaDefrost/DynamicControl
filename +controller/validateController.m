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

function validateController(SoftRobot,plotClosedLoopResults,launchSimulation)
% VALIDATECONTROLLER checks if controller design was succesfull.
%
% Examples
%       Input arguments:
%           SoftRobot : an object of type SoftRobot_class for which the
%                       controller should be tested.
%           plotClosedLoopResults: [optional] boolean, if true plot closed
%                                   loop results. Default = 1.
%           launchSimulation: [optional] boolean, if true, launch
%                               closed-loop simulation. Default = 0.
%
% See also COMPUTECONTROLLER, CONTROLVALIDATION, stateObserver.validateObserver

if nargin < 1
    error('Not enough input arguments.');
elseif nargin == 1
    plotClosedLoopResults=1;
    launchSimulation=0;
elseif nargin == 2
    launchSimulation=0;
end

if ismatrix(SoftRobot.f)
    if plotClosedLoopResults
        controller.displayControllerResults(SoftRobot.SysRed_d, SoftRobot.f, SoftRobot.DataCL)
    end
    if launchSimulation
        if isequal(SoftRobot.DataCL.method.specific,'traj_track')
%             controller.trajectoryTrackingValidation(SoftRobot.SimulationData,...
%                 SoftRobot.SysLarge_d,SoftRobot.SysRed_d,SoftRobot.DataRed,...
%                 SoftRobot.Sys_star,SoftRobot.f.L,SoftRobot.f.L_star,SoftRobot.SysE_d,1)s
         largeScale.finalClosedLoopValidation(SoftRobot.SimulationData,SoftRobot.SysLarge_d,...
              SoftRobot.SysRed_d,SoftRobot.DataRed,SoftRobot.f,SoftRobot.Sys_star,SoftRobot.obs,SoftRobot.SysE_d,0)
        else
        controller.controlValidation(SoftRobot.SimulationData,...
            SoftRobot.SysLarge_d,SoftRobot.SysRed_d,SoftRobot.DataRed,SoftRobot.f,1);
        end
    end
else
    error('Cannot test controller, feedback matrix is not computed.');
end
end