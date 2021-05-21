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

function [ f, DataCL, Sys_star] = computeController( method,SysRed_c,SysRed_d,modelName,DataRed,SysMatrices,SimulationData,SysE_d, Ke, SysLarge_d)
%COMPUTECONTROLLER Computes feedback controller
% Input arguments:
%   - method : Different algorithms and control methodologies can be used to
%               compute this controller. The choice is made with the 'method'
%               argument. The controller can either be based on:
%               - reduced system only
%                   method = 'reducedOrderControl'
%                   In this case, CONTROLLER.COMPUTEREDUCEDCONTROLLER is
%                   called and the controller is computed based on the 
%                   reduced order system only, without considering the 
%                   large-scale system.
%               - both reduced and large-scale systems. 
%                   method = 'reducedWithLargeScaleProof'
%                   In this case, CONTROLLER.COMPUTEREDLARGECONTROLLER is
%                   called and the controller is still a reduced order 
%                   controller but properties are guaranteed for the 
%                   large-scale controller.
%               - large-scale system only
%                   For now, no algorithm are implemented for this option.
%   - SysRed_c : Low order continuous time system
%   - SysRed_d : Low order discrete time system
%   - modelName : name of robot studied. Some functions uses this data to
%       tune the controller.
%   - DataRed : [optional] structure containing the model reduction data, 
%       especially the projectors.
%   - SysMatrices : [optional] structure containing the mass, stiffness, 
%       damping and input matrices.
%   - SimulationData : [optional] structure containing the data used in sofa
%       simulation, such as time step.
%   - SysE_d :  [optional] state-space object, dynamical system modeling the 
%       extending observer system (only used for trajectory tracking)
%   - obs : [optional] observer matrix (only used for trajectory tracking)
%
% Example
%   [F, DATACL] = COMPUTECONTROLLER('polePlacement',SysRed_d) computes a
%   state-feedback matrix F such that the eigenvalues of SYSRED_D.A-SYSRED_D.B*F
%   are at specific locations.
%   DATACL contains the method used to compute F, i.e. DATACL.METHOD='POLEPLACEMENT'
%   and the closed-loop poles, i.e. DATACL.EIGCL=EIG(SYSRED_D.A-SYSRED_D.B*F)
%
% See also computeReducedController, computeRedLargeController,
%  VALIDATECONTROLLER, SoftRobot_class.controller

%% check if input arguments are valid
if nargin< 3
    error('Not enough input arguments.');
elseif nargin <4
    modelName=[];
end

helper.checkInputArguments({method,SysRed_c,SysRed_d},...
                           {@isstruct, @isobject, @isobject});

if isequal(method.general,'reducedOrderControl') && ~isequal(method.specific,'traj_track')
    SysE_d=[];
    Ke=[];
end

%% Main functions
Sys_star=[];    % Sys_star is computed only if method = trajectory tracking

switch method.general
    case 'nonLinear'
        f = nonLinear.testNL;
        DataCL.method=method;
        DataCL.launchSofa=1;
    case 'largeScaleControl'
        controller.computeLargeController( method,SysLarge_d,SimulationData,modelName);
        
    case 'reducedOrderControl'
        [ f, DataCL, Sys_star ] = controller.computeReducedController(method,SysRed_c,SysRed_d,modelName,SysE_d, Ke);

    case 'reducedWithLargeScaleProof'
        % reduced controller with large scale proof requires the
        % computation of the neglected state. If reduction method is POD or BT,
        % this is already done in the reduction step. Else, the following
        % function computes the projectors to compute the neglected state
        if isequal(DataRed.method,'ITIA')
            % add projectors Vbar and Wbar to DataRed
            error('[%s] %s', mfilename,'Work in progress');
            % DataRed=modelReduction.computeProjectors(DataRed,SysLarge_d,SysMatrices);
        end
        [ f, DataCL ] = controller.computeRedLargeController(SysRed_c,DataRed,SysMatrices,SimulationData,modelName);
        
    otherwise
        f=-1;
        DataCL.launchSofa=0;
        DataCL.method='Not implemented';
end


%% check results
% f should a struct with different fields:
% mandatory : a matrix L resprenting the feedback matrix
% optional, depending on the control algorithm
% matrix L_i for the integral part, atrix L_star for the reference model
% results are good if:
% control algorithms returned a struct f containging at least L OR if it
% returned an error message (DataCL.launchSOFA=0)
assert((isstruct(f) && ismatrix(f.L)) || DataCL.launchSofa==0,'Controller matrix is not a matrix.');
end