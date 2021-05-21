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

classdef SoftRobot_class
    %SOFTROBOT_CLASS soft robot modelled with Sofa
    %   class containing all data and functions required to perform dynamic
    %   control of soft robots using Sofa.
    %
    % Example
    %   SofaFolder=startup;                             % startup returns the path to sofa scenes
    %   SoftRobot=main.SoftRobot_class;                 % creates an empty SoftRobot object
    %   SoftRobot=modelling(SoftRobot,SofaFolder);      % build a model for the robot
    %   SoftRobot=reduction(SoftRobot,SofaFolder);      % build a reduced model for the robot
    %
    % See also LARGESCALE.BUILDMODEL, MODELREDUCTION.COMPUTEREDUCTION,
    % CONTROLLER.COMPUTECONTROLLER, STATEOBSERVER.COMPUTEOBSERVER
    
    properties
        name                        % name of studied soft robot
        SysLarge_c                  % large scale continuous model
        SysLarge_d                  % large scale discrete model
        SimulationData              % SOFA simulation data
        SysMatrices                 % mass, damping and stiffness matrices
        methodRed                   % method used for model reduction
        r                           % reduced order 
        SysRed_c                    % continous time reduced model
        SysRed_d                    % discrete time reduced model
        DataRed                     % model reduction data        
        methodControl               % method used for dynamic control design
        f                           % feedback matrix
        Sys_star                    % discrete time reference model
        DataCL                      % closed loop data
        methodObs                   % method used to compute observer matrix
        obs                         % observer matrix
        DataObs                     % observer data
        SysE_d                      % discrete time extended system used for observer design
    end
    
    methods(Static)
        function [obj,status] = modelling(obj,env_var, launchSofa, useExistingTextFiles, useSavedMatFile)
            %SOFTROBOT_CLASS.MODELLING Constructs a model of the studied soft robot
            %   takes input from sofa and returns large-scale model         
            [obj.SysLarge_d, obj.SysLarge_c, obj.SimulationData, obj.SysMatrices,status]=...
                largeScale.buildModel(obj.name,env_var, launchSofa, useExistingTextFiles, useSavedMatFile);
        end
        
        function obj = reduction(obj,env_var, launchSofa, useExistingSnapshotsFile)
            %SOFTROBOT_CLASS.REDUCTION builds a reduced order model
            %   takes large system and performs model reduction
            [ obj.SysRed_c, obj.SysRed_d, obj.DataRed, obj.SysMatrices] = ...
                modelReduction.computeReduction( obj.methodRed,...
                obj.r,obj.SysLarge_c,obj.SysLarge_d,obj.SimulationData,...
                obj.SysMatrices,env_var,obj.name, launchSofa, useExistingSnapshotsFile);
        end
        
        function obj = controller(obj)
            %SOFTROBOT_CLASS.CONTROLLER computes a feedback controller
            %   takes reduced system and computes reduced feedback matrix
            [ obj.f, obj.DataCL, obj.Sys_star ] = controller.computeController( obj.methodControl,obj.SysRed_c,...
                obj.SysRed_d,obj.name,obj.DataRed,obj.SysMatrices,obj.SimulationData,obj.SysE_d, obj.obs,...
                obj.SysLarge_d);
        end
        
        function obj = observer(obj)
            %SOFTROBOT_CLASS.OBSERVER constructs an observer
            %   takes reduced model and constructs reduced observer
            [ obj.obs, obj.DataObs, obj.SysE_d ] = stateObserver.computeObserver( obj.methodObs,...
                obj.SysRed_d,obj.SimulationData);
        end
    end
end

