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

function [SimulationData,SysMatrices] = readSofaTextFiles(modelName,pathToSofaData)
%READSOFATEXTFILES Summary of this function goes here
%   Detailed explanation goes here

if nargin < 2
    error('Not enough input arguments.');
end

SimulationData=sofa.readSimulationData(modelName,pathToSofaData);

%% mass, damping and stiffness matrices
if ~SimulationData.isUsingBeamAdapter
    % if Beam adapter is not used in sofa simulation, only MDKmatrix.txt is
    % needed to construct model
    [massMatrix,stiffnessMatrix,dampingMatrix ] = sofa.readSystemMatricesFromMDKFile(SimulationData, modelName,pathToSofaData); % if template = 'vec3D'
elseif SimulationData.isUsingBeamAdapter
    % if beam adapter is used in sofa simulation, several text files have to
    % be read to construct the model. This is done in
    % buildSystemMatricesUsingBeamAdapter.m
    [massMatrix,stiffnessMatrix,dampingMatrix ] =sofa.buildSystemMatricesUsingBeamAdapter(modelName,pathToSofaData,SimulationData);
end

SysMatrices.mass=massMatrix;
SysMatrices.stiffness=stiffnessMatrix;
SysMatrices.damping=dampingMatrix;

%% input matrix
if ~SimulationData.isUsingBeamAdapter
    if ~SimulationData.isUsingReduction
        % if beam adapter is not used and if template of mechanical object =
        % 'Vec3D', use sofa.readInputMatrix
        % warning('[%s] %s %s',mfilename,'ReadInputMatrixWithRotations does not give accurate results.',...
        %    'Use readInputMatrixFromIdentification instead.');
        % inputMatrix=sofa.readInputMatrixFromIdentification(modelName,SimulationData,SysMatrices,pathToSofaData );
        inputMatrix=sofa.readInputMatrix( modelName,SimulationData,pathToSofaData );
    elseif SimulationData.isUsingReduction
        inputMatrix=sofa.readInputMatrixForReducedScene(modelName,SimulationData,pathToSofaData );
    end
elseif SimulationData.isUsingBeamAdapter
    % if beam adapter is used, mechanical object template = Rigid, that
    % implies that the state includes rotations
    % sofa.readInputMatrixWithRotations deals with this issue by removing
    % the rotation from the actuators contributions
    warning('[%s] %s %s',mfilename,'For Echelon scene, readInputMatrixWithRotations does not give accurate results.',...
        'Use readInputMatrixFromIdentification instead.');
    inputMatrix=sofa.readInputMatrixWithRotations(modelName,SimulationData,pathToSofaData ); % if template = 'rigid'
    %inputMatrix=sofa.readInputMatrixFromIdentification(modelName,SimulationData,SysMatrices,pathToSofaData );
end

SysMatrices.input=inputMatrix;
end