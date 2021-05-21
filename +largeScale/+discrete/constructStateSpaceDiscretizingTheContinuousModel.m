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

function [ SysLarge_d ] = constructStateSpaceDiscretizingTheContinuousModel(SimulationData,SysMatrices)
%CONSTRUCTSTATESPACE construct a discrete time large-scale state space model.

%%% check if input arguments are valid
if nargin < 2
    error('Not enough input arguments.');
end

helper.checkInputArguments({SimulationData, SysMatrices},...
                           {@isstruct, @isstruct});
                       

%% useRed is used to make the difference between a reduced scene and a non-reduced scene
% if the scene is not reduced, mechacanical object gives position of each
% nodes in 3 direction of space
% if scene is reduced, mechanical object gives weight of each mode.

%% init
useRed=SimulationData.isUsingReduction;
dt=SimulationData.dt;
freeIndx=SimulationData.freeIndx;
numNodes=SimulationData.numNodes;

%% Implement discretization of equation 3.20



%% construct State space
SysLarge_d=ss(AmatrixFree,Bd,C,0,dt);
end

