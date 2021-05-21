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

function [ SysLarge_c ] = constructStateSpace(SimulationData,SysMatrices)
%CONSTRUCTSTATESPACE returns SysLarge_c, a continuous time large-scale
% dynamical model of the studied soft robot.
%
% See also LARGESCALE.BUILDMODEL, SOFTROBOT_CLASS/MODELLING

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

%%
useRed=SimulationData.isUsingReduction;
freeIndx=SimulationData.freeIndx;
numNodes=SimulationData.numNodes;

mass=SysMatrices.mass(freeIndx,freeIndx);
compliance=SysMatrices.stiffness(freeIndx,freeIndx);

%damping=SysMatrices.damping(freeIndx,freeIndx);
damping=SimulationData.rayMass*mass+SimulationData.rayStiffness*compliance;

%% assembly
A=[-mass\damping -mass\compliance;
    eye(size(freeIndx,2)) zeros(size(freeIndx,2))];

inputMatrix=SysMatrices.input(freeIndx,:);
B=[mass\inputMatrix; 0*inputMatrix];

%% C matrix
% the C matrix does not change with model reduction, it still needs to
% multiply the position of each nodes
effectors=SimulationData.effectors+1;   % +1 because Sofa index starts at 0

% output = displacement of effector, velocity will be set to zeros later
C=zeros(3*numel(effectors),3*numNodes);
if ~useRed
    for idx=1:numel(effectors)
        C(3*idx-2,(3*effectors(idx))-2)=1;
        C(3*idx-1,(3*effectors(idx))-1)=1;
        C(3*idx,(3*effectors(idx)))=1;
    end
    C=C(:,freeIndx);
    C=[0*C C]; % [0*C] for velocity
    
    %%% Use the following to use only a subset of the outputs.
    useXYZasOutput = 1;
    if ~useXYZasOutput
        warning('[%s] %s',mfilename, 'use only x and y axis as output');
        toRemove=[2 3]; % x,y,z : 1,2,3
        listIdxToRemove=[];
        for i=1:numel(toRemove)
            listIdxToRemove=[listIdxToRemove toRemove(i):3:3*numel(effectors)]; %#ok<AGROW>
        end
        C(listIdxToRemove,:)=[];
    end
else
    % if sofa scene use model reduction, numNodes is the number of modes
    % but matrix C needs to be of the size of the number of nodes. So we
    % need to define it
    error('[%s] %s',mfilename,'Should be changed according to the non reduced case.');
    warning('[%s] %s',mfilename,'Adapt totalNumNodes for different robots.');
    totalNumNodes=11810;
    C=zeros(3,6*totalNumNodes);
    C(1,end/2+1+(3*effectors)-2)=1;
    C(2,end/2+1+(3*effectors)-1)=1;
    C(3,end/2+1+(3*effectors))=1;
    warning('[%s] %s',mfilename,'Need to adapt to other name of robot');
    modes=load('home/maxime/Documents/MATLAB/SoftRobots/SofaScenes/SimulationData/dynamic_arm_3d/matrixV.txt');
    V=[modes 0*modes;0*modes modes];
    C=C*V;
end

%% Final assembly

SysLarge_c=ss(A,B,C,0);
end

