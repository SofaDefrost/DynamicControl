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

function [ SysLarge_d ] = constructStateSpace(SimulationData,SysMatrices)
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

rm=SimulationData.rayMass;        % rayleigh mass
rk=SimulationData.rayStiffness;   % rayleigh stiffness

M=SysMatrices.mass;
K=SysMatrices.stiffness;
nq = size(M,1);

D=rm*M+rk*K;       
S=M+dt*D+(dt^2)*K;
iS=S\eye(size(S));

%%% state space matrices
%% A matrix
Amatrix=[eye(nq)-dt*iS*(dt*K+D),        -dt*iS*K
        dt*eye(nq)-(dt^3)*iS*K,     eye(nq)-(dt^3)*iS*K];
       
if ~useRed
    AmatrixFree=Amatrix([freeIndx freeIndx+3*numNodes],[freeIndx freeIndx+3*numNodes]);
else
    AmatrixFree=Amatrix;    % all modes are free
end

%% B matrix
inputMatrix=SysMatrices.input(freeIndx,:);
if ~useRed
    Bd=[dt*iS(freeIndx,freeIndx)*inputMatrix;
        (dt^2)*iS(freeIndx,freeIndx)*inputMatrix];
else
    Bd=[M\inputMatrix; 0*inputMatrix];
end

%% C matrix
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
    % the C matrix does not change with model reduction, it still needs to
    % multiply the position of each nodes
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

%% construct State space
SysLarge_d=ss(AmatrixFree,Bd,C,0,dt);
end

