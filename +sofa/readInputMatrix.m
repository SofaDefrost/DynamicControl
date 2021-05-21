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

function [ H ] = readInputMatrix( modelName,simulationData,SofaFolder )
%READINPUTMATRIX returns the matrix H of actuators contribution.

% read a text file from SOFA
% this text file contains indices of the nodes concerned by the actuation
% and the direction of the actuation.
% The number of nodes affected by each actuators may be different from one
% actuator to another. One cannot directly load the txt files, as the
% number of columns may be different. The use of fgetl permits to read the
% text file line by line.
% % each line represent one actuator :
% first element of the line is the number of the actuator, second element
% is the number of nodes concerned by the related actuator.
% Then third element is the indice of the node affected by the actuator and
% fourth, fifth and sixth element are the direction of actuation in
% direction x,y and z.
% And so on : 7th variable is the index of the second node affected by this
% actuator and 8th, 9th and 10th variable are the direction of this second
% node in direction x,y and z.

% The H matrix is defined as follows:
% [1st node of 1st actuator in direction x, 1st node of 2nd actuator in x;
% 1nd node of 1st actuator in direction y, 1nd node of 2nd actuator in y;
% 1nd node of 1st actuator in direction z, 1nd node of 2nd actuator in z;
% 2nd node of 1st actuator in direction x, 2nd node of 2nd actuator in x;
% ...]

% If node one is affected by the 2nd actuator, the 2nd row of H will be
% filled from 1st to 3rd values.
% If node two is affected by the 3rd actuator, the 3rd row of H will be
% filled from 4th to 6th values.
% If node n is affected by the actuator m, the mth row of H will be
% filled from value (n*3)-2 to value (n*3).

if nargin < 3
    error('Not enough input arguments.');
end

%%
numNodes=simulationData.numNodes;
nbActuators=simulationData.nbActuators;

%% load text file from SOFA
fid=fopen(strcat(SofaFolder,'/indxActuators',modelName,'.txt'),'r');
actuator=cell(nbActuators,1);
for i=1:nbActuators
    actuator{i,1}=str2num(fgetl(fid));
end
fclose(fid);

%% Construct Input Matrix
H=zeros(3*numNodes,nbActuators);

for i=1:nbActuators
    currentActuator=actuator{i};
    for idx=3:4:length(currentActuator)
        % handle the shift between matlab and Sofa, sofa indices start at 0
        % but matlab at 1, so increment every index of SOFA
        currentActuator(idx)=currentActuator(idx)+1;
        %
        indexInMatrixH=((currentActuator(idx)*3)-2):(currentActuator(idx)*3);
        H(indexInMatrixH,i)=currentActuator(idx+1:idx+3)';
    end
end
end

