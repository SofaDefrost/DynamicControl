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

function [ H ] = readInputMatrixWithRotations( modelName,SimulationData,pathToSofaData )
%READINPUTMATRIXWITHROTATIONS returns the matrix H of actuators contribution
% difference with SOFA.READINPUTMATRIX is the template used in sofa
% simulation. Here, if beam adapter is used, template of mechanical object
% = Rigid, that implies that the state contains rotations. This file remove
% the rotation from the state definition.

% read a text file from SOFA
% this text file contains indices of the nodes concerned by the actuation
% and the direction of the actuation.
% The number of nodes affected by each actuators may be different from one
% actuator to another. One cannot directly load the txt files, as the
% number of columns may be different. The use of fgetl permits to read the
% text file line by line.
% % each line represent one actuator :
% first element of the line is the index of the actuator, second element
% is the number of nodes concerned by the related actuator.
% Then third element is the indice of the node affected by the actuator and
% fourth, fifth and sixth element are the direction of actuation in
% direction x,y and z.

%%% DIFFERENCE WITH REAINPUTMATRIX:
% the 7th 8th and 9th values are the contributions of the actuator for the
% rotations rx ry zz

% And so on : 10th variable is the index of the second node affected by this
% actuator and 11th, 12th and 13th variable are the direction of this second
% node in direction x,y and z, 14th 15th and 16th rotations of this second
% node etc...

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

warning('[%s] %s %s',mfilename,'Input matrix constructed for mechanical object with rotations.',...
    'The rotations are not taken into account in the state space.');

%%
numNodes=SimulationData.numNodes;
nbActuators=SimulationData.nbActuators;

%% load text file from SOFA
fid=fopen(strcat(pathToSofaData,'/indxActuators',modelName,'.txt'),'r');
actuator=cell(nbActuators,1);
for i=1:nbActuators
    actuator{i,1}=str2num(fgetl(fid));
end
fclose(fid);

%% Construct Input Matrix
H=zeros(6*numNodes,nbActuators);    % should be 3*numNodes but is 6 because of rotations

for i=1:nbActuators
    currentActuator=actuator{i};
    for idx=3:7:length(currentActuator)     % should be 3:4 but is 3:7 because of rotations
        % handle the shift between matlab and Sofa, sofa indices start at 0
        % but matlab at 1, so increment every index of SOFA
        currentActuator(idx)=currentActuator(idx)+1;
        %
        indexInMatrixH=((currentActuator(idx)*6)-5):(currentActuator(idx)*6);       % shoudl be *3 and -2 but is *6 and -5 because of rotations
        H(indexInMatrixH,i)=currentActuator(idx+1:idx+6)';
    end
end

%% Remove parts concerning rotations
indicesToKeep=[];
for i=1:numNodes
    indicesToKeep=[indicesToKeep [1 2 3]+6*(i-1)];
end

%H=-0.1*H;

H=H(indicesToKeep,:);

end

