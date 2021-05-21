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

function [ H ] = readInputMatrixForReducedScene( modelName,simulationData,SofaFolder )

if nargin < 3
    error('Not enough input arguments.');
end

warning('[%s] %s',mfilename,'Input matrix constructed for reduced scene');

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
H=zeros(numNodes,nbActuators);

for i=1:nbActuators
    currentActuator=actuator{i};
    for idx=3:2:length(currentActuator)
        % handle the shift between matlab and Sofa, sofa indices start at 0
        % but matlab at 1, so increment every index of SOFA
        currentActuator(idx)=currentActuator(idx)+1;
        %
        indexInMatrixH=currentActuator(idx);
        H(indexInMatrixH,i)=currentActuator(idx+1);
    end
end
end

