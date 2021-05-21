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

function [ H ] = readInputMatrixFromIdentification( modelName,simulationData,SysMatrices,SofaDataFolder )
%READINPUTMATRIXFROMIDENTIFICATION returns the matrix H of actuators contribution.

% This file should be used if the default readInputMatrix does not provide
% accurate results. 

% This file requires a first SOFA Idenficitation simulation where the 
% positions of the robot under different actuator contribution are saved.

if nargin < 3
    error('Not enough input arguments.');
end

warning('[%s] %s',mfilename,'Input matrix constructed from identification');

%%
numNodes=simulationData.numNodes;
nbActuators=simulationData.nbActuators;
K=SysMatrices.stiffness;

%% load text file from SOFA
final_pos=zeros(3*numNodes,nbActuators);
for indx=1:nbActuators
    final_pos(:,indx)=load(fullfile(SofaDataFolder, modelName,strcat('final_pos_cable',num2str(indx-1),'.txt')))';
end
    
H=K*final_pos;

end

