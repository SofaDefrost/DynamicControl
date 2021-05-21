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

function [globalMassMatrix,globalStiffnessMatrix,globalDampingMatrix] = buildSystemMatricesUsingBeamAdapter(modelName,pathToSofaData,SimulationData)
%BUILDSYSTEMMATRICESUSINGBEAMADAPTER construct mass, compliance and damping
%matrices from sofa files if sofa is using beam adapter in the scene.
%
% See also: SOFA.READSYSTEMMATRICESFROMMDKFILE, SOFA.READSOFATEXTFILES

% explanation for stiffness:
% in sofa, computation of local matrices K00,K01,K10 and K11 for each beams
% of the model. Each of these matrix is of size 6x6.
% They are saved as localStiffnessMatrix=[K00 K01 K10 K11], which is a
% matrix of size 6 x 24.
% for each beam, the local stiffness matrix is saved:
% globalStiffnessMatrix = [locaStiffnessMatrix_beam1
% localStiffnessMatrix_beam2 ...], this is a matrix of size 6 x (24*numBeams)
% It's the same for mass matrix

if nargin < 3
    error('Not enough input arguments.');
end

%% Read sofa text files
numBeamsTxtFiles = dir(fullfile(pathToSofaData,'numBeams*'));
nbBeamsTopology = size(numBeamsTxtFiles,1);
numBeams = zeros(1,nbBeamsTopology);
nameTopologies = cell(1,nbBeamsTopology);
for i = 1 : nbBeamsTopology
    numBeams(i)= load(fullfile(pathToSofaData,numBeamsTxtFiles(i).name));
    nameTopologies{i} = numBeamsTxtFiles(i).name(10:end-4); % remove 'numBeams_' and '.txt'
end

[indexK,indexM,matrixK,matrixM]=readMatrices(pathToSofaData,modelName, nameTopologies);

%%
% unitl this point:
% matrixK=[K00_node0;K01_node0;K10_node0;K11_node0;K00_node1;K01_node1;...];
% idem for matrixM

%%
numNodes=cell(1,nbBeamsTopology);
vec_total=[];
for i = 1 : nbBeamsTopology
    numNodes{i}=numel(unique(indexM{i}));
    vec_total = [vec_total indexM{i}'];
    numNodesTotal=numel(unique(vec_total));
end

if ~isequal(numNodes{1},numel(unique(indexK{1}))) % should be equal for all indices, not only one
    error('[%s] %s]',mfilename,'Number of nodes of topology is incorrect');
end

listOfMatrices=cell(1,nbBeamsTopology);
for i = 1 : nbBeamsTopology
    for idx=1:24:(numBeams(i)*24)
        clear Node_i;
        Node_i.K00=matrixK{i}(idx:idx+5,:);
        Node_i.K01=matrixK{i}(idx+6:idx+11,:);
        Node_i.K10=matrixK{i}(idx+12:idx+17,:);
        Node_i.K11=matrixK{i}(idx+18:idx+23,:);
        
        Node_i.M00=matrixM{i}(idx:idx+5,:);
        Node_i.M01=matrixM{i}(idx+6:idx+11,:);
        Node_i.M10=matrixM{i}(idx+12:idx+17,:);
        Node_i.M11=matrixM{i}(idx+18:idx+23,:);
        listOfMatrices{i}=[listOfMatrices{i} Node_i];
    end
end

%%
% until this point
% numNodes= number of nodes for each topology
% numNodesTotal = total number of nodes
% listOfMatrices = [node1 node2 node3 ...] where
% node1 is a struct with fields M00,M01,M10, M11 and K00, K01, K10, K11

%% assemble global Matrices
stiffnessMatrix=cell(1, nbBeamsTopology);
massMatrix=cell(1, nbBeamsTopology);

for i = 1 : nbBeamsTopology
    stiffnessMatrix{i}=zeros(6*numNodesTotal);
    massMatrix{i}=zeros(6*numNodesTotal);
    
    numStartNodes=indexK{i}(:,1)+1;  % nodes from where beam starts, +1 because sofa indices start at 0, matlab at 1
    numEndNodes=indexK{i}(:,2)+1;    % nodes where beam ends, +1 because sofa indices start at 0, matlab at 1
    
    for k=1:numel(numStartNodes)
        idxStartNode=1+(numStartNodes(k)-1)*6;    % if idx = 1 : values from 1 to 6, if idx = 2 : values from 7 to 13 etc...
        idxEndNode=1+(numEndNodes(k)-1)*6;
        
        stiffnessMatrix{i}(idxStartNode:idxStartNode+5,idxStartNode:idxStartNode+5)=...
            stiffnessMatrix{i}(idxStartNode:idxStartNode+5,idxStartNode:idxStartNode+5)+listOfMatrices{i}(k).K00;
        stiffnessMatrix{i}(idxStartNode:idxStartNode+5,idxEndNode:idxEndNode+5)=...
            stiffnessMatrix{i}(idxStartNode:idxStartNode+5,idxEndNode:idxEndNode+5)+listOfMatrices{i}(k).K01;
        stiffnessMatrix{i}(idxEndNode:idxEndNode+5,idxStartNode:idxStartNode+5)=...
            stiffnessMatrix{i}(idxEndNode:idxEndNode+5,idxStartNode:idxStartNode+5)+listOfMatrices{i}(k).K10;
        stiffnessMatrix{i}(idxEndNode:idxEndNode+5,idxEndNode:idxEndNode+5)=...
            stiffnessMatrix{i}(idxEndNode:idxEndNode+5,idxEndNode:idxEndNode+5)+listOfMatrices{i}(k).K11;
        
        massMatrix{i}(idxStartNode:idxStartNode+5,idxStartNode:idxStartNode+5)=...
            massMatrix{i}(idxStartNode:idxStartNode+5,idxStartNode:idxStartNode+5)+listOfMatrices{i}(k).M00;
        massMatrix{i}(idxStartNode:idxStartNode+5,idxEndNode:idxEndNode+5)=...
            massMatrix{i}(idxStartNode:idxStartNode+5,idxEndNode:idxEndNode+5)+listOfMatrices{i}(k).M01;
        massMatrix{i}(idxEndNode:idxEndNode+5,idxStartNode:idxStartNode+5)=...
            massMatrix{i}(idxEndNode:idxEndNode+5,idxStartNode:idxStartNode+5)+listOfMatrices{i}(k).M10;
        massMatrix{i}(idxEndNode:idxEndNode+5,idxEndNode:idxEndNode+5)=...
            massMatrix{i}(idxEndNode:idxEndNode+5,idxEndNode:idxEndNode+5)+listOfMatrices{i}(k).M11;        
    end
end

%% assemble all matrices
globalStiffnessMatrix=zeros(6*numNodesTotal);
globalMassMatrix=zeros(6*numNodesTotal);
for i = 1 : nbBeamsTopology
   globalStiffnessMatrix = globalStiffnessMatrix+ stiffnessMatrix{i};
   globalMassMatrix = globalMassMatrix+ massMatrix{i};
end

globalDampingMatrix=SimulationData.rayMass*globalMassMatrix+SimulationData.rayStiffness*globalStiffnessMatrix;

%% remove rotations
% for each matrix :
% [x_node1 y_node1 z_node1 rot_x_node1 rot_y_node1 rot_z_node1 x_node2
% y_node2 z_node2 rot_x_node2 ...]
% keep only [x_node1 y_node1 z_node1 x_node2 y_node2...]

idxToKeep=zeros(1,numNodesTotal*3);
idx=1;
for i=1:6:(numNodesTotal*6)-2
    idxToKeep(idx:idx+2)=(i:i+2);
    idx=idx+3;
end

globalStiffnessMatrix=globalStiffnessMatrix(idxToKeep,idxToKeep);
globalMassMatrix=globalMassMatrix(idxToKeep,idxToKeep);
globalDampingMatrix=globalDampingMatrix(idxToKeep,idxToKeep);

end


%% Local Functions
function [indexK,indexM,matrixK,matrixM]=readMatrices(pathToSofaData,modelName, nameTopologies)

nbBeamsTopo=size(nameTopologies,2);
indexK=cell(1,nbBeamsTopo);
indexM=cell(1,nbBeamsTopo);
matrixK=cell(1,nbBeamsTopo);
matrixM=cell(1,nbBeamsTopo);

for i = 1:nbBeamsTopo
    f=fopen(fullfile(pathToSofaData,strcat('indexK_',nameTopologies{i},'_',modelName,'.txt')));
    indexK{i}=cell2mat(textscan(f, '%d %d'));
    fclose(f);
    
    f=fopen(fullfile(pathToSofaData,strcat('indexM_',nameTopologies{i},'_',modelName,'.txt')));
    indexM{i}=cell2mat(textscan(f, '%d %d'));
    fclose(f);
    
    f=fopen(fullfile(pathToSofaData,strcat('matrixK_',nameTopologies{i},'_',modelName,'.txt')));
    matrixK{i} = textscan(f,'%s', 'delimiter', ' ');
    fclose(f);
    matrixK{i} =str2double(matrixK{i}{1});
    matrixK{i}(isnan(matrixK{i}))=[];
    matrixK{i}=reshape(matrixK{i},numel(matrixK{i})/6,6);
    
    f=fopen(fullfile(pathToSofaData,strcat('matrixM_',nameTopologies{i},'_',modelName,'.txt')));
    matrixM{i} = textscan(f,'%s', 'delimiter', ' ');
    fclose(f);
    matrixM{i} =str2double(matrixM{i}{1});
    matrixM{i}(isnan(matrixM{i}))=[];
    matrixM{i}=reshape(matrixM{i},numel(matrixM{i})/6,6);
end

end
