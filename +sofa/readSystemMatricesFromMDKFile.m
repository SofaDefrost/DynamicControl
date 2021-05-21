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

function [ Mass,Stiffness,Damping ] = readSystemMatricesFromMDKFile( Data,modelName,pathToSofaData)
%READSYSTEMMATRICES Get Mass, stiffness and Damping Matrix from Sofa
%simulation
%   Read a text file where is stored (M+K*dt+D*dt^2)

if nargin < 3
    error('Not enough input arguments.');
end

%% Mass Matrix ; Mass=Mass' ; Mass>0
% if addLoad=0 in sofa modelling simu, then mass matrix is a coefficient
% times identity vector, i.e. all nodes have the same mass.
% In this case, mass.txt is just a scalar.
% if addLoad = 1 in sofa modelling, then some nodes have more mass than the
% others, and mass.txt is a vector that is the diag of the mass matrix.
diagMass = load(fullfile(pathToSofaData,strcat('/mass',modelName,'.txt')),'r');
if isscalar(diagMass)
    Mass=diag(diagMass*ones(3*Data.numNodes,1));
else
    % diagMass size is numNodes
    % convert it to vecotr of size 3*numNodes
    diagM=zeros(1,3*Data.numNodes);
    diagM(1:3:3*Data.numNodes-2)=diagMass;
    diagM(2:3:3*Data.numNodes-1)=diagMass;
    diagM(3:3:3*Data.numNodes)=diagMass;
    Mass=diag(diagM);
end

%% Stiffness Matrix
fid=fopen(fullfile(pathToSofaData,strcat('MDKmatrix',modelName,'.txt')),'r');
mdkCell = textscan(fid,'%f', 'delimiter', ' ','Whitespace','[]');
mdkDouble = cell2mat(mdkCell);
fclose(fid);
% Convert MDK matrix into numeric format
mdkDouble(isnan(mdkDouble))=[];

% dimension of MDK matrix :
numNodes=Data.numNodes;
if Data.isUsingReduction
    nRow=numNodes;
else
    nRow=3*numNodes;
end 
nCol=nRow;       

MDK=reshape(mdkDouble,nRow,nCol);
MDKsparse=sparse(MDK);

dt=Data.dt;
Stiffness = -( (1+dt*Data.rayMass)*Mass-MDKsparse ) / (dt^2+dt*Data.rayStiffness) ;

%% Damping Matrix
Damping=Data.rayMass*Mass+Data.rayStiffness*Stiffness;

end