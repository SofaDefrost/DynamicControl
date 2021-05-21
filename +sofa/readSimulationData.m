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

function [ simulationData ] = readSimulationData( modelName,sofaDataFolder)
%READSIMULATIONDATA Read a text files generated in SOFA, returns SOFA data
%   SOFA scene 'modelling...' writes in text files data associated to the
%   studied soft robot.
%   The file structure used in this file should match the structure used to
%   store the data. These data are saved in the modelling Python Script,
%   using the saveSimulationData.py function
%   The file format is:
%       Name of first variable
%       Value of first variable
%       Name of second variable
%       Value of second variable
%   Results are stored in cell, and then convert to numeric values.
%   str2num is used only to convert the indices of effectors as this
%   variable may be a list and converting list to numeric value using
%   str2double may lead to NaN.

if nargin < 2
    error('Not enough input arguments.');
end

%% Get simulation data 
fid=fopen(strcat(sofaDataFolder,'/SimuData',modelName,'.txt'),'r');
res = textscan(fid,'%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s','Delimiter','\n');

numNodes=str2double(cell2mat(res{2}));
rayMass=str2double(cell2mat(res{4}));   
rayStiffness=str2double(cell2mat(res{6}));   
dt=str2double(cell2mat(res{8}));      
nbActuators=str2double(cell2mat(res{10}));
nbEffectors=str2double(cell2mat(res{12}));
effectors=str2num(cell2mat(res{14})); %#ok<ST2NM>
maxForceActuators=str2double(cell2mat(res{16}));
isUsingBeamAdapter=str2double(cell2mat(res{18}));
isUsingReduction=str2double(cell2mat(res{20}));

%% Get free degree of freedom directly read from sofa file
fid = fopen(strcat(sofaDataFolder,'/indxFreeNodes',modelName,'.txt'),'r');
free_dof_char = fscanf(fid,'%s');       
fclose(fid);

freeDOF=str2num(free_dof_char); %#ok<ST2NM>
% SOFA indices starts at 0, Matlab at 1
freeDOF=freeDOF+1;                  %Offset between Sofa and Matlab

%% indices of free nodes :
if ~isUsingReduction
% 4 nodes robot example :
% if nodes 2 and 3 are free, i.e if 
% freeDOF = [2 3]
% and displacement vector is written as :
% d = [ dx1 dy1 dz1 dx2 dy2 dz2 dx3 dy3 dz3 dx4 dy4 dz4]
% one should keep :
% indices = [4 5 6 7 8 9] = [(freeDOF*3)-2 (freeDOF*3)-1 freeDOF]
freeIndx=sort([(freeDOF*3)-2 (freeDOF*3)-1 freeDOF*3]);
else
    freeIndx=1:numNodes;    % is sofa scene uses model reduction, then the state are the modes and no modes is attached
end

%% final assembly       
simulationData=struct('freeDOF',freeDOF,'freeIndx',freeIndx,...
                'numNodes',numNodes,'dt',dt,...
                'rayMass',rayMass,'rayStiffness',rayStiffness,...
                'nbActuators',nbActuators,...
                'nbEffectors',nbEffectors,...
                'effectors',effectors,...
                'maxForceActuators',maxForceActuators,...
                'isUsingBeamAdapter',isUsingBeamAdapter,...
                'isUsingReduction',isUsingReduction);

end