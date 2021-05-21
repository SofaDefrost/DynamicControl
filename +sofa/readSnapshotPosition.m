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

function [ snapshotMatrix ] = readSnapshotPosition( numNodes,modelName,sofaDataFolder )
%READSNAPSHOTPOSITION Read a text file saved in sofa containing position
%of the robot at each time step. Returns position snapshot matrix.
%
% See also SOFA.COMPUTEMOREPOD, SOFA.COMPUTEREDUCTION,
% SOFA.READSNAPSHOTVELOCITY

if nargin < 3
    error('Not enough input arguments.');
end

%snapshotTxt is a cell array where each cell contains a value of the snapshot file written by SOFA
% snapshotTxt=[T=t1 V=v1 T=t2 V=V2 T=t3 V=V3...] 
fid=fopen(strcat(sofaDataFolder,'/',modelName,'/SnapshotPosition',modelName,'.txt'),'r');
snapshotTxt=textscan(fid,'%s', 'delimiter', ' ');   % read Snapshot text file
fclose(fid);

% snapshotVector is a vector where every char of snapshotTxt are replace by
% NaN.
snapshotVector=zeros(1,cellfun(@length,snapshotTxt));
for i=1:cellfun(@length,snapshotTxt) 
    snapshotVector(i)=str2double(snapshotTxt{1}{i});
end

% remove NaN
snapshotVector(isnan(snapshotVector))=[];

%snapshotVector is now a vector defined as:
% [t1 V(t1) t2 V(t2) t3 V(t3) ...]
% next step is to reshape it to be a matrix s.t. 
% [t1     t2     t3     ...
%  V(t1)' V(t2)' V((3)' ...]

res = load(strcat(sofaDataFolder,'/',modelName,'/SimuData',modelName,'.txt'),'r');
finalTimeSnapshot=res(end);

% dimension of snapshot matrix :
nRow=3*numNodes+1;  % +1 because of dt
nCol=finalTimeSnapshot;

snapshotMatrix=reshape(snapshotVector,nRow,nCol);

snapshotMatrix=snapshotMatrix(2:end,:);           % remove dt in Snapshot matrix
