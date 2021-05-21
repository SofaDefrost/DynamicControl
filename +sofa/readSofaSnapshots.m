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

function [velocitySnapshots, positionSnapshots] =readSofaSnapshots(SofaFolder,SimulationData)
%SNAPSHOTMATRIX returns a matrix containing snapshots saved using the model
%order reduction plugin for sofa.

% text structure is:
% T =
% X=
% X0 =
% and again T= ...
% Goal : build matrice S=(X(t0) X(t1) X(t2) ... X(tf) )

fid=fopen(fullfile(SofaFolder.data,'/reduction/debug/stateFile.state'),'r');
pos_Snap_cell=textscan(fid,'%s','delimiter','=');
fclose(fid);

% fid=fopen(fullfile(SofaFolder.data,char(modelName),'/reduction/debug/stateFileVelocity.state'),'r');
% pos_Snap_cell=textscan(fid,'%s','delimiter','=');
% fclose(fid);
vel_Snap_cell=pos_Snap_cell;

if ~isequal(size(vel_Snap_cell{1},1),size(pos_Snap_cell{1},1))
    error('[%s] %s', mfilename, 'Velocity and position snapshots should have equal lengths');
end

%% now, S={'T';t0;'X',position(t0);'X0';positionX0;'T';t1,'X',position(t1)...}
% remove 'T' and 'X0'

indxToRemove=[];
warning('[%s] %s',mfilename,'TODO: find size(indxToRemove) to speed up computation');
% it depends on how 'X0' is saved, it is saved at the beginning but not at
% the end...

for i=1:size(pos_Snap_cell{1},1)
    if isequal(pos_Snap_cell{1}(i),{'T'})
        assert(isequal(vel_Snap_cell{1}(i),{'T'}));
        indxToRemove=[indxToRemove i i+1]; % i and i + 1 to remove 'T' and the value of T on the next line
    elseif isequal(pos_Snap_cell{1}(i),{'X0'})  
        %assert(isequal(vel_Snap_cell{1}(i),{'V0'}));      
        assert(isequal(vel_Snap_cell{1}(i),{'X0'}));
        indxToRemove=[indxToRemove i i+1]; % i and i + 1 to remove 'X0' and the value of X0 on the next line
    elseif isequal(pos_Snap_cell{1}(i),{'X'})  
        %assert(isequal(vel_Snap_cell{1}(i),{'V'}));    
        assert(isequal(vel_Snap_cell{1}(i),{'X'}));
        indxToRemove=[indxToRemove i]; % only i to remove 'X' and keep the value of X on the next line
    end
end

vel_Snap_cell{1}(indxToRemove)=[];
pos_Snap_cell{1}(indxToRemove)=[];

velocitySnapshots=zeros(SimulationData.numNodes*3,size(vel_Snap_cell{1},1));
positionSnapshots=zeros(SimulationData.numNodes*3,size(vel_Snap_cell{1},1));

warning('[%s] %s',mfilename,'TODO: make _str2double_ work. For now it returns NaN, so str2num is used instead');
for i=1:size(vel_Snap_cell{1},1)
    tmp_vel=str2num(cell2mat(vel_Snap_cell{1}(i)));
    tmp_pos=str2num(cell2mat(pos_Snap_cell{1}(i)));
    if SimulationData.isUsingBeamAdapter
        if ~isequal(numel(tmp_vel),numel(tmp_pos),7*SimulationData.numNodes)
            error('Invalid number of variables in the snapshots using quaternions.');
        end
        tmp_vel = removeQuatFromState(tmp_vel, SimulationData.numNodes);
        tmp_pos = removeQuatFromState(tmp_pos, SimulationData.numNodes);
    end
    velocitySnapshots(:,i)=tmp_vel';
    positionSnapshots(:,i)=tmp_pos';
end

end


function state_vector_without_quat = removeQuatFromState(state_vector, numNodes)
% remove quaternions from state vector
% when one use beam adapter, state vector is not only position and velocity
% it inclues quaternions. 
% pos = [posx, posy, pos, qx, qy, qz, qw]
% same for velocity
% (qx, qy, qz, qw) = vector of dimension 1x4 = quaternion
% it is not taken into account in the control law so we remove it

idx_quat = zeros(1,4*numNodes);

for i = 1 : numNodes
   idx_quat([1 2 3 4]+4*(i-1)) = [4 5 6 7]+7*(i-1);  
end

state_vector_without_quat = state_vector;
state_vector_without_quat(idx_quat) = [];

end