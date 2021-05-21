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

function [ SysRed, SysRed_d, DataRed ] = computeMorePOD( SysLarge_c,SysLarge_d,Snapshot,SimulationData,r)
%COMPUTEMOREPOD computes reduced order system using Proper Orthogonal Decomposition (POD)
% The snapshots are obtained thanks to a sofa simulation that has to be run
% before calling COMPUTEMORPOD
% 
% See also COMPUTEREDUCTION, COMPUTEMOREITIA, REDUCEDORDERVALIDATION,
% SOFA.READSOFASNAPSHOTS

if nargin < 4
    error('Not enough input arguments.');
end

snapshotV=Snapshot.v;
snapshotU=Snapshot.u;

%% Handle possible memory issue
% is number of snapshots is too high, memory issue can occur
% reduced the number of snpashots to avoid this problem
sub_snap = 4;
warning('[%s] %s', mfilename, sprintf('POD computing with only 1/%d of the snapshots.', sub_snap));
snapshotV = snapshotV(:, 1:3:end);
snapshotU = snapshotU(:, 1:3:end);

%% Compute POD

freeIndx=SimulationData.freeIndx;

[modesV,svV,~] = svd(snapshotV(freeIndx,:));
svV=diag(svV);

phiV = modesV(:,1:r);
phibarV=modesV(:,r+1:end);

[modesU,svU,~] = svd(snapshotU(freeIndx,:));
svU=diag(svU);

phiU = modesU(:,1:r);
phibarU=modesU(:,r+1:end);

T=[phiV zeros(numel(freeIndx),r);zeros(numel(freeIndx),r) phiU];

Tbar=[phibarV zeros(numel(freeIndx),size(modesV,1)-r);zeros(numel(freeIndx),size(modesV,1)-r) phibarU];

[Ac,Bc,Cc,~] = ssdata(SysLarge_c); 
[Ad,Bd,Cd,~] = ssdata(SysLarge_d); 

Ar_c=T'*Ac*T;
Br_c=T'*Bc;
Cr_c = Cc*T;

Ar_d=T'*Ad*T;
Br_d=T'*Bd;
Cr_d = Cd*T;

SysRed=ss(Ar_c,Br_c,Cr_c,0);
SysRed_d=ss(Ar_d,Br_d,Cr_d,0,SimulationData.dt);

DataRed.V=T;
DataRed.W=T;
DataRed.Vbar=Tbar;
DataRed.Wbar=Tbar;
DataRed.snapshotsvV=svV;
DataRed.snapshotsvU=svU;

end
