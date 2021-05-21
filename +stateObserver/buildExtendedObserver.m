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

function [SysE_d] = buildExtendedObserver(SysRed_d, SimulationData, nbDeriv)
%BUILDEXTENDEDOBSERVER builds observer system to compute observer gain
% it supposes that the unknown input is omega, distributed along the reduced
% order system through following equation : xr = Ar xr + Br u + Br omega
% the unkown input is also of dimension nu, the number of input of the
% system
%
% See also computeObserver

if nargin < 3
    error('Not enough input arguments.');
end

[Ar_d,Br_d,Cr_d,~]=ssdata(SysRed_d);
nu=size(Br_d,2);
r=size(SysRed_d.a,1);
dt=SimulationData.dt;

Br_tilde=[Br_d zeros(r,nu*nbDeriv)];

if nbDeriv ==0
    J = eye(nu);
else    
sizeEye = nu ;
J =  eye(nu);
for i=1:nbDeriv
    J = [J, [zeros((i-1)*sizeEye,sizeEye); eye(sizeEye)];
        zeros(sizeEye,i*sizeEye), eye(sizeEye)];
end
end

Ae_d=[Ar_d Br_tilde;
      zeros(size(J,1),size(Ar_d,2)), J];
      
Be_d=[Br_d;zeros(size(J,1),size(Br_d,2))];

Ce_d=[Cr_d zeros(size(Cr_d,1),size(J,2))];

SysE_d=ss(Ae_d,Be_d,Ce_d,0, dt);

end

