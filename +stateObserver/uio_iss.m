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

function [obs,DataObs,SysE_d] = uio_iss(SysRed_d,nbDeriv,SimulationData)
%UIO_ISS Unknown Input Observer (UIO)
% Design an observer for discrete-time reduced order model
% x_hat_k+1 = A_r x_hat_k + B_r u + B_r du + e
% where d_u is the unknown input considered as an unknown input and e are
% the remaining modeling errors.
% d_u is reconstructed via the unknown input observer and can be used in the
% controller.
%
% See also COMPUTEOBSERVER, BUILDEXTENDEDOBSERVER

if nargin < 3
    error('Not enough input arguments.');
end

SysE_d=stateObserver.buildExtendedObserver(SysRed_d, SimulationData, nbDeriv);

% if rank(obsv(SysE_d.a,SysE_d.c))~=size(SysE_d.a,1)
%     error('Rank of extended observability matrix = %d, size of extended system = %d. %s',...
%         rank(obsv(SysE_d.a,SysE_d.c)), size(SysE_d.a,1),'Extended system not observable.');
% end

[Ae_d,~,Ce_d,~]=ssdata(SysE_d);

%% ISS Observer
% maximize alpha (bounded) and minimize gamma
ny=size(SysE_d.c,1);
nx=size(SysRed_d.a,1);
ne=size(Ae_d,1);

alpha_o=0.15;%.25;
gamma_o=0.05;%sdpvar(1);

Fe=sdpvar(ne,ny,'full');        % Qo * Ke
Qo=sdpvar(ne);

lyap_positive=(Qo >= 1e-6*eye(size(Qo,1)));

coeff_iss=(gamma_o >= 1e-6*eye(size(gamma_o)));

matrix11=[-Qo+alpha_o*Qo,      zeros(ne,nx)  ;
    zeros(nx,ne)    -gamma_o*eye(nx)];

matrix21=[Qo*Ae_d-Fe*Ce_d,  Qo*[eye(nx);zeros(ne-nx,nx)]];

iss_obs=([matrix11, matrix21';
    matrix21,   -Qo]<=-1e-6*eye(2*ne+nx));

%% Bound observer gain
lambda=10;
obsBounded=([-lambda*Qo  Fe;Fe'  -eye(ny)]<=-1e-4);

%% solve LMI
lmiConstraints=lyap_positive+iss_obs+obsBounded;%+coeff_iss;

options = helper.changeSDPsettings;
sol=optimize(lmiConstraints,[],options);
if sol.problem
    error('[%s] %s: %s',mfilename, 'LMI not solved', yalmiperror(sol.problem));
else
    obs=value(Qo)\value(Fe);
    value(gamma_o)
    if all(abs(eig(Ae_d-obs*Ce_d))<1)
        DataObs.eig=abs(eig(Ae_d-obs*Ce_d));
        DataObs.alpha_o=value(alpha_o);
        DataObs.gamma_o=value(gamma_o);
        DataObs.Qo=value(Qo);
    else
        error('Observer unstable');
    end
end
end