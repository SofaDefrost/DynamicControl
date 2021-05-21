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

function [L,L_star,sys_star,Li] = tracking_controller(SysRed_d,SysE_d,SysRed)
%TRAJECTORY_TRACKING returns L and L_star to perform dynamic
%trajectory tracking.
%
% Output arguments:
%   L = feedback matrix
%   L_star = feedforward matrix
%   L_i = integrator matrix
%
% Input arguments:
%   SysRed_d: discrete reduced order system
%   SysE_d : discrete extended reduced observer system
%   SysRed: continuous time reduced system, used only to design the reference model
%
% Examples:
%
% See also TRACKING_STABILITY, COMPUTEREDUCEDCONTROLLER, COMPUTECONTROLLER

dt=SysRed_d.Ts;
[Ar_d,Br_d,Cr_d,~]=ssdata(SysRed_d);
Ae_d=SysE_d.a;

nu=size(Br_d,2);
ny=size(Cr_d,1);
nx=size(Ar_d,1);
ne=size(Ae_d,1);
nbDeriv=(ne-nx-nu)/nu;

%% Define reference model
sys_star=controller.buildReferenceModel(nx,ny,dt,SysRed);

%% ISS Controller
% ISS condition with respect to unknown input epsilon = modeling errors

Ar_star=sys_star.a;
Br_star=sys_star.b;
Cr_star=sys_star.c;

alpha=0.05; 
beta=2e1;
gamma=2e1;%sdpvar(1);

omega=sdpvar(1); % to minimize
     
ni = ny;
 
  
Ac = [Ar_d, Ar_d-Ar_star, zeros(nx,ny);
      zeros(nx),    Ar_star, zeros(nx,ny);
      Cr_d,     Cr_d-Cr_star,   eye(ny)];  

Cr_tilde=[Cr_d,  Cr_d-Cr_star,  zeros(ny,ni)];
  
F=sdpvar(nu,nx+ni+nx,'full');         % 

Be=[Br_d;zeros(ni,nu);0*Br_d];
Br_tilde=[Br_d zeros(nx,nu*nbDeriv);
        zeros(nx+ni,nu+nu*nbDeriv)];
    
Xt=sdpvar(nx+ni+nx);
matrix21=[Ac*Xt-Be*F, [Be*F, Br_tilde],  [-Br_star;Br_star;zeros(ni,size(Br_star,2))], [eye(nx);zeros(ni+nx,nx)];
         Cr_tilde*Xt,  zeros(ny,size(Xt,2)+(nu*(nbDeriv+1))),   zeros(ny,size(Br_star,2)),   zeros(ny,nx)];
     
lyap_positive=(Xt >= 1e-6*eye(size(Xt,1)));
lyap_bounded=(Xt <= 1e4*eye(size(Xt,1)));

coeff_iss=(omega >= 1e-6*eye(size(omega)));
% coeff_iss=(coeff_iss + (gamma >= 1e-6*eye(size(omega))));

 matrix11=blkdiag(-Xt,-beta*Xt,-beta*eye(ne-nx),-omega*eye(ny),-gamma*eye(nx));
      
iss_control=[matrix11, matrix21';
           matrix21,   [-Xt,                zeros(nx+nx+ni,ny);
                     zeros(ny,nx+nx+ni),     -alpha*eye(ny)]];
     
iss_cond=(iss_control<=-1e-6*eye(size(iss_control)));

%% Bound controller gain
lambda=1e9;
fBounded=([-lambda*Xt  F';F  -eye(nu)]<=-1e-4);

%% solve LMI
lmiConstraints=lyap_positive+lyap_bounded+iss_cond+coeff_iss+fBounded;

options = helper.changeSDPsettings;
sol=optimize(lmiConstraints,omega,options);
if sol.problem
    error('[%s] %s: %s',mfilename, 'LMI not solved', yalmiperror(sol.problem));
else
    disp('+----------------------------------------+');
    disp('Tracking design results');
    disp('LMI solved with:');
    fprintf('       alpha (decay rate)     = %1.2d \n',value(alpha));
    fprintf('       beta  (observer error) = %1.2d \n',value(beta));
    fprintf('       gamma (modeling error) = %1.2d \n',value(gamma));
    fprintf('       omega (tracking error) = %1.2d \n',value(omega));    
    disp('+----------------------------------------+');
    Le=value(F)/value(Xt);
    L= Le(:,1:nx);
    L_star = Le(:,nx+1:nx+nx)-L;
    Li= Le(:,nx+nx+1:end);
    if ~all(abs(eig(Ar_d-Br_d*L))<1)  
        error('[%s] %s',mfilename,'Closed-loop system unstable');
    end
    %% pseudo-comparison between open-loop and closed-loop
    % (do not take into account modeling errors nor integral term in the controller)
    SysRed_cl=ss([SysRed_d.a-SysRed_d.b*L, -SysRed_d.b*L_star;zeros(nx), sys_star.a],...
                    [zeros(nx,ny);sys_star.b], [SysRed_d.c, zeros(ny,nx)],0,dt);
    step(sys_star,SysRed_d,SysRed_cl);
    legend
end

end