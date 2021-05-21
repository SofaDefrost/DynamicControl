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

function isClosedLoopStable = tracking_stability(Sys_star,SysRed_d,SysE_d,Ke, L, L_star)
%
% Output arguments:
%
% Input arguments:
%
% Examples:
%
% See also ...

%% init
[Ar_star,Br_star,~,~] = ssdata(Sys_star);
[Ar,Br,~,~] = ssdata(SysRed_d);
[Ae,~,Ce,~] = ssdata(SysE_d);
nx=size(Ar,1);
ne=size(Ae,1);
nu=size(Br,2);
nu_star=size(Br_star,2);
ny = size(Ce,1);
nbDeriv=(ne-nx-nu)/nu;

Br_tilde=[Br zeros(nx,nu*nbDeriv)];

G = [Ar-Br*L,   [Br*L, Br_tilde],   Ar-Ar_star-Br*(L+L_star);
    zeros(ne,nx),    Ae-Ke*Ce,           zeros(ne,nx);
    zeros(nx,nx),     zeros(nx,ne),       Ar_star];

Br_tilde_star = [ - Br_star; zeros(ne,nu_star); Br_star];
Phi_tilde = [eye(nx); [eye(nx);zeros(ne-nx,nx)]; zeros(nx)];

ng = size(G,1);

P = sdpvar(ng);
alpha = 0.05;%sdpvar(1);
omega = 1e0;%sdpvar(1);
gamma = 7e0;%sdpvar(1);

%% Define LMI
lyap_pos = P >= 1e-3 * eye(size(P));
lyap_bounded = P <= 1e3 * eye(size(P));

matrix1= [P - alpha * P,         zeros(ng,ny+nx)                  ;
            zeros(ny,ng),       omega*eye(ny),   zeros(ny,nx) ;
            zeros(nx,ng+ny),                          gamma*eye(nx)];
        
lmiMatrix = [G, Br_tilde_star, Phi_tilde]'*P*[G, Br_tilde_star, Phi_tilde] - matrix1;

% coeff_pos=omega >= 1e-6;
% coeff_pos= coeff_pos + (gamma >= 1e-6);

%% solve LMI
lmiConstraints = lyap_pos+ lyap_bounded + (lmiMatrix <= -1e-6 * eye(size(lmiMatrix))); %+ coeff_pos +
% lmiConstraints = lyap_pos + (lmiMatrix <= -1e-6 * eye(size(lmiMatrix)));

options = helper.changeSDPsettings;
sol=optimize(lmiConstraints,[],options);
if sol.problem
    error('[%s] %s %s',mfilename, 'Optimize function failed, error: ',yalmiperror(sol.problem));
else
    disp('-----------------------------------');
    disp('Trajectory tracking design results:');
    disp('       - Trajectory tracking design is stable.');
    fprintf('       - alpha coefficient = %d\n', value(alpha));
    fprintf('       - omega coefficient = %d\n', value(omega));
    fprintf('       - gamma coefficient = %d\n', value(gamma));     
    disp('-----------------------------------');
    isClosedLoopStable = 1;
end

end