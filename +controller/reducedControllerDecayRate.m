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

function [ f, P ] = reducedControllerDecayRate( SysRed, modelName )
%REDUCEDCONTROLLERDECAYRATE computes feedback matrix for the reduced system
%without any consideration for the large-scale system. It computes a 
%feedback controller f from Linear Matrix Inequalities
% Lyapunov function V=x'*P*x
% Stability with decay rate constraint: dot(V) < -2*alpha*V
% bound of the input : u'*u < lambda*x'*P*x
%
% Input arguments :
%   SysRed : continues-time reduced order model to control.
%   modelName : [optional] name of the studied robot. If no modelName is
%   given, then lambda has a default value, else a specified lambda is applied.
% 
% See also COMPUTECONTROLLER, COMPUTEREDUCEDCONTROLLER

if nargin < 1
    error('Not enough input arguments.');
end

dimR=size(SysRed.a,1);
nbActuators=size(SysRed.b,2);

[Ar,Br,~]=ssdata(SysRed);

X=sdpvar(dimR);               % X=P^-1
Y=sdpvar(nbActuators,dimR,'full');      % Y=K.X

a=-max(real(eig(Ar)))/2;

%% add constraint on control
lambda=0.01; 

%% LMI constraints

systemStable = ((X*Ar'+Ar*X-Br*Y-Y'*Br'+2*a*X)*1e-4<=-1e-4);
lyapunovPositive = (X>=1e-4);
controlBounded=([-lambda*X  Y';Y  -eye(nbActuators)]<=-1e-4);

Cs=systemStable+lyapunovPositive+controlBounded;

options = helper.changeSDPsettings;
sol=optimize(Cs,[],options);


%%
if sol.problem==0
    f = value(Y)*value(X)^(-1) ;
    P= inv(value(X));
    disp('controller decay rate : LMI was solved.');
else
    warning('[%s] %s', mfilename, 'controller decay rate : LMI was not solved.');
    sol.info
    yalmiperror(sol.problem)
    check(Cs)
    f=-1;
end
end