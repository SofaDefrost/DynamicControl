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

function [ f,lambda ] = redLarge_dampingControl( SysMatrices,DataRed,SimulationData,modelName)
% DAMPINGCONTROL Computes feedback controller f.
% from Linear Matrix Inequality based on a
% Lyapunov function = "extended energy" of the system
% The feedback law is u=-Fv*vr-Fd*dr
% vr and dr being the reduced velocity and reduced displacement
% the reduction is done using POD
% returns F=[Fv Fd]

% Tested with a model made of 855 free nodes.
% Memory errors can happen for models with more nodes.
% For 855 nodes, problem was solved after 30 minutes.
%
% See also COMPUTECONTROLLER, VALIDATECONTROLLER

if nargin < 3
    error('Not enough input arguments.');
end

%% init
freeIndx=SimulationData.freeIndx;
nbActuators=SimulationData.nbActuators;
halfDimRed=size(DataRed.V,2)/2;

T=DataRed.V;
Tbar=DataRed.Vbar;

T1=T(1:end/2,1:end/2);
T2=Tbar(1:end/2,1:end/2);

M=SysMatrices.mass(freeIndx,freeIndx);
%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%
%%% TODO : check +/- K
K=SysMatrices.compliance(freeIndx,freeIndx);

rayMass=SimulationData.rayMass;
if rayMass==1
    rayMass=0.9;
    warning('[%s] %s.', mfilename,'rayleigh mass coefficient should not be equal to 1');
end
rayStiff=SimulationData.rayStiffness;
if rayStiff==1
    rayStiff=0.9;
    warning('[%s] %s.', mfilename,'rayleigh stiffness coefficient should not be equal to 1');
end

D=rayMass*M+rayStiff*K;
D11=sparse(T1'*D*T1);
D12=sparse(T1'*D*T2);
D21=sparse(D12');
D22=sparse(T2'*D*T2);

H=M*SysMatrices.input(freeIndx,:);
H1=sparse(T1'*H);
H2=sparse(T2'*H);

M11=sparse(T1'*M*T1);
M12=sparse(T1'*M*T2);
M21=sparse(M12');
M22=sparse(T2'*M*T2);

K11=sparse(T1'*K*T1);
K12=sparse(T1'*K*T2);
K21=sparse(K12');
K22=sparse(T2'*K*T2);

%% Step 0 : Test if energy is a Lyapunov Function
eMax=rayMass/(1-rayMass);

e=(2/3)*eMax;     % pick a e between 0+ (not zero) and eMax

% V should be a Lyapunov function with this value of e :
V=[(1+e)*M, e*M;
    e*M', (1+e)*K+e*D];

dotV=[-2*(1+e)*D+2*e*M-2*e*D, 0*M;
    0*M, -2*e*K];

% first condition: V is positive definite
condV1=all(eigs(V,size(V,1))>=1e-12);
% V is a Lyapunov function if pos def and decreasing
condV2=all(eigs(dotV,size(dotV,1))<=-1e-12);

if condV1~=1
    error('[%s] %s',mfilename,'Lyapunov candidate V is not positive definite.');
elseif condV2 ~=1
    error('[%s] %s',mfilename,'Lyapunov candidate V is not decreasing.');
else        % compute the controller
    projectedDotV=[...
        -(1+e)*D11+e*M11,   0*D11,   -(1+e)*D12+e*M12,  0*D12;
        0*D11,              -e*K11,         0*D12,       -e*K12;
        -(1+e)*D21+e*M21,   0*D21,   -(1+e)*D22+e*M22,  0*D22;
        0*D21,              -e*K21,         0*D22,        -e*K22];
    
    projectedDotV=projectedDotV+projectedDotV';   % make it symmetric
    
    % lambda control the decay rate of the lyapunov function
    % lambda is a negative scalar
    lambda=sdpvar(1,1,'full','real'); % Linear scalar (real, 1 variable)
    lambdaIsNegative=(lambda<=-1e-10);
    
    lambdaV=lambda*[...
        (1+e)*M11,            e*M11,   (1+e)*M12,              e*M12;
        e*M11,     (1+e)*K11+e*D11,       e*M12,   (1+e)*K12+e*D12;
        (1+e)*M21,            e*M21,   (1+e)*M22,              e*M22;
        e*M21,     (1+e)*K21+e*D21,       e*M22,  (1+e)*K22+e*D22];
    
    lambdaV=lambdaV+lambdaV';
    
    % don't use M, D K matrices after this line, clear them from memory
    clearvars -except projectedDotV lambdaV lambdaIsNegative lambda...
        H1 H2 freeIndx e halfDimRed nbActuators epsilon modelName
    
    Fv=sdpvar(nbActuators,halfDimRed,'full','real');
    Fd=sdpvar(nbActuators,halfDimRed,'full','real');
    
    projectedFeedback=[...
        (1+e)*Fv'*H1',   e*Fv'*H1',   (1+e)*Fv'*H2',  e*Fv'*H2';
        (1+e)*Fd'*H1',   e*Fd'*H1',   (1+e)*Fd'*H2',  e*Fd'*H2';
        zeros(2*length(freeIndx)-2*halfDimRed,2*length(freeIndx))];
    
    projectedFeedback=projectedFeedback+projectedFeedback';
    
    % LMI is : dotV+feedback < lambda V, with lambda negative
    % with feedback = u - F x_r
    matrixLMI=projectedDotV-projectedFeedback-lambdaV;
    systemIsStable=(matrixLMI*1e-9<=-eye(size(matrixLMI,1))*1e-6):'Stability full order model'; %#ok<*BDSCA>
    
    %% add constraint on control
    epsilon=50; 
    
    controlIsBounded=[-epsilon*eye(nbActuators), [Fv Fd];
        [Fv';Fd'], -eye(2*halfDimRed)]<=0;
    
    %% Solver
    clearvars -except systemIsStable lambda lambdaIsNegative controlIsBounded Fv Fd modelName
    
    cond=systemIsStable;%+lambdaIsNegative;%+controlIsBounded;
    
    options = helper.changeSDPsettings;
    
    sol=optimize(cond,lambda,options);
    
    if sol.problem==0
        Fv=value(Fv);
        Fd=value(Fd);
        lambda=value(lambda);
        f=[Fv Fd];
        fprintf('[%s] %s', mfilename,'LMI was solved.');
    else
        warning('[%s] %s', mfilename,'LMI was not solved.');
        sol.info
        yalmiperror(sol.problem)
        check(cond)
        f=-1;
        lambda=-1;
    end
    
end
end