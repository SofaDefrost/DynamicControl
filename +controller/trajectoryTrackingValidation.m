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

function save_Lyap= trajectoryTrackingValidation(SimulationData,SysLarge_d,SysRed_d,DataRed,Sys_star,L,L_star,SysE_d,doPlots)
%TRAJECTORYTRACKINGVALIDATION simulates closed loop experiment
%
% Example
%
% Input arguments:
%
% See also COMPUTECONTROLLER, SOFTROBOT_CLASS.CONTROLLER

if nargin < 4
    error('Not enough input arguments.');
elseif nargin < 5
    warning('[%s] %s', mfilename, 'doPlots input variable not set, default value = 0');
    doPlots=0;
end

freeIndx=SimulationData.freeIndx;
u0=SimulationData.u0(freeIndx);
eq=SimulationData.eqP(freeIndx);

[Ar,Br,Cr,~]=ssdata(SysRed_d);
[A_star,B_star,C_star,~]=ssdata(Sys_star);
[A,B,C,~]=ssdata(SysLarge_d);

n = size(A,1);
nr = size(Ar,1);
nu = size(Br,2);
ny = size(Cr,1);

v_state=zeros(numel(u0),1);
q_state=u0'-eq';
x=[v_state;q_state];
xr=DataRed.W'*x;      
x_star=0*xr;

%% init
total=50;

saveY_star=zeros(ny,total+1);
saveX_star=zeros(nr,total+1);
save_ErrT=zeros(nr,total+1);
saveY=zeros(ny,total+1);
saveYr=zeros(ny,total+1);
save_normX=zeros(1,total+1);
save_normXr=zeros(1,total+1);

saveU(:,1)=zeros(nu,1);
saveY_star(:,1)=C_star*x_star;
saveX_star(:,1)=x_star;
save_ErrT(:,1)=x_star-xr;
saveY(:,1)=C*x;
saveYr(:,1)=Cr*xr;
save_normX(:,1)=x'*x;
save_normXr(:,1)=xr'*xr;

% force inputs
u=zeros(nu,1);
ref=zeros(ny,1);
ref(1)=10;

%% simu
for i=1:total
    %%% reference model
    x_star=A_star*x_star+B_star*ref;
    y_star=C_star*x_star;
    err_traj=xr-x_star;
    
    %%% large state space :
    x=A*x+B*u;
    q_state=x(end/2+1:end);
    
    %%% reduced state space
    xr=Ar*xr+Br*u;
    x_proj=DataRed.V*xr;
    q_proj=x_proj(end/2+1:end);
    
    y=C*x;
    yr=Cr*xr;
    u = -L*xr-L_star*x_star;
    
    %%% plots
    if doPlots==1
        figure(1)
        clf;
        subplot(2,1,1);
        plot3(eq(1:3:end)+q_proj(1:3:end)',eq(2:3:end)+q_proj(2:3:end)',eq(3:3:end)+q_proj(3:3:end)','x');
        title(['Reduced state space, step '  num2str(i),'/' num2str(total)])    
        %axis([0 50 -50 20 0 150])    % trunk's dimensions
        subplot(2,1,2);
        plot3(eq(1:3:end)+q_state(1:3:end)',eq(2:3:end)+q_state(2:3:end)',eq(3:3:end)+q_state(3:3:end)','x');
        title(['Full state space, step '  num2str(i),'/' num2str(total)]) 
        %axis([0 50 -50 20 0 150])    % trunk's dimensions        
        drawnow
        pause(0.1)
    end
    
saveU(:,i+1)=u;
saveY_star(:,i+1)=y_star;
saveX_star(:,i+1)=x_star;
save_ErrT(:,i+1)=x_star-xr;
    saveY(:,i+1)=y;
    saveYr(:,i+1)=yr;
    save_normX(:,i+1)=x'*x;
    save_normXr(:,i+1)=xr'*xr;
end

figure(2);
clf;
% the 2 outputs should be equals, if reduction is well done
subplot(2,1,1);
stairs(saveYr');
title('Reduced state output Cr * xr')
subplot(2,1,2);
stairs(saveY');
title('full state output C * x')

figure(3);
clf;
subplot(3,1,1);
stairs(saveY_star');
title('y*, reference state')
subplot(3,1,2);
stairs(saveX_star');
title('x*, reference state')
subplot(3,1,3);
stairs(save_ErrT');
title('x* - xr')

figure(4);
clf;
stairs(saveU');
title('Actuation input')


end
