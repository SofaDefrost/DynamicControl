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

function controlValidation(SimulationData,SysLarge_d,SysRed_d,DataRed,f,doPlots)
%CONTROLVALIDATION simulates closed loop experiment
% call : control.controlValidation(SimulationData,SysMatrices,SysLarge_d,SysRed_d,DataRed,f,1)
%
% To use integral action, uncomment the related line
%
% Example
%       SoftRobot=controller(SoftRobot);
%       control.validateController(SoftRobot);  % this function calls
%                                                 CONTROLVALIDATION
%
% See also COMPUTECONTROLLER, SOFTROBOT_CLASS.CONTROLLER

if nargin < 5
    error('Not enough input arguments.');
elseif nargin < 6
    warning('[%s] %s', mfilename, 'doPlots input variable not set, default value = 0');
    doPlots=0;
end

%% check stability

%%
freeIndx=SimulationData.freeIndx;
dt=SimulationData.dt;

u0=SimulationData.u0(freeIndx);
eq=SimulationData.eqP(freeIndx);

v_state=zeros(numel(freeIndx),1);
q_state=u0'-eq';

A_d=SysLarge_d.a;
Ar_d=SysRed_d.a;
Br_d=SysRed_d.b;

if abs(eig(Ar_d-Br_d*f.L))>1
    error('Closed-loop system unstable.');
end

C=SysLarge_d.c;
Cr=SysRed_d.c;

x=[v_state;q_state];
x_proj=DataRed.W'*x;
xr=x_proj;

%% init
total=20;

saveY_state=zeros(size(C,1),total+1);
saveYr=zeros(size(C,1),total+1);
saveY_state(:,1)=C*x;
saveYr(:,1)=Cr*xr;

% force inputs
u=zeros(size(SysLarge_d.b,2),1);

saveLambda=zeros(size(SysLarge_d.b,2),total+1);

%% simu
for i=1:total    
    %%% large state space :
    x=A_d*x+SysLarge_d.b*u;
    q_state=x(end/2+1:end);
    
    %%% reduced state space
    xr=Ar_d*xr+Br_d*u;
    x_proj=DataRed.V*xr;
    qr_proj=x_proj(end/2+1:end);
    
    u=-f.L*xr;       % without integral action
    saveLambda(:,i+1)=u;
    
    %%% plots
    if doPlots==1
        figure(1)
        clf;
        subplot(2,1,1);
        plot3(eq(1:3:end)+qr_proj(1:3:end)',eq(2:3:end)+qr_proj(2:3:end)',eq(3:3:end)+qr_proj(3:3:end)','x');
        title(['Reduced state space, step '  num2str(i),'/' num2str(total)])
        subplot(2,1,2);
        plot3(eq(1:3:end)+q_state(1:3:end)',eq(2:3:end)+q_state(2:3:end)',eq(3:3:end)+q_state(3:3:end)','x');
        title(['full state space, step '  num2str(i),'/' num2str(total)])        
        drawnow
        pause(0.1)
    end
    
    saveY_state(:,i+1)=C*x;
    saveYr(:,i+1)=Cr*xr;
end

figure
subplot(2,1,1)
stairs(saveYr');
title('reduced state output')
subplot(2,1,2)
stairs(saveY_state');
title('full state output')

figure
stairs(dt:dt:(total+1)*dt,saveLambda');
title('Lambda')
end
