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

function finalClosedLoopValidation(SimulationData,SysLarge_d,SysRed_d,DataRed,f,Sys_star,obs,SysE_d, doPlots)
% FINALCLOSEDLOOPVALIDATION simulates the closed loop system with feedback
% and observer
%
% See also : controller.computeController, stateObserver.computeObserver,
% SoftRobot_class

if nargin < 8
    error('Not enough input arguments.');
elseif nargin < 9
    warning('[%s] %s', mfilename, 'doPlots input variable not set, default value = 0');
    doPlots=0;
end

%% init
freeIndx=SimulationData.freeIndx;
dt=SimulationData.dt;
u0=SimulationData.u0(freeIndx);
eq=SimulationData.eqP(freeIndx);
v_state=zeros(numel(u0),1);
q_state=u0'-eq';

[Ar_d,Br_d,Cr_d,~]=ssdata(SysRed_d);
[A_star,B_star,C_star,~]=ssdata(Sys_star);
[Ae_obs,Be_obs,Ce_obs,~]=ssdata(SysE_d);
[A,B,C,~]=ssdata(SysLarge_d);

n = size(A,1);
nr = size(Ar_d,1);
nu = size(Br_d,2);
ny = size(Cr_d,1);

x=[v_state;q_state];
xr=DataRed.W'*x;
x_star=0*xr;
y_star=C_star*x_star;

L=f.L;
Li=f.Li;
L_star=f.L_star;
Lu=1;

xeObs=zeros(size(SysE_d.a,1),1);
% or if real initial conditions are used:
% xeObs(1:nr)=xr;
xrObs=xeObs(1:nr);
yeObs=Ce_obs*xeObs;

xi=zeros(ny,1);

%% init
nbStep=10;

forceInputs=zeros(nu,1);
ref=zeros(ny,nbStep);
t = linspace(dt,nbStep*dt,nbStep);
%ref(1,:) = 30*cos(2*pi*0.05*t);

ValuesToInit=struct('y',ny,'yr',ny,'xr',nr,'forceInputs',nu,'errT',nr,'errO',nr,'xrObs',nr,'x_star',nr,'y_star',ny,'xproj',nr,'err_y',ny);
SavedVectors=helper.initSavedVectors( nbStep,  ValuesToInit );
ValuesToUpdate=struct('y',C*x,'yr',Cr_d*xr,'xr',xr,'forceInputs',forceInputs,'errT',xr-x_star,'errO',xr-xrObs,'xrObs',xrObs,'x_star',x_star,'y_star',y_star,'xproj',xr,'err_y',C*x-C_star*x_star);
SavedVectors=helper.updateSavedVectors(SavedVectors, 0, ValuesToUpdate);

%% simu
for indx=1:nbStep
    %%% reference model
    x_star=A_star*x_star+B_star*ref(:,indx);
    y_star=C_star*x_star;
    
    %%% large state space :
    x=A*x+B*forceInputs;
    q_state=x(end/2+1:end);
    
    %%% reduced state space
    xr=Ar_d*xr+Br_d*forceInputs;
    x_proj=DataRed.V*xr;
    q_proj=x_proj(end/2+1:end);
    errTraj=xr-x_star;
    
    y=C*x;
    yr=Cr_d*xr;
    
    %% extended observer
    xeObs=Ae_obs*xeObs+obs*(y(:)-yeObs(:))+Be_obs*forceInputs;
    yeObs=Ce_obs*xeObs;
    xrObs=xeObs(1:nr);
    du_hat=xeObs(nr+1:nr+size(Br_d,2));
    
    %% integrator
    xi = xi +y-y_star;
    
    %% feedback
    errObs=x_star-xrObs;
    forceInputs= -L*xr-L_star*x_star;%-Li*xi-Lu*du_hat;
    
    %% plots
    if doPlots
        figure(1)
        clf;
        subplot(2,1,1);
        plot3(eq(1:3:end)+q_proj(1:3:end)',eq(2:3:end)+q_proj(2:3:end)',eq(3:3:end)+q_proj(3:3:end)','x');
        title(['Reduced state space, step '  num2str(indx),'/' num2str(nbStep)])
        %axis([0 50 -50 20 0 150])    % trunk's dimensions
        subplot(2,1,2);
        plot3(eq(1:3:end)+q_state(1:3:end)',eq(2:3:end)+q_state(2:3:end)',eq(3:3:end)+q_state(3:3:end)','x');
        title(['Full state space, step '  num2str(indx),'/' num2str(nbStep)])
        %axis([0 50 -50 20 0 150])    % trunk's dimensions
        drawnow
        pause(0.1)
    end
    
ValuesToUpdate=struct('y',y,'yr',yr,'xr',xr,'forceInputs',forceInputs,'errT',...
    xr-x_star,'errO',xr-xrObs,'xrObs',xrObs,'x_star',x_star,'y_star',y_star,...
    'xproj',xr,'err_y',y-y_star,'yObs',yeObs,'duHat',du_hat);
    SavedVectors=helper.updateSavedVectors(SavedVectors, indx, ValuesToUpdate);
    
end

vectorT = 0:dt:nbStep*dt;

figure(2);
clf;
% the 2 outputs should be equals, if reduction is well done
subplot(2,1,1);
stairs(vectorT,SavedVectors.yr');
title('Reduced state output Cr * xr')
subplot(2,1,2);
stairs(vectorT,SavedVectors.y');
title('full state output C * x')

figure(3);
clf;
subplot(4,1,1);
stairs(vectorT,SavedVectors.y_star');
title('y*, reference state')
subplot(4,1,2);
stairs(vectorT,SavedVectors.x_star');
title('x*, reference state')
subplot(4,1,3);
stairs(vectorT,SavedVectors.xr');
title('xr')
subplot(4,1,4);
stairs(vectorT,SavedVectors.errT');
title('x* - xr')

figure(4)
clf;
for indx=1:size(xr)
    subplot(numel(xr)/(numel(xr)/2),numel(xr)/2,indx);
    stairs(vectorT,SavedVectors.xproj(indx,:)','b');
    hold on
    stairs(vectorT,SavedVectors.xrObs(indx,:)','r--');
    legend('xr','xrobs')
end

figure(5);
clf;
subplot(3,1,1);
stairs(vectorT,SavedVectors.forceInputs');
title('forceInput')
subplot(3,1,2);
stairs(vectorT,(-L*SavedVectors.xr)');
title('-L*xr')
subplot(3,1,3);
stairs(vectorT,(-L*SavedVectors.xrObs)');
title('L*xrObs')

figure(6);
clf;
for i=1:ny
    subplot(ny,1,i)
stairs(vectorT,SavedVectors.y(i,:)');
hold on
stairs(vectorT,SavedVectors.y_star(i,:)');
stairs(vectorT,SavedVectors.yObs(i,:)');
stairs(vectorT,[0 ref(i,:)]');
legend('y','y star','y obs','ref')
title(strcat('output ',num2str(i)));
end

figure(7);
clf;
stairs(vectorT,SavedVectors.duHat');
title('du hat')

figure(8);
clf;
stairs(vectorT,SavedVectors.x_star','--');
hold on
stairs(vectorT,SavedVectors.xr');

end