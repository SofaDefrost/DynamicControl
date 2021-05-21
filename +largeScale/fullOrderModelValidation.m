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

function fullOrderModelValidation(SimulationData,SysMatrices,SysLarge_d, plotResults)
%FULLORDERMODELVALIDATION simulates large-scale systems.
%Compares two discrete large scale systems :
% 'sofa reference' simulation using euler implicit scheme
% a large-scale discrete state space constructed based on this euler scheme
%
% A validation of the continuous time system might be done through the
% 'reducedOrderValidation' function. It compares the full and low order
% state space models. Using ITIA, the low order system is computed thanks
% to the continuous time large-scale model. So, comparing the large-scale
% discrete time model and the low order from ITIA is a fair comparison.
%
% Attention: Difference of amplitude in actuators forces in Matlab and Sofa
% If in sofa actuator force = 1, then in matlab it must be equal to 1/dt
%
% To define the linearization point, sofa saves the position of the robot
% in a text file (saved in SimulationData.eqP).
% This position corresponds to a gravity and an actuation force.
%
% See also BUILDMODEL, LARGESCALE.DISCRETE.SOFAIMPLICITEULER

if nargin < 3
    error('Not enough input arguments.');
elseif nargin < 4
    plotResults=0;
end

freeIndx=SimulationData.freeIndx;
dt=SimulationData.dt;
u0=SimulationData.u0;
eq=SimulationData.eqP;
eq_free=SimulationData.eqP(freeIndx);
u0_free=SimulationData.u0(freeIndx);
isUsingReduction=SimulationData.isUsingReduction;

if ~isUsingReduction
    constraintNodes=1:3*SimulationData.numNodes;
else
    constraintNodes=1:SimulationData.numNodes;
end

constraintNodes(freeIndx)=[];

M=SysMatrices.mass;
K=SysMatrices.stiffness;
D=SysMatrices.damping;

A_euler=M+dt*D+(dt^2)*K;    % left side euler scheme

A_d=SysLarge_d.a;
C=SysLarge_d.c;

q_euler=u0-eq;
v_euler=zeros(size(q_euler));

q_state=u0_free-eq_free;
v_state=zeros(size(q_state));

% use (:) to force vectors to be column vectors
eq=eq(:);eq_free=eq_free(:);
q_euler=q_euler(:);v_euler=v_euler(:);q_state=q_state(:);v_state=v_state(:);

x=[v_state;q_state];

saveY_state(:,1)=C*x;
saveY_euler(:,1)=C*[v_euler(freeIndx);q_euler(freeIndx)];

%% init
total=50;

forceInputs=zeros(size(SysLarge_d.b,2),1);       % lambda = actuators contributions around the equilibrium point
% the change of equilibrium point is done by changing SimulationData.eqP


%% simu
figure(1);
clf;
for i=1:total
    %%% SOFA reference = implicit euler :
    controlInput_euler=SysMatrices.input*forceInputs;
    controlInput_euler(controlInput_euler<0)=0;
    dv=A_euler\(dt*(-K*q_euler-dt*K*v_euler-D*v_euler+controlInput_euler));
    v_euler=v_euler+dv;
    v_euler(constraintNodes)=0;
    q_euler=q_euler+dt*v_euler;
    q_euler(constraintNodes)=0;
    
    %%% large state space :
    controlInput_ss=SysLarge_d.b*forceInputs;
    controlInput_ss(controlInput_ss<0)=0;
    x=A_d*x+controlInput_ss;
    q_state1=x(end/2+1:end);
    %%% plots
    if plotResults
        figure(1);
        clf;
        title(i)
        subplot(2,1,1);
        if ~isUsingReduction
            plot3(eq_free(1:3:end)+q_state1(1:3:end),eq_free(2:3:end)+q_state1(2:3:end),eq_free(3:3:end)+q_state1(3:3:end),'x');
        else
            plot(eq_free+q_state1,'x');
        end
        title(['Discrete Large state space, step '  num2str(i),'/' num2str(total)])
        % axis([0 50 -50 20 0 150])    % trunk's dimensions
        subplot(2,1,2);
        if ~isUsingReduction
            plot3(eq(1:3:end)+q_euler(1:3:end),eq(2:3:end)+q_euler(2:3:end),eq(3:3:end)+q_euler(3:3:end),'x');
        else
            plot(eq+q_euler,'x');
        end
        hold on
        if ~isUsingReduction
            plot3(eq(constraintNodes(1:3:end))+q_euler(constraintNodes(1:3:end)),eq(constraintNodes(2:3:end))+q_euler(constraintNodes(2:3:end)),eq(constraintNodes(3:3:end))+q_euler(constraintNodes(3:3:end)),'rx');
        end
        title(['SOFA reference : euler implicit step '  num2str(i),'/' num2str(total)])
        %axis([0 50 -50 20 0 150])    % trunk's dimensions
        xlabel('x axis');
        ylabel('y axis');
        zlabel('z axis');
        drawnow
        pause(0.1)
    end
    
    saveY_state(:,i+1)=C*x;
    saveY_euler(:,i+1)=C*[v_euler(freeIndx);q_euler(freeIndx)];
end

vectorT = 0:dt:total*dt;

figure(3);
clf;
subplot(2,1,1)
stairs(vectorT,saveY_state');
title('full state output')
subplot(2,1,2)
stairs(vectorT,saveY_euler');
title('Sofa reference : euler implicit')

end