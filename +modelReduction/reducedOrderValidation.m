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

function reducedOrderValidation(SimulationData,SysLarge_d,SysRed_d,DataRed,doPlots)
%REDUCEDORDERVALIDATION simulates full and reduced order model and compare
%results
%   The reduced system might be obtained using continuous time model and
%   then discretization. This file can also be used to compare (indirectly)
%   discrete and continuous time systems.
%   In any case, the reduced order sytem is a discrete time system and is
%   compared to the large scale discrete time system.
%
% See also COMPUTEREDUCTION, COMPUTEMOREITIA, COMPUTEMOREPOD

if nargin < 4
    error('Not enough input arguments.');
elseif nargin < 5
    warning('[%s] %s', mfilename, 'doPlots input variable was not set, default value = 0');
    doPlots=0;
end    

dt = SimulationData.dt;
freeIndx=SimulationData.freeIndx;

u0=SimulationData.u0(freeIndx);
eq=SimulationData.eqP(freeIndx);

[A_d, B_d, C_d, ~]=ssdata(SysLarge_d);
[Ar_d, Br_d, Cr_d, ~]=ssdata(SysRed_d);
ny=size(Cr_d,1);
nr=size(Ar_d,1);
nx=size(A_d,1);

v=zeros(nx/2,1);
q=u0'-eq';
x=[v;q];
x_proj=DataRed.W'*x;
xr=x_proj;

%% init
nbStep=50;

forceInputs=zeros(size(SysRed_d.b,2),1);
%forceInputs(1)=4/dt;

% dispInputs = convertForceToDisp(xr, u, DataRed., SysMatrices, SimulationData);

ValuesToInit=struct('y',ny,'yr',ny,'xr',nr);
SavedVectors=helper.initSavedVectors( nbStep,  ValuesToInit );
ValuesToUpdate=struct('y',C_d*x,'yr',Cr_d*xr,'xr',xr);
SavedVectors=helper.updateSavedVectors(SavedVectors, 0, ValuesToUpdate);


%% simu
for i=1:nbStep    
    %%% SOFA reference = implicit euler :
    x=A_d*x+B_d*forceInputs;   
    q=x(end/2+1:end);    
        
    %%% reduced state space
    xr=Ar_d*xr+Br_d*forceInputs;    
    x_proj=DataRed.V*xr;
    q_proj=x_proj(end/2+1:end);
    
    %%% plots
    if doPlots==1
    figure(1)
    clf;
    subplot(2,1,1);
    plot3(eq(1:3:end)+q_proj(1:3:end)',eq(2:3:end)+q_proj(2:3:end)',eq(3:3:end)+q_proj(3:3:end)','x');
    title(['Reduced state space, step '  num2str(i),'/' num2str(nbStep)])
    subplot(2,1,2);
    plot3(eq(1:3:end)+q(1:3:end)',eq(2:3:end)+q(2:3:end)',eq(3:3:end)+q(3:3:end)','x');
    title(['SOFA reference : euler implicit, step '  num2str(i),'/' num2str(nbStep)])
    
    drawnow
    end
    
    ValuesToUpdate=struct('y',C_d*x,'yr',Cr_d*xr,'xr',xr);
    SavedVectors=helper.updateSavedVectors(SavedVectors, i, ValuesToUpdate);
end

figure(2);
clf;
subplot(2,1,1)
stairs(SavedVectors.yr');
title('reduced state output');
subplot(2,1,2)
stairs(SavedVectors.y');
title('Sofa reference : euler implicit')

end