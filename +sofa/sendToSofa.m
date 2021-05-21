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

function sendToSofa(SoftRobot,SofaFolder,sendFeedback,sendObs,sendForceToDisp, sendFull)
%SENDTOSOFA send matrices needed for sofa simulation
% SOFA closed loop simulation requires :
% F, T, observerGain, C, Ar, Br, Cr;, Ae

%% check if input arguments are valid 
if nargin < 2
    error('Not enough input arguments.');
elseif nargin ==2
    sendFeedback=0;
    sendObs=0;
    sendForceToDisp=0;
elseif nargin == 3
    sendObs=0;
    sendForceToDisp=0;
elseif nargin==4
    sendForceToDisp=0;
end

helper.checkInputArguments({SoftRobot,SofaFolder},...
                           {@isobject, @isstruct});
               
                       
%% main functions
[Ar,Br,Cr]=ssdata(SoftRobot.SysRed_d);
T=SoftRobot.DataRed.W;
C=SoftRobot.SysLarge_c.c;

save(fullfile(SofaFolder.data,'defaultMatrices.mat'),'Ar','Br','Cr','T','C');

success=exist(fullfile(SofaFolder.data,'defaultMatrices.mat'),'file');

if sendFeedback
    if isequal(SoftRobot.methodControl.general,'reducedOrderControl') ...
            && isequal(SoftRobot.methodControl.specific,'traj_track')
        L=SoftRobot.f.L;
        L_star=SoftRobot.f.L_star;
        Li=SoftRobot.f.Li;
        save(fullfile(SofaFolder.data,'feedback.mat'),'L','Li','L_star');
        success=success && exist(fullfile(SofaFolder.data,'feedback.mat'),'file');
        [A_star,B_star,C_star] = ssdata(SoftRobot.Sys_star);
        save(fullfile(SofaFolder.data,'trackingMatrices.mat'),'A_star','B_star','C_star');
        success=success && exist(fullfile(SofaFolder.data,'trackingMatrices.mat'),'file');
    else
        L=SoftRobot.f.L;
        save(fullfile(SofaFolder.data,'feedback.mat'),'L');
        success=success && exist(fullfile(SofaFolder.data,'feedback.mat'),'file');
    end
end

if sendObs
    [Ae,Be,Ce]=ssdata(SoftRobot.SysE_d);
    obs = SoftRobot.obs;
    save(fullfile(SofaFolder.data,'obsMatrices.mat'),'Ae','Be','Ce','obs');
    success=success && exist(fullfile(SofaFolder.data,'obsMatrices.mat'),'file');
end

if sendForceToDisp
    freeIndx=SoftRobot.SimulationData.freeIndx;
    rm=SoftRobot.SimulationData.rayMass;        % rayleigh mass
    rk=SoftRobot.SimulationData.rayStiffness;   % rayleigh stiffness
    M=SoftRobot.SysMatrices.mass(freeIndx,freeIndx);
    K=SoftRobot.SysMatrices.stiffness(freeIndx,freeIndx);
    D=0*(-rm*M+rk*K);       % D = damping of force field but we use only solver damping
    dt=SoftRobot.SimulationData.dt;
    Aeuler=((1+dt*rm)*M-dt*D-dt*(dt+rk)*K);
    invAeuler=sparse(Aeuler\eye(size(Aeuler)));
    H=sparse(SoftRobot.SysMatrices.input(freeIndx,:)');
    
    t1=T(1:end/2,:);
    M1=H*invAeuler*K*t1;
    
    t2=T(end/2+1:end,:);
    M2=H*invAeuler*K*t2;
    
    M3=H*invAeuler*M*t2;
    
    M4=full(H*invAeuler*H');
    save(strcat(SofaFolder.data,SoftRobot.name,'/','convertForceToDisp.mat'),'M1','M2','M3','M4');
    
    success= success && exist(fullfile(SofaFolder.data,SoftRobot.name,'convertForceToDisp.mat'),'file');
end

if sendFull
    [A,B,C]=ssdata(SoftRobot.SysLarge_d);
    save(fullfile(SofaFolder.data,'fullMatrices.mat'),'A','B','C');
    success=success && exist(fullfile(SofaFolder.data,'fullMatrices.mat'),'file');
end

if ~success
    error('Something wrent wrong while writing the files');
else
    disp('Matrices written in SimulationData directory.');
end

end

%#ok<*NASGU>
%#ok<*ASGLU>