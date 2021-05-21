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

function damping_wrt_eigs(SysMatrices,SimulationData)
%%% DAMPING_WRT_EIGS plot damping parameter with respect to the eigenvalues
%%% of the model of the studied soft robots

%% init
K=-SysMatrices.compliance;    
M=SysMatrices.mass;    

%% eigenvalues computation
% The eigenvalues of M^(-1)K are positive but the computation of eigenvalues
%of large scale matrices may be inaccurate
[~,VP,flag]=eigs(M\K,size(M,1));         

if flag==0          %if the computation has converged    
VP=sort(diag(VP));

% If the computation has converged but is not consistent, i.e. if some
% eigenvalues are not positive, remove them, send a warning and plot the
% results without the non-positive eigenvalues
if any(VP<0)
    VP(VP<0)=[];
    warning('MyComponent:IncorrectType','Problem with eigenvalue computation.\nSome eigenvalues are not positive.');
end

alpha=SimulationData.rayMass;
beta=SimulationData.rayStiffness;
Ak=zeros(SimulationData.numNodes*6,SimulationData.numNodes*6);
indxi=1;indxj=1;
for i=1:length(VP)
    Bki=[-alpha-beta*VP(i) -VP(i);1 0];
    Ak(indxi:indxi+1,indxj:indxj+1)=Bki;        % Ak = Tridiag matrice de A dans nouvelle base    
    indxi=indxi+2;
    indxj=indxj+2;
end

ksi=zeros(1,length(VP));
for i=1:length(VP)
    ksi(i)=(0.5)*((alpha/sqrt(VP(i)))+beta*sqrt(VP(i)));
end

% general shape of the curve
% lambda=linspace(0.001,10000,500);
% Allure=zeros(1,length(lambda));
% for i=1:length(lambda)
%     Allure(i)=0.5*((alpha/sqrt(lambda(i)))+beta*sqrt(lambda(i)));
% end

figure(2)
clf;
plot(VP,ksi,'LineWidth',5)
set(gca,'FontSize',20);
title('\xi(\lambda)','FontSize',40);
xlabel('\lambda','FontSize',40);
ylabel('\xi','FontSize',40);
set(gca,'YScale','log');
set(gca,'XScale','log');

disp('Min Eigenvalues =')
min(VP)
disp('Min Damping =')
min(ksi)
else
    error('Problem with eigenvalue computation')
end
end

