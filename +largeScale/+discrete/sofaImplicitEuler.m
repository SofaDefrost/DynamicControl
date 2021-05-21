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

function Vid=sofaImplicitEuler(SimulationData,SysMatrices,SysLarge_d,doPlots)
%SOFAIMPLICITEULER reproduces the simulation of sofa
% call : Vid=largeScale.discrete.sofaImplicitEuler(SimulationData,SysMatrices,SysLarge_d,1)
% v=VideoWriter('test.mp4','MPEG-4')
% open(v)
% writeVideo(v,Vid)
% close(v)

numNodes=SimulationData.numNodes;
freeIndx=SimulationData.freeIndx;

u0=SimulationData.u0;
eqP=SimulationData.eqP;

noeudsContraint=1:3*numNodes;
noeudsContraint(freeIndx)=[];

rm=SimulationData.rayMass;        % rayleigh mass
rk=SimulationData.rayStiffness;   % rayleigh stiffness

M=SysMatrices.mass;
K=SysMatrices.compliance;

D=0*(-rm*M+rk*K);       % D = damping of force field but we use only solver damping

dt=SimulationData.dt;

A=((1+dt*rm)*M-dt*D-dt*(dt+rk)*K);

v=zeros(size(K,1),1);
q=u0'-eqP';

total=50;
saveV=zeros(size(K,1),total+1);
saveU=zeros(size(K,1),total+1);
saveV(:,1)=v;
saveU(:,1)=q;

lambda=zeros(size(SysLarge_d.b,2),1);

figure(1)
clf;
Vid=getframe;
for i=1:total
    % euler scheme
    dv=A\(dt*(K*q+(dt+rk)*K*v+D*v-rm*M*v+SysMatrices.input*lambda));
    v=v+dv;
    v(noeudsContraint)=0;
    q=q+dt*v;
    q(noeudsContraint)=0;
    %%% plots
    if doPlots==1
    clf;
    plot3(eqP(1:3:end)+q(1:3:end)',eqP(2:3:end)+q(2:3:end)',eqP(3:3:end)+q(3:3:end)','x');
    hold on
    plot3(eqP(noeudsContraint(1:3:end))+q(noeudsContraint(1:3:end))',eqP(noeudsContraint(2:3:end))+q(noeudsContraint(2:3:end))',eqP(noeudsContraint(3:3:end))+q(noeudsContraint(3:3:end))','rx');
    %Vid(i)=getframe(gcf);
    axis([0 1000 -50 50])
    drawnow
    pause(0.1)
    end
    
    saveV(:,i+1)=v;
    saveU(:,i+1)=q;
end

figure(2)
clf,
plot(1:total+1,SysLarge_d.c(:,:)*[saveV(freeIndx,:);saveU(freeIndx,:)])

end
