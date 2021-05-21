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

function displayReductionResults(SysRed_c,SysRed_d,SysLarge_c,SysLarge_d,DataRed)
%PLOTREDUCTIONRESULTS plots reduced system properties 
% Compare eigenvalues :
%   Plot all eigenvalues of reduced system
%   Plot 6 first and 6 last eigenvalues of sysLarge
% Display bode plot of the reduced system
%
% See also COMPUTEREDUCTION, VALIDATEREDUCTION

if nargin < 5
    error('Not enough input arguments.');
end

%% Compare Eigenvalues continuous system
poleR_c=eig(SysRed_c.a);
poleL_c=eigs(SysLarge_c.a,size(SysLarge_c.a,1));

figure(1);
clf;
plot(real(poleR_c),imag(poleR_c),'bx','LineWidth',20);    % eigenvalue of red
hold on;
plot(real(poleL_c(1:6)),imag(poleL_c(1:6)),'rx','LineWidth',20);    % 6 first eigenvalue of large
plot(real(poleL_c(end-6:end)),imag(poleL_c(end-6:end)),'gx','LineWidth',20);    % 6 last eigenvalue of large
title(sprintf('Eigenvalues of Large and Reduced system using %s', DataRed.method));
set(gca,'XScale','log');
legend('Eig of Red','First Eigs of Large','Last Eigs of Large');


%% Compare Eigenvalues discrete system
poleR_d=eig(SysRed_d.a);
poleL_d=eigs(SysLarge_d.a,size(SysLarge_d.a,1));

figure(1);
clf;
plot(real(poleR_d),imag(poleR_d),'bx','LineWidth',20);    % eigenvalue of red
hold on;
plot(real(poleL_d(1:6)),imag(poleL_d(1:6)),'rx','LineWidth',20);    % 6 first eigenvalue of large
plot(real(poleL_d(end-6:end)),imag(poleL_d(end-6:end)),'gx','LineWidth',20);    % 6 last eigenvalue of large
title(sprintf('Eigenvalues of Large and Reduced system using %s', DataRed.method));
set(gca,'XScale','log');
legend('Eig of Red','First Eigs of Large','Last Eigs of Large');

%% Bode plots of reduced order system
figure(2);          % magnitude
clf;
h=bodeplot(SysRed_c);
title('Bode (magnitude) plot of reduced system');
setoptions(h,'PhaseVisible','off');

figure(3);          % phase
clf;
i=bodeplot(SysRed_c);
title('Bode (phase) plot of reduced system');
setoptions(i,'MagVisible','off');

%% Controllability and Observabillity of reduced order system
fprintf('\n--------------------------------------------\n');
fprintf('Controllability and observability properties:\n');

if rank(ctrb(SysRed_c.a,SysRed_c.b))==size(SysRed_c.a,1)
    disp('Reduced order system is controllable.');
else
    disp('Reduced order system is not controllable.');
end

if rank(obsv(SysRed_c.a,SysRed_c.c))==size(SysRed_c.a,1)
    disp('Reduced order system is observable.');
else
    disp('Reduced order system is not observable.');
end

fprintf('\n--------------------------------------------\n');

end