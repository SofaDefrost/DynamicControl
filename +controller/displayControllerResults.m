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

function displayControllerResults(SysRed_d, f, DataCL)
%DISPLAYCONTROLLERRESULTS takes discrete-time low order system and closed-loop data and
%display corresponding properties such as stability etc...
%
% See also computeController, VALIDATECONTROLLER, SoftRobot_class.controller

if nargin < 3
    error('Not enough input arguments.');
end

%% This file is written for discrete time systems
% warning('[%s] %s',mfilename,'This file is written for discrete time systems');
if ~SysRed_d.Ts > 0
    error('Reduced system given as input is not a discrete time system');
end

%% Check if controller matrix has correct size
assert(ismatrix(f.L),'Controller matrix is not a matrix.');
n2 = size(f.L,2);
assert(isequal(n2,size(SysRed_d.a,1)),'Controller matrix has wrong dimension.');
n1 = size(f.L,1);
if ~isequal(DataCL.method.specific,'traj_track')
    assert(isequal(n1,size(SysRed_d.b,2)),'Controller matrix has wrong dimension.');
else
    % for trajectory tracking, controller includes reference system.
    assert(n1 >= size(SysRed_d.b,2),'Controller matrix has wrong dimension.');
end

%% Compare Eigenvalues of continuous time system
poleOL=eig(SysRed_d.a);
poleCL=eig(SysRed_d.a-SysRed_d.b*f.L);

assert(isequal(numel(poleOL),numel(poleCL)),'Number of poles of closed loop and open loop must be equal.');

figure(1);
clf;
plot(real(poleOL),imag(poleOL),'bx','LineWidth',3);
hold on;
plot(real(poleCL),imag(poleCL),'rx','LineWidth',3);
t = linspace(0,2*pi,100);
plot(sin(t),cos(t));
title(sprintf('Eigenvalues of open and closed loop using %s, %s', DataCL.method.general, DataCL.method.specific));
legend('Eig of open loop','Eig of closed loop');

%%
disp('-----------------------------------');
disp('Open-loop poles versus closed-loop');
disp('(discrete-time)');
disp('Absolute value of:');
disp('Open-loop | Closed-loop');
for idx = 1:numel(poleOL)
    fprintf('%1.3d | %1.3d\n',abs(poleOL(idx)), abs(poleCL(idx)));
end
disp('-----------------------------------');


end