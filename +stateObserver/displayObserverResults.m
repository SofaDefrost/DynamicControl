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

function displayObserverResults(SysRed_d, obs, DataObs)
%DISPLAYOBSERVERRESULTS display observer design results
% takes low order system and oberser data and
% display corresponding properties such as stability etc...
%
% See also COMPUTEOBSERVER

if nargin < 3
    error('Not enough input arguments.');
end

%% This file is written for discrete time systems
% warning('[%s] %s',mfilename,'This file is written for discrete time systems');
if ~SysRed_d.Ts > 0
    error('Reduced system given as input is not a discrete time system');
end

%% Check if controller matrix has correct size
assert(ismatrix(obs),'Controller matrix is not a matrix.');
n1 = size(obs,1);
assert(n1 >= size(SysRed_d.a,1),'Observer matrix has wrong dimension.');
n2 = size(obs,2);
assert(isequal(n2,size(SysRed_d.c,1)),'Observer matrix has wrong dimension.');

%% Compare Eigenvalues of continuous time system
poleOL=eig(SysRed_d.a);
poleObs=DataObs.eig;

assert(numel(poleOL) <= numel(poleObs),'Inconsistent number of poles.');

figure(1);
clf;
plot(real(poleOL),imag(poleOL),'bx','LineWidth',3);
hold on;
plot(real(poleObs),imag(poleObs),'rx','LineWidth',3);
t = linspace(0,2*pi,100);
plot(sin(t),cos(t));
title(sprintf('Eigenvalues of open loop and observer using %s', DataObs.method));
legend('Eig of open loop','Eig of observer');

%%
disp('-----------------------------------');
disp('Open-loop poles versus observer');
fprintf('    eig min open loop = %1.3d\n',min(abs(poleOL)));
fprintf('    eig max open loop = %1.3d\n',max(abs(poleOL)));
disp('Abs value of observer eigenvalues:');
for idx = 1:numel(poleObs)
     fprintf('    %1.3d\n', abs(poleObs(idx)));
end
disp('-----------------------------------');


end