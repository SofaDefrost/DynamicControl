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

function status = checkContinuousAndDiscrete(SysLarge_c, SysLarge_d, mainFilename)
% from SOFA, it might happen that the discrete time system is accurate,
% while the continuous time system is not.
% ITIA algorithm does not handle discrete time system, so first step is to
% check if the continusous time system is accurate
%
% Output = status, equals to 1 if there is no difference between
% discrete and continuous time systems.
% Otherwise, equals 0

warning('[%s] %s\n%s\n%s',mfilename,'This file checks the accuracy of both continuous and discrete-time models.',...
    'The computation of the eigenvalues can be time consuming.',...
    sprintf('Comment the call to this function in %s to save time.',mainFilename));

Ads=sparse(SysLarge_d.a);
Acs=sparse(SysLarge_c.a);

e_d=eigs(Ads,size(Ads,1));
e_c=eigs(Acs,size(Acs,1));

% NB: convergence problem using
%e_d=eigs(Ads,1,'largestabs');
%e_c=eigs(Acs,1,'largestreal');

discreteStable = (max(abs(e_d)) < 1);
continuousStable = (max(real(e_c))< 0 );

status = 1;

if discreteStable ~= continuousStable
    warning('[%s] %s', mfilename, 'Continuous and Discrete time systems give opposite results for stability analysis');
    status = 0;
end
end