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

function [e_d,e_c]=displayModelData(SysMatrices,SimulationData,SysLarge_d,SysLarge_c)
% change function name, not only display

fprintf('\n--------- Model Data ---------\n');
fprintf('Modelling simulation with sampled time dt = %.2d. \n',SimulationData.dt);

Ads=sparse(SysLarge_d.a);
Acs=sparse(SysLarge_c.a);
 
e_d=eigs(Ads,size(Ads,1));
e_c=eigs(Acs,size(Acs,1));


fprintf('Largest absolute value of eigenvalues (discrete-time) : %f.\n', max(abs(e_d)));
fprintf('Smallest absolute value of eigenvalues(discrete-time) : %f.\n', min(abs(e_d)));

fprintf('Largest real part of eigenvalues (continuous-time) : %f.\n', max(real(e_c)));
fprintf('Smallest real part of eigenvalues (continuous-time) : %f.\n', min(real(e_c)));

disp('------------------------------');

figure(1);
clf;
spy(SysMatrices.mass);
title('Mass matrix');

figure(2)
clf;
spy(SysMatrices.stiffness);
title('Stiffness matrix');

end

