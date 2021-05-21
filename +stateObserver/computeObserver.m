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

function [ obs, DataObs,SysE_d ] = computeObserver( method,SysRed_d,SimulationData)
%COMPUTEOBSERVER Computes reduced order observer
%   returns observer matrix
%
%   SysLarge,SysRed and SysRed_d are only modified is
%   method='polePlacement', the number of outputs is limited to 1.
%
%   See also UIO_ISS, DISPLAYOBSERVERRESULTS, VALIDATEOBSERVER

if nargin < 2
    error('Not enough input arguments.');
end


switch method
    case 'extendedObserver'
        nbDeriv=1;
        [ obs , DataObs , SysE_d] = stateObserver.uio_iss(SysRed_d,nbDeriv,SimulationData);
        DataObs.launchSofa=1;
        DataObs.method=method;
    otherwise
        disp('not implemented yet');
        obs=-1;
        DataObs.method='Not implemented';
        DataObs.launchSofa=0;
end

