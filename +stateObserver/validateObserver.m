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

function validateObserver(SoftRobot,plotObserverResults)
% VALIDATEOBSERVER checks if observer design was succesfull.
%
% Examples
%       Input arguments:
%           SoftRobot : an object of type SoftRobot_class for which the
%                       controller should be tested.
%           plotObserverResults: [optional] boolean, if true plot observer
%           results. Default = 1
%
% See also COMPUTEOBSERVER, DISPLAYOBSERVERRESULTS, controller.VALIDATECONTROLLER

if nargin < 1
    error('Not enough input arguments.');
elseif nargin == 1
    plotObserverResults=1;
end

if ismatrix(SoftRobot.obs)
    if plotObserverResults
        stateObserver.displayObserverResults(SoftRobot.SysRed_d, SoftRobot.obs, SoftRobot.DataObs);
    end
    % TODO : simulation to test observer
else
    error('Cannot test observer, it has not been computed.');
end
end