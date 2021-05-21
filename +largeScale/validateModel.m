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

function validateModel(SoftRobot,displayModelData,launchSimulation)
% VALIDATEMODEL checks if modelling step was succesfull.
%
%       Input arguments:
%           SoftRobot : an object of type SoftRobot_class for which the
%                       model should be tested.
%           displayModelData: [optional] boolean, if true plot modelling
%                           results. Default = 1.
%           launchSimulation: [optional] boolean, if true, launch
%                               modelling simulation. Default = 0.
%
% See also BUILDMODEL, FULLORDERMODELVALIDATION

if nargin < 1
    error('Not enough input arguments.');
elseif nargin ==1
    displayModelData=1;
    launchSimulation=0;
elseif nargin == 2
    launchSimulation=0;
end

if SoftRobot.SimulationData.numNodes>600
    f=uifigure;
    msg='High number of nodes in the mesh, this operation will be time consuming. Continue?';
    answer = uiconfirm(f,msg,'Confirm',...
        'Icon','warning');
    close(f);
else
    answer='OK';
end

if isequal(answer,'OK')
    
    if displayModelData
        largeScale.displayModelData(SoftRobot.SysMatrices,SoftRobot.SimulationData,...
            SoftRobot.SysLarge_d,SoftRobot.SysLarge_c);
    end
    if launchSimulation
        largeScale.fullOrderModelValidation(SoftRobot.SimulationData,SoftRobot.SysMatrices,...
            SoftRobot.SysLarge_d,1);
        %largeScale.discrete.sofaImplicitEuler(SoftRobot.SimulationData,SoftRobot.SysMatrices,SoftRobot.SysLarge_d,1)
    end
    
end

end

