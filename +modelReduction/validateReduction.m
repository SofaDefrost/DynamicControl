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

function validateReduction(SoftRobot,plotReductionResults,launchSimulation)
% VALIDATEREDUCTION checks if reduction step was succesfull.
%
% See also REDUCTION.COMPUTEREDUCTION, REDUCTION.REDUCEDORDERVALIDATION

if SoftRobot.DataRed.status         % if reduction step is done
    if isequal(SoftRobot.DataRed.method,'noReduction')
        warning('[%s] %s', mfilename, 'You will not be able to compute a controller nor observer without reduction');
    end
   if plotReductionResults 
        modelReduction.displayReductionResults(SoftRobot.SysRed_c,SoftRobot.SysRed_d,...
            SoftRobot.SysLarge_c, SoftRobot.SysLarge_d, SoftRobot.DataRed);
   end
   if launchSimulation
        modelReduction.reducedOrderValidation(SoftRobot.SimulationData,...
            SoftRobot.SysLarge_d,SoftRobot.SysRed_d,SoftRobot.DataRed,1);
    end
else
    error('Problem with reduction, DataRed.launchSofa should be equal to 1.');
end
end