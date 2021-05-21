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

function [ f, DataCL ] = computeRedLargeController( SysRed_c,SysRed_d,DataRed,SysMatrices,SimulationData,modelName)
%COMPUTEREDLARGECONTROLLER Computes controller based on reduced order system
%   and aims at providing properties for the large-scale system.
%   It returns a reduced order feedback matrix F in u= - Fx_r and
%   DataCL = closed loop data, such that the performances and the stability
%   of the large-scale model are guaranteed.
%
% Input arguments:
%   SysRed_c : continuous time low order system
%   SysRed_d : discrete time low order system
%   DataRed : structure containing model reduction data, such as the
%       projectors.
%   SysMatrices : structure containing mass, stiffness, damping and input
%       matrices.
%   SimulationData : structure containing the data of sofa simulation, such
%       as the time step.
%   modelName: string representing the name of the robot studied.
%
% See also COMPUTECONTROLLER, CONTROLLER.REDLARGE_DAMPINGCONTROL

if nargin < 5
    modelName=[];
end

warning('[%s] %s %s %s',mfilename,'For now, only one method is implement for this',...
'methodology. It is implemented in the function CONTROLLER.REDLARGE_DAMPINGCONTROL.',...
'It is written for continuous time systems. TODO: Write it for discrete time !');

try
    [ f, lambda] = controller.redLarge_dampingControl( SysMatrices,DataRed,SimulationData,modelName);
    DataCL.lambda=lambda;
    if f==-1    %if no error but LMI not sovled
        DataCL.error="LMI not solved";
        DataCL.launchSofa=0;
    else
        DataCL.eigCL=eig(SysRed_c.A-SysRed_c.b*f);
        DataCL.launchSofa=1;
        isPolesLocationCorrect(SysRed_c.a,SysRed_c.b,f,'continuous')
    end
catch ME
    f=-1;
    DataCL.error=ME.message;
end



end

% throw an error if closed-loop is unstable
function isPolesLocationCorrect(A,B,f,method)
p=eig(A-B*f);
if isequal(method,'discrete')
    if any(abs(p)>1)
        warning('[%s] %s', mfilename, 'Closed-loop unstable');
    end
else
    if any(real(p)>0)
        warning('[%s] %s', mfilename, 'Closed-loop unstable');
    end
end
end
