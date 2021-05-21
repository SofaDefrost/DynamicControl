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

function [ f, DataCL ] = computeLargeController( method,SysLarge_d)
%
% See also COMPUTECONTROLLER, CONTROLLER.REDLARGE_DAMPINGCONTROL

if nargin < 1
    error('Not enough input arguments.');
end

n = size(SysLarge_d.a,1);
if n > 300
    warning('[%s] %s %s',mfilename, 'This function has not been tested for systems of these dimensions.',...
    'It did not provide any results after 10 hours of computation for a system with 4000 states.');
end 
   

switch method.specific
    case 'systune'
        %%% create tunable system
        G = SysLarge_d;G.u='u';G.y='y';
        nbAct = size(SysLarge_d.b,2);
        D = tunableGain('Decoupler',eye(nbAct));D.u = 'e';D.y = 'p';
        pid_block=cell(nbAct,1);
        for i = 1 :nbAct
            pid_i = tunablePID(sprintf('pid_%s',num2str(i)),'pi',G.Ts);
            pid_i.u = sprintf('p(%c)',num2str(i));
            pid_i.y = sprintf('u(%c)',num2str(i));
            pid_block{i}=pid_i;
        end
        Sum = sumblk('e=r-y',nbAct);
        CLry = connect(G,D,pid_block{1:end},Sum,'r','y');
        %%% tune performances
        ref = TuningGoal.StepTracking('r','y',0.7);
        softReq=ref;
        [CL,~,~] = systune(CLry,softReq);
        viewGoal(softReq,CL);
        f.dec = CL.Blocks.Decoupler.Gain.Value;
        f.pid=cell(nbAct,1);
        for i=1:nbAct
            f.pid{i}.Kp=CL.Blocks.(sprintf('pid_%s',num2str(i))).Kp.Value;
            f.pid{i}.Kd=CL.Blocks.(sprintf('pid_%s',num2str(i))).Kd.Value;
        end
        DataCL.method=method;
        DataCL.launchSofa=1;
    otherwise
        warning('[%s] %s',mfilename, 'Method not implemented yet');
        f=-1;
        DataCL.method='Not implemented';
        DataCL.launchSofa=0;
end
end
