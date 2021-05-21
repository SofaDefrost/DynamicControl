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

function [ f, DataCL, Sys_star ] = computeReducedController( method,SysRed,SysRed_d,modelName,SysE_d, Ko)
%COMPUTEREDUCEDCONTROLLER Computes controller based only on the reduced
%   order system. Returns feedback matrix F in u= - Fx_r and DataCL =
%   closed loop data.
% Different methods are available for this methodology. Input arguments:
%   method : (string) method to be used to compute controller, can be:
%               - 'traj_track' : work in progress
%               - 'decayRate' : controller is computed by setting a decay rate
%               for the reduced order system. For continuous-time systems
%               only.
%               - 'polePlacement' : controller is computed by placing poles
%               of reduced system at desired locations. For discrete-time
%               system only.
%   SysRed : (dynamical system) reduced order system.
%   modelName : (string, optional) name of the studied model. Some
%       functions uses this data to tune the performances of the
%       controller.
%
% See also COMPUTECONTROLLER, VALIDATECONTROLLER

if nargin< 2
    error('Not enough input arguments.');
elseif nargin < 4
    modelName=[];
end

if isequal(method.specific,'traj_track') && (nargin < 6 || isempty (SysE_d) || isempty(Ko) || ~ismatrix(Ko))
    warning('[%s] %s',mfilename,'Trajectory tracking requires extended observer system and observer gain.');
    f=-1;
    Sys_star=-1;
    DataCL.msg='Missing data for trajectory tracking design.';
    DataCL.launchSofa=0;
    return
end

Sys_star=[];    % Sys_star is computed only if method = trajectory tracking


%% Compute reduced controller
switch method.specific
    case 'traj_track'
        [ L, L_star, Sys_star, Li ] = controller.tracking_controller(SysRed_d,SysE_d,SysRed);
        isClosedLoopStable = controller.tracking_stability(Sys_star,SysRed_d,SysE_d,Ko, L, L_star);
        if ~isClosedLoopStable
            error('Trajectory tracking design leads to unstable system');
        else
            f.L = L;
            f.Li = Li;
            f.L_star = L_star;
            DataCL.method=method;
            DataCL.launchSofa=1;
            DataCL.eigCL=eig(SysRed_d.A-SysRed_d.b*L);
            isPolesLocationCorrect(SysRed_d.a,SysRed_d.b,L,'discrete')
        end
    case 'polePlacement'
        eig_ol=eig(SysRed_d.a);     % open-loop poles
        [ L ] = place(SysRed_d.a,SysRed_d.b,abs(eig_ol)/1.1);
        DataCL.method=method;
        DataCL.eigCL=eig(SysRed.A-SysRed.b*L);
        DataCL.launchSofa=1;
        isPolesLocationCorrect(SysRed_d.a,SysRed_d.b,L,'discrete')
        f.L=L;
    case 'decayRate'
        [ L, P] = controller.reducedControllerDecayRate( SysRed,modelName );
        DataCL.Lyap=P;
        DataCL.method=method;
        DataCL.eigCL=eig(SysRed.A-SysRed.b*L);
        DataCL.launchSofa=1;
        isPolesLocationCorrect(SysRed.a,SysRed.b,L,'continuous')
        f.L=L;
    case 'systune'
        nx=size(SysRed_d.a,1);
        ny=size(SysRed_d.c,1);
        nu=size(SysRed_d.b,2);
        dt=SysRed_d.Ts;
        
        G = SysRed_d;G.u='u';G.y='y';
        D = tunableGain('Decoupler',eye(nu));D.u = 'e';D.y = 'p';
        pid_block=cell(nu,1);
        for i = 1 :nu
            pid_i = tunablePID(sprintf('pid_%s',num2str(i)),'pi',G.Ts);
            pid_i.u = sprintf('p(%c)',num2str(i));
            pid_i.y = sprintf('u(%c)',num2str(i));
            pid_block{i}=pid_i;
        end
        Sum = sumblk('e=r-y',nu);
        CLry = connect(G,D,pid_block{1:end},Sum,'r','y');
        
        %%% tune performances
        sys_star=controller.buildReferenceModel(nx,ny,dt,SysRed);
        ref = TuningGoal.StepTracking('r','y',sys_star); 
        softReq=ref;
        [CL,~,~] = systune(CLry,softReq);
        viewGoal(softReq,CL);
        % isPolesLocationCorrect(CL.a,CL,L,'discrete')
               
        step(G,CL,sys_star)
        legend
    otherwise
        warning('[%s] %s',mfilename, 'Method not implemented yet');
        f=-1;
        DataCL.method='Not implemented';
        DataCL.launchSofa=0;
end
end

% throw an error if closed-loop is unstable
function isPolesLocationCorrect(A,B,f,method)
p=eig(A-B*f);
if isequal(method,'discrete')
    if any(abs(p)>1)
        error('[%s] %s', mfilename, 'Closed-loop unstable');
    end
else
    if any(real(p)>0)
        error('[%s] %s', mfilename, 'Closed-loop unstable');
    end
end
end
