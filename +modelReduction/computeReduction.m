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

function [ SysRed, SysRed_d, DataRed, SysMatrices] = computeReduction(method,order,SysLarge_c,SysLarge_d,SimulationData,SysMatrices,env_var,modelName,launchSofaShaking,useExistingSnapshots)
%COMPUTEREDUCTION Computes reduced order model
%   From the full order model and the desired reduction method, the
%   function returns a reduced order model and associated data.
%
%   Input args:
%               - method : string, can be either POD, ITIA or BT
%               - order  : integer, desired dimension for reduced system
%                           NOTE: not applicable when using BT method
%               - SysLarge_c : continuous time high dimension system
%               - SysLarge_d : discrete time high dimension system
%               - SimulationData
%               - SysMatrices : [optional], only used with POD
%                               struct containing system matrices
%               - env_var : [optional], only used with POD
%                           struct containing environment variables
%               - modelName : [optional], only used with POD
%                             string
%               - launchSofaShaking : [optional], only used with POD
%                       boolean, if true, a sofa simulation is launched to
%                       shake the robot and save snapshots to compute POD.
%               - useExistingSnapshots : [optional], only used with POD
%                       boolean, if true, the POD is computed using existing
%                       snapshots file from a previous sofa shaking simulation.
%                       NOTE : Even if it is checked in the main GUI if required
%                       snapshots file exist to compute POD, it is checked
%                       inside the function again. It can be useful when
%                       one does not use the main GUI.
%
% LaunchSofaShaking and useExistingSnapshots cannot be true at the same time.
%
% The SOFA shaking simulation rely on the Model Reduction plugin. Depending
% on the size of the mesh and the number of actuators, it can be time consuming.
%
% ITIA and Balanced Truncation (BT) algorithms rely on the continuous time system, which can be
% not accurate because of numerical errors from the transfer between SOFA and Maltab
% while the discrete time system is more robust with respect to these numerical errors.
% Therefore before using ITIA, a step is added to check if the continuous system is accurate
%
% See also COMPUTEMOREPOD, COMPUTEMOREITIA, REDUCEDORDERVALIDATION

%% check if input arguments are valid
if nargin < 5
    error('Not enough input arguments.');
elseif nargin == 8 && isequal(method,'POD')
    warning('[%s], %s',mfilename,'No option set, by default a new sofa simulation is launched.');
    launchSofaShaking = 1;
    useExistingSnapshots = 0;
elseif nargin ==9 && isequal(method,'POD')
    warning('[%s], %s', mfilename, sprintf('%s %s', 'Please give values for 2 boolean variables :',...
        'launchSofa and useExistingSnapshots. Otherwise default behavior = launch new sofa simulation.'));
    launchSofaShaking = 1;
    useExistingSnapshots = 0;
elseif nargin == 10 && isequal(method,'POD')
    if nnz([launchSofaShaking useExistingSnapshots]) > 1
        error('Only one boolean variable can be true at the same time.');
    end
    if ~any([launchSofaShaking useExistingSnapshots])
        warning('[%s], %s', mfilename, sprintf('%s %s', 'If LaunchSofa = 0 and UseExistingSnapshots = 0, this functions does nothing.'));
        [ SysRed, SysRed_d, SysMatrices] = deal([],[],[]);
        DataRed.status = 0;
        return;
    end
end

helper.checkInputArguments({method, order, SysLarge_c, SysLarge_d, SimulationData},...
    {@ischar, @isnumeric, @isobject, @isobject, @isstruct});

%% main function
switch method
    case 'POD'
        if launchSofaShaking
            fprintf('[%s] %s\n',mfilename,'Processing sofa simulation...Shaking robot...');
            sofa.launchPODinSOFA(modelName, env_var, SimulationData);
            [SysMatrices.snapshots.v, SysMatrices.snapshots.u] = sofa.readSofaSnapshots(env_var, SimulationData );
            disp('Done : SOFA shaking simulation.');
            disp('-------------------------------');
            disp('Computing POD...');
            [ SysRed,SysRed_d,DataRed ] = modelReduction.computeMorePOD(SysLarge_c,SysLarge_d,SysMatrices.snapshots,SimulationData,order/2);
            DataRed.method=method;
            DataRed.status=1;
        end        
        if useExistingSnapshots
            areSnapshotsFilesPresent = exist(fullfile(env_var.data,'/reduction/debug/stateFile.state'),'file') && ...
                exist(fullfile(env_var.data,'/reduction/debug/velocityStateFile.state'),'file');
            if ~areSnapshotsFilesPresent
                error('No snapshots file exist. Please launch a sofa simulation to save snapshots.');
            else
                [SysMatrices.snapshots.v, SysMatrices.snapshots.u] = sofa.readSofaSnapshots(env_var, SimulationData);
                [SysRed, SysRed_d, DataRed] = modelReduction.computeMorePOD(SysLarge_c,SysLarge_d,SysMatrices.snapshots,SimulationData,order/2);
                DataRed.method=method;
                DataRed.status=1;
            end
        end        
        
    case 'ITIA'
        % ITIA requires the MORE toolbox
        if ~exist('mor.lti','file')
            error('The MORE toolbox is not installed, you can not use ITIA method.');
        end
        % Before using ITIA, check if the continuous system is accurate
        status = helper.checkContinuousAndDiscrete(SysLarge_c,SysLarge_d);
        if status % if continuous time system is accurate
            [ SysRed ,DataRed] = modelReduction.computeMoreITIA(SysLarge_c,order);
            SysRed_d=c2d(SysRed,SimulationData.dt);
            DataRed.method=method;
            DataRed.status=1;
        else
            error('Continuous time system might not be accurate enough to use ITIA algorithm. Please use another reduction method.');
        end
    case 'BT'
        % Before BT, check if the continuous system is accurate
        status = helper.checkContinuousAndDiscrete(SysLarge_c,SysLarge_d, mfilename);
        if status % if continuous time system is accurate
            warning('[%s]: %s %s', mfilename,'reduced order SoftRobot.r not used for Balanced Truncation.',...
                'Size defined by tol (default = 1e-6)');
            disp('BT reduction launched.');
            nRed=1;  % used just to start the while loop
            tol=1e-6;
            while ~mod(nRed,2)==0        % reduced size has to be an even number
                [SysBal,G,T,Ti] = balreal(SysLarge_c);
                tol=tol/1.5;
                SysRed=modred(SysBal,G<tol);
                nRed=size(SysRed.a,1);
            end
            fprintf('Reduced order= %d \n', nRed);
            DataRed.W=T(1:size(SysRed.a,1),:)';
            DataRed.Wbar=T(size(SysRed.a,1)+1:end,:)';
            DataRed.V=Ti(1:size(SysRed.a,1),:)';
            DataRed.Vbar=Ti(size(SysRed.a,1)+1:end,:)';
            SysRed_d=c2d(SysRed,SimulationData.dt);
            DataRed.method=method;
            DataRed.status=1;
        else
            error('Continuous time system might not be accurate enough to use BT algorithm. Please use another reduction method.');
        end
    otherwise
        error('Not implemented yet : available reduction methods : POD, ITIA and BT');
end

% check reduction results : successfull is DataRed.status = 1
if exist('DataRed','var') && isequal(DataRed.status,1)
    fprintf('\n[%s] Done: %s\n',mfilename,method);
    disp('--------------------');
else
    error('Something wrent wrong during the computation the reduced order model.');
end

end