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

function [SysLarge_d, SysLarge_c, SimulationData, SysMatrices, modelIsBuilt]=buildModel(modelName, env_var, launchSofa, useCurrentTextFiles, useSavedMatFile)
%BUILDMODEL Returns simulation data, system matrices, and state-space models
%
%  Input args:
%               modelName : name of studied robot
%               env_var : environment variables containing path to sofa
%                           scenes and python scripts
%               launchSofa : [optional] boolean, if true a sofa simulation
%                               is launched to saved new system matrices
%               useCurrentTextFiles : [optional] boolean, if true model is
%                               built from existing text files from previous sofa simulation
%               useSavedMatFile : [optional] boolean, if true model is
%                               loaded from existing mat file 
%               Warning : Only one boolean can be true at a time.
%                           If no option is given as input (nargin = 2), 
%                           then launchSofa = 1, a new sofa simulation is
%                           launched
%
% See also MAIN.SOFTROBOT_CLASS/MODELLING, FULLORDERMODELVALIDATION

if nargin < 2
    error('Not enough input arguments.');
elseif nargin == 2
    warning('[%s], %s',mfilename,'No option set, by default a new sofa simulation is launched.');
        launchSofa = 1;
    useCurrentTextFiles = 0;
    useSavedMatFile = 0;
elseif nargin > 2 && nargin < 5
    warning('[%s], %s', mfilename, sprintf('%s %s', 'Please give values for 3 boolean variables :',...
    'launchSofa, useCurrentTextFiles, useSavedMatFile. Otherwise default behavior = launch new sofa simulation.'));
    launchSofa = 1;
    useCurrentTextFiles = 0;
    useSavedMatFile = 0;
elseif nargin == 5
    if nnz([launchSofa useCurrentTextFiles useSavedMatFile]) > 1
        error('Only one boolean variable can be true at the same time.');
    end    
end

modelIsBuilt=0;     % model is not built, set default value for output variables:
SysLarge_d=[];
SysLarge_c=[];
SimulationData=[];
SysMatrices=[];
pathToSofaData=env_var.data;

%% if .mat file exists in directory, load it ?
if launchSofa
        % Launch Sofa modelling simulation
        sofa.runSofaFromMatlab(modelName,env_var,'modelling');
        disp('---------------------');
        disp('Sofa simulation done.');
        disp('Start modelling...');
        % If is not using beam adapter in sofa simulation, only the
        % MDKmatrix text file is needed to construct model.
        % if sofa simulation is using beam adapter, several files are
        % needed to construct a model.
        [SysLarge_d, SysLarge_c, SimulationData, SysMatrices]=constructModelFromSimulation(modelName,pathToSofaData);
        modelIsBuilt=1;    
end

if useCurrentTextFiles    
    textFilesFromSofaExist=or(checkIfTextFilesExist(pathToSofaData, modelName, 'standard'),...
        checkIfTextFilesExist(pathToSofaData, modelName, 'BeamAdapter')); % if all files are found, construct model ?
    if ~textFilesFromSofaExist
        warning('Required text files are not present. Please relaunch sofa modelling simulation');
        modelIsBuilt = 0;
    else
        disp('Start reading text files from sofa. App will come back after reading files...');
            [SysLarge_d, SysLarge_c, SimulationData, SysMatrices]=constructModelFromSimulation(modelName,pathToSofaData);
            modelIsBuilt=1;
    end
end

if useSavedMatFile
    if exist(strcat('./savedModels/',modelName,'.mat'),'file')==2
        disp('Loading...');
        l=load(strcat(modelName,'.mat'),'SoftRobot');
        SysLarge_d=l.SoftRobot.SysLarge_d;
        SysLarge_c=l.SoftRobot.SysLarge_c;
        SimulationData=l.SoftRobot.SimulationData;
        SysMatrices=l.SoftRobot.SysMatrices;
        modelIsBuilt=1;
    else
        warning('Not mat file found for this robot.');
        modelIsBuilt = 0;
    end 
end

%% Add to SimulationData equilibrium point and initial position used for modeling step
% eqP and u0 = equilibrium point and initial position of robot
% in sofa modelling simulation
SimulationData.u0=load(strcat(env_var.data,'/u0Complete',modelName,'.txt'));
SimulationData.eqP=load(strcat(env_var.data,'/EqPointComplete',modelName,'.txt'));

%% Use following lines to remove outputs for the system
% default output is : displacement along x, y and z
% update outputToStudy for your needs
outputsToStudy=["x","y", "z"];  
[SysLarge_c,SysLarge_d]=helper.updateModelOutputs(SysLarge_c,SysLarge_d,outputsToStudy,mfilename);

%% End modelling, check if succesfull:
if ~modelIsBuilt
    warning('[%s] %s',mfilename,'Modelling not done.');
    SysLarge_d = [];
    SysLarge_c = [];
    SimulationData = [];
    SysMatrices = [];
end

% helper.checkContinuousAndDiscrete(SysLarge_c,SysLarge_d,mfilename);

end


%% local function to be called to construct model from text files
function [SysLarge_d, SysLarge_c, SimulationData, SysMatrices]=constructModelFromSimulation(modelName,pathToSofaData)
% Get model info from Sofa
commonTextFilesFromSofaExist=checkIfTextFilesExist(pathToSofaData, modelName, 'common');
if commonTextFilesFromSofaExist
[SimulationData,SysMatrices]=sofa.readSofaTextFiles(modelName,pathToSofaData);
end
%Construct State Space
SysLarge_d=largeScale.discrete.constructStateSpace(SimulationData,SysMatrices);
SysLarge_c=largeScale.continuous.constructStateSpace(SimulationData,SysMatrices);
end

function textFilesFromSofaExist=checkIfTextFilesExist(pathToSofaData, modelName, opt)
% check if required text files are present to construct a model from sofa
% simulation data
% Depending on the modeling strategy used in SOFA, different text files are
% needed, depending if the scene uses BeamAdapter plugin, ModelReduction
% plugin etc...
switch opt
    case 'common'
    % 'common' = required with all modeling strategies
        actuatorTxtFile=exist(strcat(pathToSofaData,'/indxActuators',modelName,'.txt'),'file');
        nodesTxtFile=exist(strcat(pathToSofaData,'/indxFreeNodes',modelName,'.txt'),'file');
        dataTxtFile=exist(strcat(pathToSofaData,'/SimuData',modelName,'.txt'),'file');
        textFilesFromSofaExist = actuatorTxtFile && nodesTxtFile && dataTxtFile;
    case 'standard' % i.e. not BeamAdapter neither ModelReduction
        matrixTxtFile=exist(strcat(pathToSofaData,'/MDKmatrix',modelName,'.txt'),'file');
        textFilesFromSofaExist = matrixTxtFile && checkIfTextFilesExist(pathToSofaData,modelName, 'common');
    case 'BeamAdapter'
        listOfFiles= dir(fullfile(pathToSofaData,modelName));
        numBeamsFile = 0;
        matrixMFile = 0;
        matrixKFile = 0;
        for i =1 : numel(listOfFiles)
            numBeamsFile = numBeamsFile || contains(listOfFiles(i).name,'numBeams');
            matrixMFile = matrixMFile || contains(listOfFiles(i).name,'matrixM');
            matrixKFile = matrixKFile || contains(listOfFiles(i).name,'matrixK');
        end
        textFilesFromSofaExist = numBeamsFile && matrixMFile && matrixKFile ...
            && checkIfTextFilesExist(pathToSofaData,modelName, 'common');
end
end
