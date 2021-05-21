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

function options = changeSDPsettings()
%CHANGESDPSETTINGS sets options for LMI problems (e.g. what solver is used
%etc...)

%% Default 
options=sdpsettings;

%% change solver
% solver = 'sedumi'; % sedumi ...
% options.solver = solver;

%% debug mode
% debug = 0 ;    % boolean
% options.debug = debug;

%% verbose mode
% verbose = 0 ;    % boolean
% options.verbose = verbose;

%% save duals
% saveduals = 0 ;    % boolean
% options.saveduals = saveduals;

%% showprogress
% showprogress = 0 ;    % boolean
% options.showprogress = showprogress;

end

