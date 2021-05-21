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

function status = checkInputArguments(args, types)
% CHECKINPUTARGUMENTS returns an error if the function input argument does
% not match a specific type.

if nargin < 2 
    error('Not enough input arguments.');
else
    assert(iscell(args) && iscell(types));
end

for i=1:numel(args)
    functionToCheck = types{i};
    argToCheck= args{i};
    functionName = functions(functionToCheck);
    functionName = functionName.function;
    assert(functionToCheck(argToCheck), sprintf('Argument %d is not a %s', i, functionName));
end

% return 1 if no errors
status = 1;
end

