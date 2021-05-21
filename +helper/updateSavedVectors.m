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

function [SavedVectors] = updateSavedVectors(SavedVectors, indx, ValuesToUpdate)
% UPDATESAVEDVECTORS takes initialized saved vectors in struct SavedVectors
% and update current values (at time given by indx value).
% Values whose name are given in input structure "ValuesToUpdate" are
% updated.
% Input arguments:
%       SavedVectors: old values of SavedVectors
%       indx : indx of vector to be saved
%       ValuesToUpdate : struct containing pair argument, name of the vector
%       to save and its dimension
%
% Example:
% Initialize vectors to be saved: y of dimension ny, and x of dimension nx
% ValuesToInit=struct('y',ny,'x',nx); 
% SavedVectors=helper.initSavedVectors( nbStep,  ValuesToInit );
%
% Then, update the values of saved vectors:
% for indx = 1:10
%       ValuesToUpdate=struct('y',C*x,'x',x);
%       SavedVectors=helper.updateSavedVectors(SavedVectors, indx, ValuesToUpdate);
% end
%
% See also INITSAVEDVECTORS, LARGESCALE.FINALCLOSEDLOOPVALIDATION

if nargin < 3
    error('Not enough input arguments.');
end

fields=fieldnames(ValuesToUpdate);
for i=1:numel(fields)
    SavedVectors.(fields{i})(:,indx+1)=ValuesToUpdate.(fields{i});
end

end