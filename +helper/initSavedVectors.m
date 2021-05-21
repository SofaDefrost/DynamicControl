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

function [SavedVectors] = initSavedVectors(nbStep, ValuesToInit)
%INITSAVEDVECTORS initialize vectors to be saved during simulation. After
%initialiation, vectors are updated using UPDATESAVEDVECTORS
% Input arguments:
%       nbStep : number of step of the simulation, is equal to the size of
%       savedVectors
%       ValuesToInit : struct containing pair argument, name of the vector
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
% See also UPDATESAVEDVECTORS, LARGESCALE.FINALCLOSEDLOOPVALIDATION

if nargin < 2
    error('Not enough input arguments.');
end

fields=fieldnames(ValuesToInit);

for i=1:numel(fields)
    SavedVectors.(fields{i})=zeros(ValuesToInit.(fields{i}),nbStep+1);
end

end