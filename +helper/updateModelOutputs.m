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

function [newSysLarge_c,newSysLarge_d] = updateModelOutputs(SysLarge_c, SysLarge_d,outputsToStudy,mainfilename)
%UPDATEMODELOUTPUTS takes as input the alrge-scale models and updates the
% number of outputs depending on the value of the input field
% 'outputsToStudy'
% Example:
%           intput argument outputsToStudy equals
%               ["x"] : consider only one output along x direction
%               ["x", "y"] : consider only two outputs along x and y directions
%               ["z", "y"] : consider only two outputs along z and y directions
%               ["x", "x", "x", "x"] : consider outputs along x directions 4 times

if ~isequal(outputsToStudy,["x","y","z"])
    nbOutputs = numel(outputsToStudy);
    disp('------------')
    warning('[%s] %s \n%s%s',mainfilename,'Non-standard number of outputs.','This model considers outputs along ',sprintf(['%s ', repmat('and %s ',1,nbOutputs-1)],outputsToStudy));
    disp('------------')
    outputsToKeep = zeros(nbOutputs,1);
    for i=1:nbOutputs
        if outputsToStudy(i)=="x"
            outputsToKeep(i)=1;
        elseif outputsToStudy(i)=="y"
            outputsToKeep(i)=2;
        elseif outputsToStudy(i)=="z"
            outputsToKeep(i)=3;
        end
    end
    newSysLarge_c=ss(SysLarge_c.a,SysLarge_c.b,SysLarge_c.c(outputsToKeep,:),SysLarge_c.d(outputsToKeep,:));
    newSysLarge_d=ss(SysLarge_d.a,SysLarge_d.b,SysLarge_d.c(outputsToKeep,:),SysLarge_d.d(outputsToKeep,:),SysLarge_d.Ts);
else
    newSysLarge_c = SysLarge_c;
    newSysLarge_d = SysLarge_d;
end
    

end