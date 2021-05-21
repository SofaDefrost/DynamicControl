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

function [ SysRed ,info] = computeMoreITIA(SysLarge,r)
%COMPUTEMOREITIA Uses MORE toolbox to compute reduced state space
%model using Iterative Tangential Interpolation Algorithm (ITIA)
% For more information, see <a href="http://w3.onera.fr/more/">MORE Toolbox</a>.
% 
%   See also COMPUTEREDUCTION, COMPUTEMOREPOD

if nargin < 2
    error('Not enough input arguments.');
end

%% Order reduction with the MORE toolbox

clear opt
opt.checkH2 = 1; % check H2 optimality at each iterations
opt.ensureStab=1;
opt.extraInfo=1;
opt.verbose=1;

if size(SysLarge.a,1)>100
    [SysRed, info] = mor.lti({sparse(SysLarge.a),sparse(SysLarge.b),sparse(SysLarge.c),sparse(SysLarge.d)},r,opt);
else
    [SysRed, info] = mor.lti(SysLarge,r,opt);
end

end
