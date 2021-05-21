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

function sys_star_d = buildReferenceModel(nx,ny,dt,SysRed)
% Reference model defined as: one subsystem for each output
% one input for each output
% all output are decoupled
% Attention : cannot have more outputs than reduced states
%
% If SysRed is given as fourth input argument, then the reference model is
% built from the open-loop eigenvalues.


if ny>0 && ny<= nx && ~isequal(nx/ny,floor(nx/ny))
     error('Reference model requires the number of outputs to be a multiple of the reduced system dimensions.');
end

if nargin < 3
    error('Not enough input arguments.');
elseif nargin < 4
    poles_cl = -1:-1:-nx/ny;
elseif nargin ==4
    poles_ol = eig(SysRed);
    poles_cl = unique(real(poles_ol)*1.5);
    poles_cl = poles_cl(1:nx/ny);
end

ones_up=ones(nx/ny-1,1);
A_tmp=diag(ones_up,1)+diag(poles_cl);
A_star=A_tmp;
for i=2:ny
    A_star=blkdiag(A_star,A_tmp);
end
B_star=zeros(nx,ny);
rowB_star=nx:-nx/ny:nx/ny;
for i=1:numel(rowB_star)
    B_star(rowB_star(i),rowB_star(i)*(ny/nx))=1;
end
C_star=(fliplr(B_star'));
sys_star=ss(A_star,B_star,C_star,0);

% ensure dcgain = 1 for reference model
sys_star=ss(A_star,B_star*inv(dcgain(sys_star)),C_star,0);
sys_star_d=c2d(sys_star,dt);

end

