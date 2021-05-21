#!/usr/bin/env python
# -*- coding: utf-8 -*-

###########################################################################
#                         Matlab toolbox for SOFA                         #
#                                                                         #
#				                                                          # 
# This plugin is distributed under the GNU LGPL (Lesser General           #
# Public License) license with the same conditions than SOFA.             #
#                                                                         #
# Contributors: Defrost team  (INRIA, University of Lille, CNRS,          #
#               Ecole Centrale de Lille)                                  #
#                                                                         #
# Contact information: https://project.inria.fr/softrobot/contact/        #
#                                                                         #
###########################################################################


'''
computeStateVector returns the state vetor of a LTI system.
It returns x=[v;d], where v and d are the velocity and displacement vectors.
Velocity and Position vectors are read from the mechanical object, displacement is obtained 
using self.eqPoint, i.e. d=u-self.eqPoint
Input argument:
	- self
	- eqPoint : position vector where the model has been linearized
	- buildConstraintState : boolean, if true, consider only the degrees of freedom that are free, i.e. not attached.
									  if false, the state vector is made of position and velocity of all the nodes of the mesh.

The value of buildConstraintState must match the size of eqPoint. 

Get Position and Velocity vectors U and V
Both vecotrs are of type list, convert it to matrix of dimension (numberOfNodes,3)
The feedback controller requires a state vector, need to rearrange these matrices from
V=[Vx_node0 Vy_node0 Vz_node0
    Vx_node1 Vy_node1 Vz_node1
    ... ]		
to a vector V=[Vx_node0 Vy_node0 Vz_node0 Vx_node1 Vy_node1 Vz_node1 Vx_node2... ]

'''

import numpy as np

def main(self, eqPoint, buildConstraintState):
		v=np.matrix(self.mechaObject.findData("velocity").value)		
		u=np.matrix(self.mechaObject.findData("position").value)

		numberOfNodes=self.numNodes  

		if buildConstraintState == 1:	
			u=u[self.freeDOF]				# Consider only free degree of freedom
			v=v[self.freeDOF]
			numberOfNodes=len(self.freeDOF)

		u0=[]
		v0=[]
		i=0
		while i<numberOfNodes:
			for j in range(0,3):
				u0.append(u[i,j])					
				v0.append(v[i,j])
			i+=1

		if buildConstraintState:
			u = np.matrix(u0)-self.eqPoint.transpose()				#U = [Ux(node0) Uy(node0) Uz(node0) ...]
		else:
			u = np.matrix(u0)-self.eqPointComplete.transpose()				#U = [Ux(node0) Uy(node0) Uz(node0) ...]

		v = np.matrix(v0)											#V = [Vx(node0) Vy(node0) Vz(node0) ...]

		x=np.concatenate((v, u), axis=1).transpose()		#state vector

		return x, u , v