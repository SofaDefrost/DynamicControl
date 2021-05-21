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



'''  SIMUINIT : initialize all variables required in the pythonScriptController used to : 	
	 save data to construct a model (modelling.py)											
     save snapshots (snapshots.py) 															
     test closed-loop control (closedLoop.py) 												
     depending on the simulation, different initialization are required.

Details of the different case :
 	modelling.py : 	save all data needed in matlab to construct a model of the studied robot
	snapshots.py : 	shake the robot and save snapshots of position and velocity to compute POD in matlab
 	closedLoop.py: 	check controller designed in matlab 
'''

import numpy as np
import os
from tools import computeStateVector, getEqPoint 	#only used if simuToRun='closedloop'

print "*********** Warning: simu init **********"
print "This file supposes that the corresponding Sofa scene is made such that the component have specific names."
print "*********** End warning *********"

def default(self):
## Default initialization, needed by all pythonScriptController
	# self.solver is used in saveSimulationData to get rayleigh damping coefficient
	self.solver = self.node.getChild(self.mainName).getObject("odesolver")
	# self.mechaObject is used in computeStateVector to get position and velocity vectors
	self.mechaObject=self.node.getChild(self.mainName).getObject('meca')						

	self.actuators=[]
	if self.nbActuators>0:
		for i in range(0,self.nbActuators):
			self.actuators.append(self.node.getChild(self.mainName).getChild('constraintPoints').getObject("actuator"+str(i)))			
		self.actuationType=str(self.actuators[0].valueType)

	

	# Number of nodes of the mesh
	U=np.matrix(self.mechaObject.findData("position").value)
	self.numNodes=len(U) 		# Number of nodes of the mesh

	self.constraintDOF=self.node.getChild(self.mainName).getObject('RestShapeSpringsForceField').findData("points").value 

	# indices of free DOF
	self.freeDOF=range(self.numNodes)			# self.FreeDOF = [1 2 3 4 5...NumNodes]
	for elem in self.constraintDOF:	
		self.freeDOF.remove(elem[0])		# self.FreeDOF = self.FreeDOF without constraintDOF	
	
	self.t=0

	self.node.findData("animate").value=1
	return self



def modelling(self):
	# default initialization
		self=default(self)
	# then init specific to modelling simu:
		# delete existing MDKmatrix.txt file if it exists
		try:
			os.remove(self.pathToData+"/MDKmatrix"+self.modelName+".txt")
		except OSError:
			pass
		# mass matrix to be saved in saveSimulationData
		if not self.isUsingBeamAdapter:
			diagMassList = self.node.getChild(self.mainName).getObject("DiagonalMass").findData('vertexMass').value
			self.diagMass = [item for sublist in diagMassList for item in sublist]
		# else = if BeamAdapterIsUsed
		# 	Then mass is saved directly in saveSimulationData
		
		## set equilibrium point of robot equal to zero
		## eqPoint is used in compute state vector
		numConstraints=len(self.constraintDOF)
		self.eqPoint=np.zeros((3*(self.numNodes-numConstraints),1))
		self.eqPointComplete=np.zeros((3*(self.numNodes),1))
		return self


def closedLoop(self):
	# default initialization
		self=default(self)
	# then init specific to closedLoop simu:
		self.forceInputs=np.zeros((self.nbActuators,1))
		self.dispInputs=np.zeros((self.nbActuators,1))
		self.savedVectors=dict()	# empty dictionary
		self.r=self.Ar.shape[0] 	# reduced size	
		self.eqPoint, self.eqPointComplete=getEqPoint.main(self.modelName,self.pathToData)
		x, u, v=computeStateVector.main(self, self.eqPoint, 1)
		xr = self.T.transpose()*x
		ny=self.Cr.shape[0]
		self.y=np.zeros((ny,1))

		## init observer
		if self.computeObs:
			self.yeObs=np.zeros((ny,1))	
			ne=self.Ae.shape[0] 				# extended size
			z=np.zeros((ne-self.r,1))
			# to set initial condition xrObs(t=0)=xr(t=0), set realInitCond=1, else = 0 and observer starts at 0
			realInitCond=1
			self.xrObs_Omega=realInitCond*np.concatenate((xr,z),axis=0)	# extended state : [observed state, disturbance]

		if self.tracking:
			self.ref = np.matrix([0,0,0]).transpose() # np.zeros((ny,1))
			self.x_star = np.zeros((self.r,1))

		return self
		

def openLoop(self):
	# default initialization
		self = default(self)
	# then init specific to openLoop simu:
		self.savedVectors=dict()	# empty dictionary
		self.eqPoint, self.eqPointComplete = getEqPoint.main(self.modelName,self.pathToData)
		return self