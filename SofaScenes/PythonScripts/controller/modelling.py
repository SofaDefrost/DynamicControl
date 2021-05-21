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
modelling.py is used if input argument of the sofa scene is 'modeling'.
It saves simulation data around an equilibrium point.
The data are saved in "pathToData/modelName/".
pathToData and modelName are defined in tools/parameters.

self.isUsingBeamAdapter is used in saveSimulationData to save the Mass, stiffness and damping matrices. 
If false, the sparseLDLsolver is used to save (M+dt*D+dt*dt*K) in a text file that is then read in Matlab
if true, the mass and stiffness matrices are saved from AdaptiveForceFieldAndMass

isUsingReduction indicates if the sofa scene uses the Model Reduction plugin

Input arguments:

beamObjectList : [optional, used only if BeamAdapter plugin is used]
				  Used to reconstruct mass and stiffness matrix from Beam components
'''

import Sofa
import math
import numpy as np
import sys
import os
import exceptions

import time

from tools import saveSimulationData, computeStateVector, simuInit
from shutil import move

class Modelling(Sofa.PythonScriptController):

	def __init__(self, rootNode, isUsingReduction, isUsingBeamAdapter,nbActuators, nbEffectors, mainName, \
		idxEffector,tFinal, forcesActuators, pathToData, modelName, maxForceActuators, addLoad=0, beamObjectsList=None):
			self.nbActuators=nbActuators
			self.mainName=mainName
			self.nbEffectors=nbEffectors
			self.idxEffector=idxEffector
			self.tFinal=tFinal 	#number of steps of simulation
			self.forcesActuators=forcesActuators
			self.isUsingBeamAdapter=isUsingBeamAdapter
			self.isUsingReduction=isUsingReduction
			self.pathToData=pathToData
			self.modelName=modelName	
			self.maxForceActuators=maxForceActuators
			self.beamObjects = beamObjectsList 			# used only with beamAdapter

	def bwdInitGraph(self, node):
		self.node = node
		self.status = 0
		self=simuInit.modelling(self)

	def onBeginAnimationStep(self,dt):
		if self.nbActuators>0:
			for i in range(0,self.nbActuators):
				self.actuators[i].findData("value").value=self.forcesActuators[i]

		self.t=self.t+dt

		if self.t<2*dt:
			## save equilibrium point 
			Xc,Uc,Vc=computeStateVector.main(self, self.eqPointComplete, 0)
			
			np.savetxt(self.pathToData+"/u0Complete"+self.modelName+".txt",Uc)

		if self.t>(self.tFinal*dt)-dt:
			## save equilibrium point 
			X,U,V=computeStateVector.main(self, self.eqPoint, 1)
			Xc,Uc,Vc=computeStateVector.main(self, self.eqPointComplete, 0)
			
			np.savetxt(self.pathToData+"/EqPoint"+self.modelName+".txt",U)
			np.savetxt(self.pathToData+"/EqPointComplete"+self.modelName+".txt",Uc)

			saveSimulationData.main(self,dt,self.pathToData,self.modelName, \
				self.node.getChild(self.mainName).getObject("preconditioner"),self.mechaObject, self.maxForceActuators, self.beamObjects)

			self.status = moveFile(self)

		if self.t>(self.tFinal*dt) and self.status:
				self.node.findData("animate").value=0
				sys.exit()
		elif self.t>(self.tFinal*dt) and not self.status:
				raise Exception('At least one of the required text files have not been written.')
				


def moveFile(self):
	# copy from scene directory to data directory
	
	if not self.isUsingBeamAdapter:
		status = os.path.isfile(self.pathToData+"/MDKmatrix"+self.modelName+".txt")
		return status
	else:
		status = 1
		for item in self.beamObjects:
			txtFileMatrixK = "matrixK_"+item.getName()+".txt"
			txtFileIndexK = "indexK_"+item.getName()+".txt"
			txtFileMatrixM = "matrixM_"+item.getName()+".txt"
			txtFileIndexM = "indexM_"+item.getName()+".txt"
			if not (os.path.isfile(txtFileMatrixK) and os.path.isfile(txtFileMatrixM) \
				and os.path.isfile(txtFileIndexK) and os.path.isfile(txtFileIndexM)):
				status = 0
			else:
				move(txtFileMatrixK,self.pathToData+"/matrixK_"+item.getName()+"_"+self.modelName+".txt")
				move(txtFileIndexK,self.pathToData+"/indexK_"+item.getName()+"_"+self.modelName+".txt")
				move(txtFileMatrixM,self.pathToData+"/matrixM_"+item.getName()+"_"+self.modelName+".txt")
				move(txtFileIndexM,self.pathToData+"/indexM_"+item.getName()+"_"+self.modelName+".txt")
				status = status and os.path.isfile(self.pathToData+"/matrixK_"+item.getName()+"_"+self.modelName+".txt")
				status = status and os.path.isfile(self.pathToData+"/indexK_"+item.getName()+"_"+self.modelName+".txt")
				status = status and os.path.isfile(self.pathToData+"/matrixM_"+item.getName()+"_"+self.modelName+".txt")
				status = status and os.path.isfile(self.pathToData+"/indexM_"+item.getName()+"_"+self.modelName+".txt")
		return status