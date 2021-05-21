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

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Sofa
import math
import numpy as np
import sys

from tools import computeStateVector, plotVectors, saveVectors, getMatlabMatrices, simuInit, convertForceToDisp

class OpenLoop(Sofa.PythonScriptController):

	def __init__(self, rootNode, nbActuators,mainName,idxEffector,tFinal,forcesActuators, pathToData, modelName, isUsingBeamAdapter):
			self.nbActuators=nbActuators
			self.mainName=mainName
			self.idxEffector=idxEffector
			self.nbStepTotal=tFinal 	#number of steps of simulation
			self.forcesActuators=forcesActuators
			self.pathToData=pathToData
			self.modelName=modelName
			self.isUsingBeamAdapter = isUsingBeamAdapter
			self.needDisp=0
			self.needFull=1

    ############################
	###     initialization   ###
	############################
	def bwdInitGraph(self, node):
			self.node = node

			self=getMatlabMatrices.openLoop(self.pathToData,self)
			self=simuInit.openLoop(self)    	
			for i in range(0,self.nbActuators):
				self.actuators[i].findData("valueType").value="force"

			self.dispInputs=np.zeros((1,self.nbActuators)).transpose()
			self.nbStep = 0

			x, u, v=computeStateVector.main(self, self.eqPoint, 1, self.isUsingBeamAdapter)
			self.x_lin=x
			self.xr_lin=self.T.transpose()*x

			
	def onBeginAnimationStep(self,dt):

		############################
		###     STOP SIMULATION  ###
		############################
			if self.nbStep>self.nbStepTotal:	 
				self.node.findData("animate").value=0
				plotVectors.main(dt,self.savedVectors,"y_nl","y_lin")
				sys.exit()

			self.nbStep+=1
			self.dt=dt

		############################
		###     RUN SIMULATION   ###
		############################
		### 	Create motion    
			forceInputs = [0,0,0,0] # adapt to number of cables
			for i in range(0,self.nbActuators):
				self.actuators[i].findData("value").value=self.forcesActuators[i]+forceInputs[i]

		### 	Save vectors to plot	
			x, self.u, v=computeStateVector.main(self, self.eqPoint, 1, self.isUsingBeamAdapter)
			xr=self.T.transpose()*x
			y_nl=self.C*x
			yr_nl=self.Cr*self.T.transpose()*x

			## simulate linear models
			self.x_lin = self.A*self.x_lin+(1/(dt))*self.B*np.matrix(forceInputs).transpose()
			y_lin = self.C*self.x_lin

			self.xr_lin = self.Ar*self.xr_lin+(1/(dt))*self.Br*np.matrix(forceInputs).transpose()
			yr_lin = self.Cr*self.xr_lin

			self.savedVectors=saveVectors.main(self.savedVectors,"y_nl",y_nl,"y_lin",y_lin)