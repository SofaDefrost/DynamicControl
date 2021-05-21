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

'''
Tests controller designed in matlab.
It requires (matlab) matrices for control. 
These matrices are saved from matlab to "pathToData/nameOfRobot/"
where pathToData and nameOfRobot are defined in PythonScripts/tools/parameters. 
tools/parameters updates path to include required PythonScripts.
'''

import Sofa
import math
import numpy as np
import sys

from tools import saveSimulationData, computeStateVector, simuInit, plotVectors, saveVectors, \
 getMatlabMatrices, computeExtObs, computeFeedback, convertForceToDisp


class Control(Sofa.PythonScriptController):

	def __init__(self, rootNode, nbActuators,mainName,idxEffector,tFinal, forcesActuators, pathToData, modelName):
			self.nbActuators=nbActuators
			self.mainName=mainName
			self.idxEffector=idxEffector
			self.tFinal=tFinal 	#number of steps of simulation
			self.computeObs= 0 		# set to 1 if you want to compute observed state
			self.useObs = 0 		# set to 1 if you want to use observed state in feedback
			self.needDisp= 0 		# set to 1 if actuators require displacement input (else actuation =  force)
			self.tracking = 0		
			self.applyFeedback = 1
			self.pathToData=pathToData
			self.modelName=modelName
			self.forcesActuators=forcesActuators
			self.t=0

    ############################
	###     initialization   ###
	############################
	def bwdInitGraph(self, node):
			self.node = node
			self=getMatlabMatrices.feedback(self.pathToData,self)
			self=simuInit.closedLoop(self)
			self.actuationType='force'
			for i in range(0,self.nbActuators):
				self.actuators[i].findData("valueType").value=self.actuationType

	def onBeginAnimationStep(self,dt):

		############################
		###     STOP SIMULATION  ###
		############################
			if (self.t-(self.tFinal*dt))>1e-8:	 # if self.t > tFinal * dt
				self.node.findData("animate").value=0
				plotVectors.main(self.dt,self.savedVectors,"y","forceInputs")
				#plotVectors.compareEachElement(self.dt,self.savedVectors)
				sys.exit()

			self.dt=dt
			self.t=self.t+dt

		############################
		###     RUN SIMULATION   ###
		############################

			################################
			###     get state of robot   ###
			################################	
			x, u, v=computeStateVector.main(self, self.eqPoint, 1)
			xr=self.T.transpose()*x			#reduced state vector

			######################
			###     observer   ###
			######################
			if self.computeObs:
				self.xrObs_Omega, xrObs, omega = computeExtObs.main(self.Ae,self.Be,self.Ce,self.obs,self.xrObs_Omega, \
					self.y,self.yeObs,self.forceInputs,self.r)
				self.yeObs = self.Ce*self.xrObs_Omega

			####################################
			###     compute control inputs   ###
			####################################
			x_red=xr	# by default : feedback without observer
			if self.useObs:
				x_red=xrObs

			if self.applyFeedback:
				if self.tracking:
					self.forceInputs, self.x_star, y_star =computeFeedback.tracking(self.ref, x_red, self.x_star, self.f,self.f_star,self.nbActuators, \
						self.A_star,self.B_star,self.C_star, omega)
				else:
					self.forceInputs=computeFeedback.main(x_red,self.L,self.nbActuators)
			else:
				self.forceInputs=0*computeFeedback.main(x_red,self.f,self.nbActuators)
	
			###################################
			#####    Set actuators input   ####
			###################################
			if self.applyFeedback:
				if self.t>0*dt:
					if self.actuationType=='force':
						#self.dispInputs, varDelta =convertForceToDisp.main(u,v,self.invAeuler,self.H,self.massMatrix,self.matrixK,self.forceInputs,self.dispInputs,dt,self)
						for i in range(self.nbActuators):
							self.actuators[i].findData("value").value=self.forcesActuators[i]+float(self.forceInputs[i][0])*dt  # or [0][i] ?
					elif self.actuationType=='displacement':
						self.dispInputs, varDelta =convertForceToDisp.main(u,v,self.invAeuler,self.H,self.massMatrix,self.matrixK,self.forceInputs,self.dispInputs,dt,self)
						for i in range(self.nbActuators):
							self.actuators[i].findData("value").value=float(self.dispInputs[i][0])*dt
			else:
				for i in range(self.nbActuators):
					self.actuators[i].findData("value").value=self.forcesActuators[i]

			##################################
			###     measurements outputs   ###
			##################################
			self.y=self.C*x#+10*np.random.rand(3,1)								# real outputs
			yr=self.C*self.T*self.T.transpose()*x 		# outputs of reduced system

			##########################
			###     save vectors   ###
			##########################
			normU = u*u.transpose()
			normV = v*v.transpose()
			self.savedVectors=saveVectors.main(self.savedVectors,"xr",xr,"yr",yr,"y",self.y, \
				"forceInputs",self.forceInputs)

	################################
	###     plot saved vectors   ###
	################################
	def onKeyPressed(self,c):    
		if ord(c)==ord('X'):
			plotVectors.main(self.t,self.dt,self.savedVectors,"y","xr")