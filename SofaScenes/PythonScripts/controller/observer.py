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
Tests observer designed in matlab.
It requires (matlab) matrices for control. 
These matrices are saved from matlab to "pathToData/nameOfRobot/"
where pathToData and nameOfRobot are defined in PythonScripts/tools/parameters. 
tools/parameters updates path to include required PythonScripts.
'''
import Sofa
import math
import numpy as np

import tools.parameters
import computeStateVector
import plotVectors
import saveVectors
import getMatlabMatrices
import simuInit
import computeExtObs

class Observer(Sofa.PythonScriptController):

	#def __init__(self, rootNode, nameOfInputParameter):
	#		self.parameter=nameOfParameter

    ############################
	###     initialization   ###
	############################
	def bwdInitGraph(self, node):
			self.node = node
			mainName='beam'
			nbActuators=2
			tFinal=60
			idxEffector=528

			optMatrices=['obs']
			self=getMatlabMatrices(pathToData,mainName,self,optMatrices)
			self=simuInit(mainName,nbActuators,idxEffector,tFinal,'observer',modelName,pathToData,self)

			self.cable1.findData("value").value=0
			self.cable2.findData("value").value=0
	def onBeginAnimationStep(self,dt):

			self.t=self.t+dt
			self.dt=dt

		############################
		###     STOP SIMULATION  ###
		############################
			if self.t>self.tFinal*dt:		
				self.node.findData("animate").value="0"	
				plotVectors(self.t,self.dt,self.savedVectors,"y","errY","xr","xrObs")
				sys.exit()

		############################
		###     RUN SIMULATION   ###
		############################

			################################
			###     get state of robot   ###
			################################	
			x, u, v=computeStateVector(self)
			xr=self.T.transpose()*x			#reduced state vector
			######################
			###     observer   ###
			######################
			self.xrObs_Omega, xrObs, omega = computeExtObs(self.Ae,self.obs,self.Be,self.xrObs_Omega,self.y,self.yeObs,self.forceInputs,self.r)

			##################################
			###     measurements outputs   ###
			##################################
			self.y=self.C*x#+10*np.random.rand(3,1)								# real outputs
			yr=self.C*self.T*self.T.transpose()*x 		# outputs of reduced system
			self.yeObs=self.Ce*self.xrObs_Omega 			# reconstructed outputs of reduced system

			##########################
			###     save vectors   ###
			##########################
			self.savedVectors=saveVectors(self.savedVectors,"xr",xr,"xrObs",xrObs,"errX",xr-xrObs,"errY",self.y-self.yeObs,"yr",yr,"yObs",self.yeObs,"y",self.y,"d",omega)

	################################
	###     plot saved vectors   ###
	################################
	def onKeyPressed(self,c):    
		if ord(c)==ord('X'):
			plotVectors(self.t,self.dt,self.savedVectors,"y","errY","xr","xrObs")