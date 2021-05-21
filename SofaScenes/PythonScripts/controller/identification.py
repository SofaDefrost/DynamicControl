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

import tools.parameters
import simuInit
import computeCompleteStateVector 	
import getEqPoint

print('Launched: identification simulation.')

class Identification(Sofa.PythonScriptController):

	def __init__(self, rootNode, nbActuators,mainName,timeStepsPerCable,forcesCables):
			self.nbActuators=nbActuators
			self.mainName=mainName
			self.timeStepsPerCable=timeStepsPerCable
			self.tFinal=(nbActuators)*timeStepsPerCable 	#number of steps of simulation
			self.forcesCables=forcesCables

    ############################
	###     initialization   ###
	############################
	def bwdInitGraph(self, node):
			self.node = node

			# self.mechaObject is used in computeStateVector to get position and velocity vectors
			self.mechaObject=self.node.getChild(self.mainName).getObject('meca')	

			# Number of nodes of the mesh
			U=np.matrix(self.mechaObject.findData("position").value)
			self.numNodes=len(U) 		# Number of nodes of the mesh

			self.eqPoint, self.eqPointComplete=getEqPoint(modelName,pathToData)

			self.cable=[]
			for i in range(0,self.nbActuators):
				self.cable.append(self.node.getChild(self.mainName).getChild('constraintPoints').getObject("cable"+str(i)))
			
			self.t=0
			self.timeSteps_oneCable=0
			self.indx=0 
			self.node.findData("animate").value=1


	def onBeginAnimationStep(self,dt):

			self.t+=dt

		############################
		###     STOP SIMULATION  ###
		############################
			if (self.t-(self.tFinal*dt))>1e-8:	 # if self.t > ( tFinal * dt)	
				self.node.findData("animate").value=0
				x, u, v=computeCompleteStateVector(self)
				np.savetxt(pathToData+"/final_pos_cable"+str(self.indx)+".txt",u)
				print("Saved final position cable "+str(self.indx))
				print("Done: identification simulation.")
				sys.exit()

		############################
		### 	Create motion    ###
		############################
			# get to desired equilibrium point	
			for i in range(0,self.nbActuators):
				self.cable[i].findData("value").value=self.forcesCables[i]

			# move cables one by one and save position
			defaultActuation=1
			if self.indx<self.nbActuators:
				if self.timeSteps_oneCable<self.timeStepsPerCable:
					self.cable[self.indx].findData("value").value=self.forcesCables[self.indx]+defaultActuation
					self.timeSteps_oneCable+=1
				else:
					x, u, v=computeCompleteStateVector(self)
					np.savetxt(pathToData+"/final_pos_cable"+str(self.indx)+".txt",u)
					print("Saved final position cable "+str(self.indx))
					self.timeSteps_oneCable=0
					self.indx+=1