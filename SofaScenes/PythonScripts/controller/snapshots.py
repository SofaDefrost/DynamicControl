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
import numpy
import matplotlib.pyplot as plot

import tools.parameters

class Snapshots(Sofa.PythonScriptController):

    def __init__(self, rootNode, nbActuators,mainName,idxEffector,tFinal,forcesCables):
			self.nbActuators=nbActuators
			self.mainName=mainName
			self.idxEffector=idxEffector
			self.tFinal=tFinal 	#number of steps of simulation
			self.forcesCables=forcesCables

    def initGraph(self, node):
	self.node = node

	self.cable1=self.node.getChild(self.mainName).getChild('constraintPoints').getObject("cable0")
	self.cable2=self.node.getChild(self.mainName).getChild('constraintPoints').getObject("cable1")
	self.cable3=self.node.getChild(self.mainName).getChild('constraintPoints').getObject("cable2")
	self.cable4=self.node.getChild(self.mainName).getChild('constraintPoints').getObject("cable3")

	self.t=0

	self.node.findData("animate").value="1"
	self.points=0

    def onBeginAnimationStep(self,dt):
	if self.t>self.tFinal*dt:	
		self.node.findData("animate").value=0 
	else:	
		coeff=200
		if self.t<50*dt:						#Converge to eq point
			self.cable1.findData("value").value=self.forcesCables[0]
			self.cable2.findData("value").value=self.forcesCables[1]
			self.cable3.findData("value").value=self.forcesCables[2]
			self.cable4.findData("value").value=self.forcesCables[3]
		elif self.t<150*dt:			
			self.cable1.findData("value").value=self.forcesCables[0]+9
			self.cable2.findData("value").value=self.forcesCables[1]
			self.cable3.findData("value").value=self.forcesCables[2]
			self.cable4.findData("value").value=self.forcesCables[3]
		elif self.t<250*dt:			
			self.cable1.findData("value").value=self.forcesCables[0]
			self.cable2.findData("value").value=self.forcesCables[1]
			self.cable3.findData("value").value=self.forcesCables[2]
			self.cable4.findData("value").value=self.forcesCables[3]
		elif self.t<350*dt:			
			self.cable1.findData("value").value=self.forcesCables[0]
			self.cable2.findData("value").value=self.forcesCables[1]+15
			self.cable3.findData("value").value=self.forcesCables[2]
			self.cable4.findData("value").value=self.forcesCables[3]
		elif self.t<450*dt:			
			self.cable1.findData("value").value=self.forcesCables[0]
			self.cable2.findData("value").value=self.forcesCables[1]
			self.cable3.findData("value").value=self.forcesCables[2]
			self.cable4.findData("value").value=self.forcesCables[3]
		elif self.t<550*dt:			
			self.cable1.findData("value").value=self.forcesCables[0]
			self.cable2.findData("value").value=self.forcesCables[1]
			self.cable3.findData("value").value=self.forcesCables[2]+15
			self.cable4.findData("value").value=self.forcesCables[3]
		elif self.t<650*dt:			
			self.cable1.findData("value").value=self.forcesCables[0]
			self.cable2.findData("value").value=self.forcesCables[1]
			self.cable3.findData("value").value=self.forcesCables[2]
			self.cable4.findData("value").value=self.forcesCables[3]
		elif self.t<750*dt:			
			self.cable1.findData("value").value=self.forcesCables[0]
			self.cable2.findData("value").value=self.forcesCables[1]
			self.cable3.findData("value").value=self.forcesCables[2]
			self.cable4.findData("value").value=self.forcesCables[3]+15
		elif self.t>750*dt:			
			self.cable1.findData("value").value=self.forcesCables[0]
			self.cable2.findData("value").value=self.forcesCables[1]
			self.cable3.findData("value").value=self.forcesCables[2]
			self.cable4.findData("value").value=self.forcesCables[3]

	self.t=self.t+dt



## saved snapshots animation:
# coeff=200
# 		if self.t<100*dt:						#Converge to eq point
# 			self.cable1.findData("value").value=self.forcesCables[0]
# 			self.cable2.findData("value").value=self.forcesCables[1]
# 		elif self.t<150*dt:				#converge to eq
# 			self.cable1.findData("value").value=self.forcesCables[0]+2*coeff*dt
# 			self.cable2.findData("value").value=self.forcesCables[1]
# 		elif self.t<250*dt:					#converge to eq
# 			self.cable1.findData("value").value=self.forcesCables[0]
# 			self.cable2.findData("value").value=self.forcesCables[1]
# 		elif self.t<300*dt:					#second side for 200dt
# 			self.cable1.findData("value").value=self.forcesCables[0]+4*coeff*dt
# 			self.cable2.findData("value").value=self.forcesCables[1]
# 		elif self.t<400*dt:					#converge to eq
# 			self.cable1.findData("value").value=self.forcesCables[0]
# 			self.cable2.findData("value").value=self.forcesCables[1]
# 		elif self.t<450*dt:					#second side for 200dt
# 			self.cable1.findData("value").value=self.forcesCables[0]+5*coeff*dt
# 			self.cable2.findData("value").value=self.forcesCables[1]
# 		elif self.t<550*dt:					#converge to eq
# 			self.cable1.findData("value").value=self.forcesCables[0]
# 			self.cable2.findData("value").value=self.forcesCables[1]
# 		elif self.t<600*dt:						#Converge to eq point
# 			self.cable1.findData("value").value=self.forcesCables[0]
# 			self.cable2.findData("value").value=self.forcesCables[1]+2*coeff*dt
# 		elif self.t<700*dt:				#converge to eq
# 			self.cable1.findData("value").value=self.forcesCables[0]
# 			self.cable2.findData("value").value=self.forcesCables[1]
# 		elif self.t<750*dt:					#second side for 200dt
# 			self.cable1.findData("value").value=self.forcesCables[0]
# 			self.cable2.findData("value").value=self.forcesCables[1]+4*coeff*dt
# 		elif self.t<850*dt:					#converge to eq
# 			self.cable1.findData("value").value=self.forcesCables[0]
# 			self.cable2.findData("value").value=self.forcesCables[1]
# 		elif self.t<900*dt:					#second side for 200dt
# 			self.cable1.findData("value").value=self.forcesCables[0]
# 			self.cable2.findData("value").value=self.forcesCables[1]+6*coeff*dt
# 		elif self.t<1000*dt:					#converge to eq
# 			self.cable1.findData("value").value=self.forcesCables[0]
# 			self.cable2.findData("value").value=self.forcesCables[1]
# 		elif self.t<1050*dt:						#Converge to eq point
# 			self.cable1.findData("value").value=self.forcesCables[0]+2*coeff*dt
# 			self.cable2.findData("value").value=self.forcesCables[1]+2*coeff*dt
# 		elif self.t<1150*dt:				#converge to eq
# 			self.cable1.findData("value").value=self.forcesCables[0]
# 			self.cable2.findData("value").value=self.forcesCables[1]
# 		elif self.t<1200*dt:					#second side for 200dt
# 			self.cable1.findData("value").value=self.forcesCables[0]+4*coeff*dt
# 			self.cable2.findData("value").value=self.forcesCables[1]+4*coeff*dt
# 		elif self.t<1250*dt:					#converge to eq
# 			self.cable1.findData("value").value=self.forcesCables[0]
# 			self.cable2.findData("value").value=self.forcesCables[1]
# 		elif self.t<1300*dt:					#second side for 200dt
# 			self.cable1.findData("value").value=self.forcesCables[0]+6*coeff*dt
# 			self.cable2.findData("value").value=self.forcesCables[1]+6*coeff*dt
# 		elif self.t<1350*dt:					#converge to eq
# 			self.cable1.findData("value").value=self.forcesCables[0]
# 			self.cable2.findData("value").value=self.forcesCables[1]
# 		elif self.t<1400*dt:					#converge to eq
# 			self.cable1.findData("value").value=self.forcesCables[0]+3*coeff*dt*(self.t-1350*dt)
# 			self.cable2.findData("value").value=self.forcesCables[1]
# 		elif self.t<1450*dt:						#Converge to eq point
# 			self.cable1.findData("value").value=self.forcesCables[0]+3*coeff*dt
# 			self.cable2.findData("value").value=self.forcesCables[1]+3*coeff*dt*(self.t-1400*dt)
# 		elif self.t<1500*dt:						#Converge to eq point
# 			self.cable1.findData("value").value=self.forcesCables[0]+3*coeff*dt*(1-(self.t-1450*dt))
# 			self.cable2.findData("value").value=self.forcesCables[1]+3*coeff*dt
# 		elif self.t<1550*dt:				#converge to eq
# 			self.cable1.findData("value").value=self.forcesCables[0]
# 			self.cable2.findData("value").value=self.forcesCables[1]+3*coeff*dt*(1-(self.t-1500*dt))
# 		elif self.t<1650*dt:					#second side for 200dt
# 			self.cable1.findData("value").value=self.forcesCables[0]
# 			self.cable2.findData("value").value=self.forcesCables[1]
# 		elif self.t<1700*dt:					#converge to eq
# 			self.cable1.findData("value").value=self.forcesCables[0]+6*coeff*dt*(self.t-1650*dt)
# 			self.cable2.findData("value").value=self.forcesCables[1]
# 		elif self.t<1750*dt:						#Converge to eq point
# 			self.cable1.findData("value").value=self.forcesCables[0]+3*coeff*dt
# 			self.cable2.findData("value").value=self.forcesCables[1]+3*coeff*dt*(self.t-1700*dt)
# 		elif self.t<1800*dt:						#Converge to eq point
# 			self.cable1.findData("value").value=self.forcesCables[0]+3*coeff*dt*(1-(self.t-1750*dt))
# 			self.cable2.findData("value").value=self.forcesCables[1]+3*coeff*dt
# 		elif self.t<1850*dt:				#converge to eq
# 			self.cable1.findData("value").value=self.forcesCables[0]
# 			self.cable2.findData("value").value=self.forcesCables[1]+3*coeff*dt*(1-(self.t-1800*dt))
# 		elif self.t>1900*dt:					#second side for 200dt
# 			self.cable1.findData("value").value=self.forcesCables[0]
# 			self.cable2.findData("value").value=self.forcesCables[1]