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

import sys

import os
import getpass
modelName=os.path.basename(os.getcwd())

# here, simulation data are saved in the directory as the scene file.
# for other communication between Matlab and Sofa, a directory dedicated to data is used. 
# It is define din the 'SofaFolder' structure.

pathToData='/home/maxime/Sofa/SimulationData/'+modelName

print(pathToData)

class Test(Sofa.PythonScriptController):

	def initGraph(self, node):
		self.node = node
		self.t=0
		self.node.findData("animate").value=1

	def onBeginAnimationStep(self,dt):
		if self.t>10*dt:	
			self.node.findData("animate").value="0"
			print(pathToData+"/Result.txt")		
			file=open(pathToData+"/Result.txt", "a") 			
			file.write("%s\n" % 'SOFA is working.') 
			file.close()  
			sys.exit()

		self.t=self.t+dt
