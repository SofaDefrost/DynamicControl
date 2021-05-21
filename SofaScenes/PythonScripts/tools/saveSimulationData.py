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
saveSimulationData is used in the modelling PythonScriptController to save all simulation data
that are needed in matlab to construct a model of the studied robot.

optional input arguments used if beam adapter is used
This file requires each object containing the topology of the beam to reconstruct the mass and stiffness matrices
'''

import os
import numpy 
import time

def main(self,dt,pathToData,nameRobot,nameOfPreconditioner,mechaObject,maxForceActuators, beamObjects = None ):
	#####################################################
	### 	Save system matrices : 					  ###
	### 		mass, stiffness and damping 		  ###
	#####################################################
		if not self.isUsingBeamAdapter:
			# save new MDKmatrix.txt, old one has been deleted in simuInit
			nameOfPreconditioner.findData("savingPrecision").value=18 
			nameOfPreconditioner.findData("savingFilename").value=pathToData+"/MDKmatrix"+nameRobot+".txt"
			nameOfPreconditioner.findData("savingMatrixToFile").value=1	
			# save mass file
			file=open(pathToData+"/mass"+nameRobot+".txt", "w") 				#indices constraints nodes
			for i in self.diagMass:
				file.write(str(i)+'\n')
			file.close() 
		elif self.isUsingBeamAdapter:
			for item in beamObjects:
				item.findData('storeSystemMatrices').value='1'
				txtFile_numBeams = pathToData+"/numBeams_"+item.getName()+".txt"
				file=open(txtFile_numBeams, "w") 
				file.write(str(item.findData('numBeams').value)) 
				file.close()

	#####################################################
	### 	Save input matrix 	 					  ###
	#####################################################	
		### save indices of actuators
		indxActuators=mechaObject.findData("constraint").value
		# ToDo : how to save indxActuators with better precision ? 
		# currently, string is used because :
		# can not convert it to matrix because each row has different size 
		file=open(pathToData+"/indxActuators"+nameRobot+".txt", "w") 
		file.write(str(indxActuators)) 
		file.close() 

	#####################################################
	### 	Save simulation data : 					  ###
	### 		time step, number of nodes... 		  ###
	#####################################################
		### save indices of Free nodes of the mesh
		file=open(pathToData+"/indxFreeNodes"+nameRobot+".txt", "w") 	#indices constraints nodes
		file.write(str(self.freeDOF)) 
		file.close()  
		### save Simulation data
		rayleighStiff=self.solver.findData("rayleighStiffness").value
		rayleighMass=self.solver.findData("rayleighMass").value

		file=open(pathToData+"/SimuData"+nameRobot+".txt", "w")
		file.write("Number of Nodes\n%s\nRayleigh mass parameter\n%s\nRayleigh stiffness parameter\n%s\n\
Step time\n%s\nNumber of actuators\n%s\nNumber of effectors\n%s\nIndices of effectors\n%s\n\
Maximum force actuators\n%s\nIs using beam adapter\n%s\nIs using reduction\n%s\n" \
			% (str(self.numNodes), str(rayleighMass), str(rayleighStiff), str(dt), \
			str(self.nbActuators), str(self.nbEffectors), str(self.idxEffector), str(maxForceActuators), \
			str(self.isUsingBeamAdapter), str(self.isUsingReduction)))
		file.close()  
