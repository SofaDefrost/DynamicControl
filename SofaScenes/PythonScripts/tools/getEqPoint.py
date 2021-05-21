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
Returns equilibrium point of the robot that has been saved before during the modelling simulation.
Eqpoint must be saved in a text file named EqPointNameOfStudiedRobot.txt

Output args:
	- eqPoint : vector of position where the robot has been linearized, without the constraints nodes
	- eqPointComplete : vector of position where the robot has been linearized, with the constraint nodes
'''

import numpy as np

def main(modelName,pathToData):
	file=open(pathToData+"/EqPoint"+modelName+".txt", "r")
	eqPoint=file.read()
	file.close()
	eqPoint=np.matrix(eqPoint).transpose()

	file=open(pathToData+"/EqPointComplete"+modelName+".txt", "r")
	eqPointComplete=file.read()
	file.close()
	eqPointComplete=np.matrix(eqPointComplete).transpose()

	return eqPoint, eqPointComplete