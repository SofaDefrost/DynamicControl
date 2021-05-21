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

import sys
import os
import getpass

'''
parameters.py adds the PythonScripts/tools directory to the path.
Files in tools directory are called from the python controller.
Warning : Adapt pathToData to your file's architecture.
'''

# modelName is the name of the directory where the sofa .pyscn file is located.
modelName=os.path.basename(os.getcwd()) 	
dir_path = os.path.dirname(os.path.realpath(__file__))
pathToData='../../Projects/'+modelName+'/data/'


if len(sys.argv) == 1:
	addController = 0
else:
	addController = 1
	simu=str(sys.argv[1])

	if simu=='modelling':
	    from controller.modelling import Modelling
	elif simu=='snapshots':
	    from controller.snapshots import Snapshots
	elif simu=='closedLoop':
	    from controller.closedLoop import Control
	elif simu=='openLoop':
	    from controller.openLoop import OpenLoop
	elif simu=='realDemo':
	    from controller.realDemo import RealDemo


	def chooseController(simu, rootNode, nbAct, nbEff, mainName, idxEffector, isUsingReduction,isUsingBeamAdapter, \
		forcesCables, pathToData,modelName, maxForceActuators, addLoad = 0 , beamObjectsList = None):
		if simu=='modelling':
			tFinalModelling=50
			Modelling(rootNode, isUsingReduction, isUsingBeamAdapter, nbAct, nbEff, mainName, idxEffector, \
				tFinalModelling, forcesCables, pathToData, modelName, maxForceActuators, addLoad, beamObjectsList)
		elif simu=='snapshots':
			tFinalSnapshots=800
			Snapshots(rootNode , nbAct, mainName, idxEffector, tFinalSnapshots, forcesCables)
		elif simu=='identification':
			timeStepsPerCable=500 # bad name, should be 'timeStepsBeforeConvergence'
			Identification(rootNode, nbAct, mainName, timeStepsPerCable, forcesCables)
		elif simu=='closedLoop':
			tFinalClosedLoop=20
			Control(rootNode, nbAct, mainName,idxEffector,tFinalClosedLoop, forcesCables,pathToData, modelName)
		elif simu=='openLoop':
			tFinalOpenLoop=150
			OpenLoop(rootNode , nbAct, mainName, idxEffector, tFinalOpenLoop, forcesCables, pathToData, modelName, isUsingBeamAdapter)
		elif simu=='NL':
			NonLinear(rootNode , nbAct, mainName, idxEffector,  forcesCables)			
		elif simu=='realDemo':
			tFinal=800
			RealDemo(rootNode, nbAct, mainName,idxEffector,tFinal, forcesCables,pathToData, modelName)