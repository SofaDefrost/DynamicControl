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
Save snapshots of both position and velocity of the robot.
Requires the Model Reduction Plugin
(see example in Sofa/plugins/ModelOrderReduction/tools/modelOrderReduction.py)

Input arguments:
(all of type 'string')

absPathToSceneDir : 	absolute path to directory where the scene to reduce is stored.
modelName 		  : 	name of the robot studied 	(e.g. name of the scene without '.pyscn' extension)
nbActuators		  : 	number of actuators to use to "shake" the robot and save snapshots
maxForceToApply   : 	maximum force to apply to each actuator
						the force applied to each actuators goes from 0 to maxForceToApply
						with an increment computed such that there are 50 increments between force = 0 and force = maxForceToApply
'''

import sys
import os
import warnings

def saveSnapshots(absPathToSceneDir, modelName, nbActuators, maxForceToApply, pathToSofaSimulationData, pathToModelReductionPlugin):
	sys.path.append(pathToModelReductionPlugin+'python/') 
	# MOR IMPORT
	try :
		from mor.reduction import ReduceModel
		from mor.reduction.container import ObjToAnimate
	except:
		raise ImportError("Read the doc about Model Order Reduction plugin.")

	originalScene = absPathToSceneDir+modelName+".pyscn"

	outputDir = pathToSofaSimulationData+"/reduction/"

	nodesToReduce ='/'+modelName

	maxForceToApply = float(maxForceToApply)
	incr = maxForceToApply/200
	listObjToAnimate = []
	for i in range(int(nbActuators)):
		actuator_i = ObjToAnimate(modelName+"/constraintPoints/actuator"+str(i), incr=incr ,incrPeriod=5, rangeOfAction=maxForceToApply, dataToWorkOn= "value")
		listObjToAnimate.append(actuator_i)

	# Tolerance
	tolModes = 0.001
	tolGIE =  0.05

	reduceMyModel = ReduceModel(    originalScene,  
	                                nodesToReduce,
	                                listObjToAnimate,
	                                tolModes,
	                                tolGIE,
	                                outputDir,
	                                saveVelocitySnapshots = True)

	reduceMyModel.phase1()