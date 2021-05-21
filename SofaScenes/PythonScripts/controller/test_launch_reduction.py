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

''' This file enables to launch the POD reduction from POD to test the algorithm.
It is not used in the standard pipeline, the reduction is normally launched from LAUNCHPODINSOFA.m, the corresponding MATLAB file.'''

import os
import sys
import reduction


scene = "/home/maxime/Documents/MATLAB/SoftRobots/SofaScenes/Projects/Trunk2Cables/"
modelName = "Trunk2Cables"
nbActuators = 2
maxForce = 10
pathToData = "/home/maxime/Documents/MATLAB/SoftRobots/SofaScenes/Projects/Trunk2Cables/data"
pathToMOR = "/home/maxime/Sofa/plugins/ModelOrderReduction/"


reduction.saveSnapshots(scene,modelName,nbActuators,maxForce,pathToData, pathToMOR)