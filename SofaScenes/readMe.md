Overview
========

$/SoftRobots/SofaScenes contains two directories:

- Projects/ : contains several directories, one for each soft robot. Each directory contains a .pyscn file that describes the sofa scene of the robot.

- PythonScripts/ : contains the python scripts called by the sofa scenes. It contains two directories, PythonScripts/controller contains the scripts called from the sofa scene according to the sofa simulation. it can be "modelling", "closedLoop", "openLoop", etc... The firectory PythonScripts/tools contains several files called by the python controllers.  


Sofa Scenes
-----------

Each directory contains a .pyscn file that describe the sofa simulation. Each scene is launched this way:

- runSofa sceneName.pyscn --argv inputArgument

where inputArgument is a string describing the simulation to launch. It can be:

| Name of inputArgment  | Description                                                 | 
|-----------------------|-------------------------------------------------------------|
|  'modelling'          | save simulation data to create a dynamical model in matlab  | 
|  'openLoop'           | compare models from matlab and sofa                         | 
|  'closedLoop'         | tests controller designed in matlab                         |
|  'observer'           | tests observer designed in matlab                           | 
|  'snapshots'          | save snapshots to perform model reduction                   | 

- Remark:
Scene must be written with specific names to match the name in the PythonScriptController.

### Scenes parameters
The python controller takes as input the following parameters, that must be defined in the scene file:

	totalMass : float, total mass of the structure
	pRatio : float, Poisson's ratio of the material
	yModulus : float, Young's modulus of the material
	addLoad : boolean, if true, more weight is added to specific part of the robot
	mainName : string, name of node containing main mechanical object of the scene
	isUsingBeamAdapter : boolean, true if the scene uses beam adapter
	isUsingReduction : boolean, true if the scenes uses the reduction plugin
	nbAct : integer, number of actuators of the structure
	nbEff : integer, number of effector of the robot's = number of outputs of the system
	idxEffector : list of integers, list of indices of the effectors
	maxForceActuators : integer, maximum force an actuator can apply

### Add to the file header:
	import os
	import sys

	path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'
	dirPath = os.path.dirname(os.path.abspath(__file__))+'/'
	sys.path.append(dirPath+'../../PythonScripts/')

	from tools.parameters import *

### Add to the scene header:
The following line adds the required PythonScriptController to the scene. The function chooseController is defined in tools/parameters.py

	if addController:
		chooseController(simu,rootNode, nbAct, mainName, idxEffector, isUsingReduction,isUsingBeamAdapter, \
        	forcesCables, pathToData, modelName, maxForceActuators, addLoad) 
