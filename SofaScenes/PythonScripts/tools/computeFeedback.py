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


'''computeFeedback returns the control input, i.e. the actuator contributions.
This output is a force, and should therefore be converted to displacement if needed.
This can be done using convertForceToDisp.py'''


import numpy as np

def main(x_red,f,nbAct):

	actuatorsForce= -f*x_red  # x_red = either xr or xr_obs

	### control input Saturation 
	# if control[0] < 0 set control[0] to zero
	# as cables cannot push:

	fMin = 00
	for i in range(0,nbAct):
		if actuatorsForce[i][0]<fMin:
			actuatorsForce[i][0]=fMin

	fMax = 100
	for i in range(0,nbAct):
		if actuatorsForce[i][0]>fMax:
			actuatorsForce[i][0]=fMax

	return actuatorsForce



def tracking(ref, x_red, xi, x_star, L, Li, L_star, nbActuators, A_star, B_star, C_star, omega, nbAct):
	'''	x_red  : reduced order state (with or without observer)
		x_star : state of reference model
		omega 	   : unknown input of observer
	'''

	x_star = A_star*x_star + B_star*ref
	y_star = C_star * x_star

	actuatorsForce = -L * x_red - 1*Li*xi -L_star * x_star - 1*omega[:nbAct]

	forceMax = 5e2
	forceMin = -forceMax
	for i in range(len(actuatorsForce)):
		if actuatorsForce[i]>forceMax:
			actuatorsForce[i]=forceMax
		if actuatorsForce[i]< forceMin:
			actuatorsForce[i]= forceMin

	if actuatorsForce[0]<0:
		actuatorsForce[0]=0

	return actuatorsForce , x_star, y_star

