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
computeExtObs returns the observed state from the outputs and the reduced order system.
'''

import numpy as np

def main(Ae,Be,Ce,obs,xrObsOmega,y,yeObs,controlInputs,r):
	eigA, eigVecA=np.linalg.eig(Ae-obs*Ce)
	#print(abs(eigA))
	tmp=obs*(yeObs-y)
	# print(abs(eigA))
	xrObs_Omega=Ae*xrObsOmega+Be*controlInputs-obs*(yeObs-y)#
	xrObs=xrObs_Omega[0:r]
	nd=Ae.shape[0]-r			# nb of disturbances
	omega=xrObs_Omega[r:] 		# distrubances = nd last element of xrObs_omega

	return xrObs_Omega, xrObs, omega