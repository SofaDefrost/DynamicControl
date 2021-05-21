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
save vectors passed as input
call : manySavedVectors=saveVectors(manySavedVectors,"nameOfFirstVector",firstVector,"nameOfSecondVector",secondVector)
then access to the saved vectors : 
manySavedVectors["nameOfDesiredVector"]
'''

import numpy as np

def main(savedVectors,*arg):
	# if dictionay is empty, i.e. if no vectors have been saved
	if len(savedVectors)==0:				
		savedVectors={arg[0] : arg[1]}
		for i in xrange(2,len(arg),2): 
			savedVectors[arg[i]]=arg[i+1]
	elif any(savedVectors):	
		# if dictionay is not empty, i.e. if one vector has already been saved
		for i in xrange(0,len(arg),2): 
			savedVectors[arg[i]]=np.concatenate((savedVectors[arg[i]],arg[i+1]),axis=1)
	else:
		savedVectors=0

	return(savedVectors)