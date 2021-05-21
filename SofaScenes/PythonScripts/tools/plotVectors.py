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
-plotVectors.main plots vectors whose name are given in argument.
-The vector to be plotted must have been saved in a dictionary 
'''


import matplotlib.pyplot as plot
import numpy as np

def main(dt,savedVectors,*arg):
	for i in xrange(0,len(arg),1):
		fig=plot.figure(i)
		lengthVec = savedVectors[arg[i]].shape[1]
		t=np.arange(0,lengthVec*dt,dt)
		if lengthVec==len(t):
			plot.plot(t,savedVectors[arg[i]].transpose(),linewidth=2.0) 
			plot.xlabel("Time (s)", fontsize=21)		
		else:
			plot.plot(savedVectors[arg[i]].transpose(),linewidth=2.0) 		
			plot.xlabel("nb step", fontsize=21)				
		plot.ticklabel_format(axis='y', style='sci',scilimits=(1,3))
		ax = plot.gca()
		ax.yaxis.get_offset_text().set_fontsize(26)
		plot.xticks(fontsize=21, rotation=0)
		plot.yticks(fontsize=21, rotation=0)
		plot.title(arg[i])
	plot.show()
	return 0


def plotY(dt,savedVectors,*arg):
	y_reel = savedVectors["yreel"]
	y_obs = savedVectors["yobs"]
	lengthVec = y_reel.shape[1]
	t=np.arange(0,lengthVec*dt,dt)
	if lengthVec==len(t):
			line_ref = plot.plot(t,y_obs.transpose(),'-',color='red',linewidth=2.0, label = 'ref') 	
			line_yreel = plot.plot(t,y_reel.transpose(),'-',color='blue',linewidth=2.0, label = 'yreel')
	plot.show()
	return 0