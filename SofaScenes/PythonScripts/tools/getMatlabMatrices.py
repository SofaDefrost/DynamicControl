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

import numpy as np
from scipy import sparse
from scipy.io import loadmat

'''
getMatlabMatrices returns all matlab matrices that are needed for sofa simulation.
'''

def default(pathToData,self):
	defaultMatrices=loadmat(pathToData+'/defaultMatrices.mat')
	self.T=np.matrix(defaultMatrices['T'])
	self.Ar=np.matrix(defaultMatrices['Ar'])
	self.Br=np.matrix(defaultMatrices['Br'])
	self.Cr=np.matrix(defaultMatrices['Cr'])
	self.C=np.matrix(defaultMatrices['C'])
	return self

def openLoop(pathToData,self):
	self = default(pathToData,self)
	if self.needDisp:
		self = dispMatrices(pathToData,self)
	if self.needFull:
		self = full(pathToData,self)
	return self

def obs(pathToData,self):
	self = default(pathToData,self)
	obsMatrices=loadmat(pathToData+'/obsMatrices.mat')
	self.Ae=np.matrix(obsMatrices['Ae'])
	self.Be=np.matrix(obsMatrices['Be'])
	self.Ce=np.matrix(obsMatrices['Ce'])
	self.obs=np.matrix(obsMatrices['obs'])
	return self

def full(pathToData,self):
	fullMatrices=loadmat(pathToData+'/fullMatrices.mat')
	self.A=np.matrix(fullMatrices['A'])
	self.B=np.matrix(fullMatrices['B'])
	self.C=np.matrix(fullMatrices['C'])
	return self

def feedback(pathToData,self):
	self = default(pathToData,self)
	if self.computeObs:
		self = obs(pathToData,self)
	feedbackMatrix=loadmat(pathToData+'/feedback.mat')
	self.L=np.matrix(feedbackMatrix['L'])
	if self.tracking:
		self = tracking(pathToData, self)
		self.Li=np.matrix(feedbackMatrix['Li'])
		self.L_star=np.matrix(feedbackMatrix['L_star'])
	if self.needDisp:
		self = dispMatrices(pathToData,self)
	return self

def tracking(pathToData,self):
	trackingMatrices=loadmat(pathToData+'/trackingMatrices.mat')
	self.A_star=np.matrix(trackingMatrices['A_star'])
	self.B_star=np.matrix(trackingMatrices['B_star'])
	self.C_star=np.matrix(trackingMatrices['C_star'])
	return self


def dispMatrices(pathToData,self):
	dispMatrices=loadmat(pathToData+'/convertForceToDisp.mat')
	self.M1=np.matrix(dispMatrices['M1'])
	self.M2=np.matrix(dispMatrices['M2'])
	self.M3=np.matrix(dispMatrices['M3'])
	self.M4=np.matrix(dispMatrices['M4'])
	return self
