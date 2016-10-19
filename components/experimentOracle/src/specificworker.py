#
# Copyright (C) 2016 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import sys, os, traceback, time

from PySide import *
from genericworker import *

import random

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000

		self.loadOrGenerateExperimentList()

		self.ui.startButton.clicked.connect(self.goStartPosition)
		self.ui.missionButton.clicked.connect(self.doMission)
		self.ui.doneButton.clicked.connect(self.missionDone)

		self.timer.start(self.Period)

	def setParams(self, params):
		return True
	
	def loadOrGenerateExperimentList(self):
		self.points = {}
		self.points['0'] = [1.0,2.0,3.0]
		self.points['1'] = [1.0,2.0,3.0]
		self.points['2'] = [1.0,2.0,3.0]
		self.points['3'] = [1.0,2.0,3.0]
		self.points['4'] = [1.0,2.0,3.0]
		self.points['5'] = [1.0,2.0,3.0]
		self.points['6'] = [1.0,2.0,3.0]
		self.points['7'] = [1.0,2.0,3.0]
		self.points['8'] = [1.0,2.0,3.0]
		self.points['9'] = [1.0,2.0,3.0]
		
		self.objects = ['cup', 'ball', 'wrench', 'screen', 'screwdriver', 'stapler', 'mouse', 'bag', 'pingpong', 'noodles']
		
		
		
		try:
			self.experiments = []
			with open('experimentSchedule.txt','r') as f:
				for line in f.readlines():
					self.experiments.append(line.split('#'))
		except:
			for obj in self.objects:
				srcs = self.points.keys()
				random.shuffle(srcs)
				for src in srcs:
					print obj, srcs
				
	@QtCore.Slot()
	def compute(self):
		pass
	
	
	@QtCore.Slot()
	def goStartPosition(self):
		return True

	@QtCore.Slot()
	def doMission(self):
		return True


	@QtCore.Slot()
	def missionDone(self):
		return True





