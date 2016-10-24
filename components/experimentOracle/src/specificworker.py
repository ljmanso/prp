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
import os.path

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)

		self.T = QtCore.QTime()
		self.T.start()

		self.current = 0
		self.logfile = open('experimentLog.txt', 'w')
		self.loadOrGenerateExperimentList()

		self.Period = 200
		self.timer.start(self.Period)
		self.timer.timeout.connect(self.compute)
		self.ui.startButton.clicked.connect(self.goStartPosition)
		self.ui.missionButton.clicked.connect(self.doMission)
		self.ui.doneButton.clicked.connect(self.missionDone)

		self.ui.wb1.clicked.connect(self.wb1)
		self.ui.wb2.clicked.connect(self.wb2)
		self.ui.wb3.clicked.connect(self.wb3)
		self.ui.wb4.clicked.connect(self.wb4)
		self.ui.wb5.clicked.connect(self.wb5)
		self.ui.rb1.clicked.connect(self.rb1)
		self.ui.rb2.clicked.connect(self.rb2)
		self.ui.rb3.clicked.connect(self.rb3)
		self.ui.rb4.clicked.connect(self.rb4)
		self.ui.rb5.clicked.connect(self.rb5)
		
		self.notEvenReadyButtons = [ self.ui.startButton ]
		self.readyButtons = [ self.ui.missionButton ]
		self.inMissionButtons = [ self.ui.doneButton, self.ui.wb1, self.ui.wb2, self.ui.wb3, self.ui.wb4, self.ui.wb5, self.ui.rb1, self.ui.rb2, self.ui.rb3, self.ui.rb4, self.ui.rb5 ]
		self.uistate = ''
		self.setUIState("notEvenReady")
		
	def log(self, s):
		s2 = str(self.T.elapsed()) + ' # ' + s + '\n'
		self.logfile.write(s2)
		print s2
		self.logfile.flush()


	def setParams(self, params):
		return True
	
	def loadOrGenerateExperimentList(self):
		self.points = {}
		self.points['0'] = [0.0, 0.1, 0.2, 1.2]
		self.points['1'] = [1.0, 1.1, 1.2, 1.2]
		self.points['2'] = [2.0, 2.1, 2.2, 1.2]
		self.points['3'] = [3.0, 3.1, 3.2, 1.2]
		self.points['4'] = [4.0, 4.1, 4.2, 1.2]
		
		self.objects = ['cup', 'ball', 'wrench', 'screen', 'screwdriver', 'stapler', 'mouse', 'bag', 'pingpong', 'noodles']
		
		
		
		self.experiments = []
		schedule = 'experimentSchedule.txt'
		if os.path.isfile(schedule):
			with open(schedule, 'r') as f:
				for line in f.readlines():
					experiment = eval(line)
					print experiment
					self.experiments.append(experiment)
		else:
			with open(schedule, 'w') as f:
				for obj in self.objects:
					srcs = self.points.keys()
					random.shuffle(srcs)
					for src in srcs:
						experiment = [ obj, src, self.points[src] ]
						print experiment
						self.experiments.append(experiment)
						f.write(str(experiment)+'\n')

		self.log("Experiment started")



	@QtCore.Slot()
	def compute(self):
		self.ui.targetLabel.setText(str(self.experiments[self.current]))
		self.ui.counterLabel.setText(str(self.current))
		pass


	@QtCore.Slot()
	def goStartPosition(self):
		self.setUIState("ready")
		print 'goStartPosition()'
		target = RoboCompTrajectoryRobot2D.TargetPose
		target.doRotation = True
		target.x = self.experiments[self.current][2][0]
		target.y = self.experiments[self.current][2][1]
		target.z = self.experiments[self.current][2][2]
		target.rx = 0.
		target.ry = self.experiments[self.current][2][3]
		target.rz = 0.
		self.trajectoryrobot2d_proxy.go(target)


	@QtCore.Slot()
	def wb1(self):
		self.wrongTable(1)
	@QtCore.Slot()
	def wb2(self):
		self.wrongTable(2)
	@QtCore.Slot()
	def wb3(self):
		self.wrongTable(3)
	@QtCore.Slot()
	def wb4(self):
		self.wrongTable(4)
	@QtCore.Slot()
	def wb5(self):
		self.wrongTable(5)
	@QtCore.Slot()
	def wrongTable(self, t):
		self.log("Going to wrong table" + str(t))

	@QtCore.Slot()
	def rb1(self):
		self.rightTable(1)
	@QtCore.Slot()
	def rb2(self):
		self.rightTable(2)
	@QtCore.Slot()
	def rb3(self):
		self.rightTable(3)
	@QtCore.Slot()
	def rb4(self):
		self.rightTable(4)
	@QtCore.Slot()
	def rb5(self):
		self.rightTable(5)
	@QtCore.Slot()
	def rightTable(self, t):
		self.log("Going to right table" + str(t))



	@QtCore.Slot()
	def doMission(self):
		self.agmexecutive_proxy.setMission("/home/robocomp/robocomp/components/robocomp-shelly/etc/targetFindMug.aggt")


	@QtCore.Slot()
	def missionDone(self):
		self.setUIState("notEvenReady")
		self.log("Mission " + str(self.current) + " done. It took " + str(self.missionT.elapsed()) + " ms")
		self.current += 1


	def setUIState(self, v):
		if v == "notEvenReady":
			for b in self.notEvenReadyButtons: b.setEnabled(True)
			for b in self.readyButtons: b.setEnabled(False)
			for b in self.inMissionButtons: b.setEnabled(False)
		if v == "ready":
			for b in self.notEvenReadyButtons: b.setEnabled(False)
			for b in self.readyButtons: b.setEnabled(True)
			for b in self.inMissionButtons: b.setEnabled(False)
		if v == "inMission":
			for b in self.notEvenReadyButtons: b.setEnabled(False)
			for b in self.readyButtons: b.setEnabled(False)
			for b in self.inMissionButtons: b.setEnabled(True)
		self.uistate = v





	#
	# reloadConfigAgent
	#
	def reloadConfigAgent(self):
		ret = bool()
		return ret


	#
	# activateAgent
	#
	def activateAgent(self, prs):
		if 'imagine' in prs['plan'] and self.uistate != "inMission":
			self.setUIState("inMission")
			self.log("Start mission " + str(self.current) + " " + str(self.experiments[self.current]))
			self.missionT = QtCore.QTime()
			self.missionT.start()

		ret = bool()
		return ret


	#
	# setAgentParameters
	#
	def setAgentParameters(self, prs):
		ret = bool()
		return ret


	#
	# getAgentParameters
	#
	def getAgentParameters(self):
		ret = ParameterMap()
		return ret


	#
	# killAgent
	#
	def killAgent(self):
		pass


	#
	# uptimeAgent
	#
	def uptimeAgent(self):
		ret = int()
		return ret


	#
	# deactivateAgent
	#
	def deactivateAgent(self):
		ret = bool()
		return ret


	#
	# getAgentState
	#
	def getAgentState(self):
		ret = StateStruct()
		return ret


