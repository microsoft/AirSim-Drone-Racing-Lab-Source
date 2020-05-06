import re
import numpy as np
import matplotlib.pyplot as plt
MAX_TIME = 100
		
class Racer:
	def __init__(self,name):
		self.name = name
		self.odometry = []
		self.violations = []
		self.collisions = []
		self.time = []
		self.penalty = []
		self.passed_gate = []
		self.missed_gate = []
		self.is_dqed = False
		self.is_finished = False
		self.complete_time = 0
	def parse(self, string):
		if not (self.is_finished):
			if string.startswith("Odometry:"):
				self.odometry += [Odometry.parse(string)]
			elif string.startswith("Gates Passed:"):
				self.passed_gate += [int(re.findall("\d+", string)[0])]
			elif string.startswith("Gates Missed:"):
				self.missed_gate += [int(re.findall("\d+", string)[0])]
			elif string.startswith("Time:"):
				self.time += [float(re.findall("\d+", string)[0]) / 1000]
			elif string.startswith("Penalty:"):
				self.penalty += [int(re.findall("\d+", string)[0])]
			elif "DISQUALIFIED" in string:
				self.is_dqed = True
				self.is_finished = True
				complete_time = MAX_TIME
				print("{} got DQed!".format(self.name))
			elif "FINISHED" in string:
				self.complete_time = self.time[-1]
				self.is_finished = True
				print("{} finished at {}!".format(self.name, self.complete_time))
	def score(self):
		if self.complete_time:
			self.effective_time = self.complete_time + self.penalty[-1]
		else:
			self.effective_time = MAX_TIME
		
		return self.effective_time

class Odometry:
	x = 0
	y = 0
	z = 0
	roll = 0
	pitch = 0
	yaw = 0
	def parse(string):
		return_odom = Odometry()
		all_vars = re.findall("-?\d+\.\d+", string)
		if len(all_vars) == 6:
			return_odom.x = float(all_vars[0]) / 100
			return_odom.y = float(all_vars[1]) / 100
			return_odom.z = float(all_vars[2]) / 100
			return_odom.roll = float(all_vars[3]) / 100
			return_odom.pitch = float(all_vars[4]) / 100
			return_odom.yaw = float(all_vars[5])
			return return_odom

class Evaluator:
	def __init__(self, filename):
		file = open(filename, "r")
		self.file_content = file.readlines()
		file.close()
		self.racer = {}
		self.keys = []
	
	def createTable(self):
		current = ""
		for line in self.file_content:
			if line.startswith("Racer: "):
				current = line[-2]
				if current not in self.racer.keys():
					self.racer.update({current : Racer(current)})
					self.keys += [current]
					print("added racer {} to map".format(current))
			elif current != "":
				self.racer[current].parse(line)
	def score(self):
		scores = []
		lag_times = {}
		for key in self.keys:
			self.racer[key].score()
		lag_times.update({self.keys[0] : self.racer[self.keys[1]].effective_time - self.racer[self.keys[0]].effective_time})
		lag_times.update({self.keys[1] : self.racer[self.keys[0]].effective_time - self.racer[self.keys[1]].effective_time})
		return lag_times
		
		
	def plot(self):
		gates = {}
		for key in self.racer:
			gates.update({key : [0]})
			for i in range(1, len(self.racer[key].time)):
				if self.racer[key].passed_gate[i] > self.racer[key].passed_gate[i-1]:
					gates[key] += [255]
				else:
					gates[key] += [0]
		for key in self.racer:
			plt.subplot(811)
			plt.plot(self.racer[key].time, [odom.x for odom in self.racer[key].odometry])
			plt.title('x')
			
			plt.subplot(812)
			plt.plot(self.racer[key].time, [odom.y for odom in self.racer[key].odometry])
			plt.title('y')
			plt.subplot(813)
			plt.plot(self.racer[key].time, [odom.z for odom in self.racer[key].odometry])
			plt.title('z')
			
			plt.subplot(814)
			plt.plot(self.racer[key].time, self.racer[key].penalty)
			plt.title('penalty')
			
			plt.subplot(814)
			plt.plot(self.racer[key].time, self.racer[key].penalty)
			plt.title('penalty')
			
			plt.subplot(3,1,3)
			plt.plot([odom.x for odom in self.racer[key].odometry], [odom.y for odom in self.racer[key].odometry])
			plt.title('parametric')
			
			# plt.subplot(614)
			# plt.plot(self.racer[key].time, [odom.roll for odom in self.racer[key].odometry])
			# plt.title('roll')
			# plt.subplot(615)
			# plt.plot(self.racer[key].time, [odom.pitch for odom in self.racer[key].odometry])
			# plt.title('pitch')
			# plt.subplot(616)
			# plt.plot(self.racer[key].time, [odom.yaw for odom in self.racer[key].odometry])
			# plt.title('yaw')
		plt.figlegend([key for key in self.racer])
		plt.subplots_adjust(hspace=0.4)
		plt.show()
	
		
	# def getMaxLinearVelocity(self):
	
	# def get
	
	# def parseData(line_in_file):
		# if line_in_file.startswith("

	
	
	
	
e = Evaluator("race_log.log")
e.createTable()
print(e.score())
e.plot()

		