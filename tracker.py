import threading,time
from Core import MyQueue
from Tools.logSystem import *


MAX_LOGITUDINAL_SPEED = 10000			# Speed units
BREAKINGDISTANCE_TO_WAYPOINT = 0.5	#[m]
MAX_ALLOWED_DISTANCE_TO_PATH = 0.1	#[m]


class Tracker(threading.Thread):
	"""
	tracker receives data from pathplanner, position estimator and World Model Client
	drive commands are sent to ioHandler, general information is sent to world model
	client
	"""
	def __init__(self):
		"""the construct"""
		threading.Thread.__init__(self)
		#generate instance variables
		self.myQueue=MyQueue.MyQueue()
		self.queue_tracker = self.myQueue.queue_tracker
		self.queue_ioHandler = self.myQueue.queue_ioHandler
		self.queue_worldModelClient = self.myQueue.queue_worldModelClient
		self.remoteControlMode = False
		self.x_pos = []	# absolute position [m]
		self.y_pos = []	# absolute position [m]
		self.orientation = []	# see worldObject.py (clockwise dircetion with respect to pos. x-Axis) [rad]
		self.nextWaypoint=[]	# the next waypoint to be approached
		self.lastWaypoint=[]	# the last waypoint that was visited (to calculate current distance to planned path)
		# 
		# ---------------------------------> x
		# |
		# |
		# |
		# |
		# v  y
		# 
		# x and y describe the centerpoint of an object (in meters)
					
	def run(self):	
		"""called by threading.Thread.start()"""
		mylogger.info("Running.\n")
		while 1:
			sender, command, receiveData = self.queue_tracker.get()	# get new data from other threads
			
			if command == "quit":
				mylogger.info("Going down")
				break
					
			if sender == "wc":				# worldModelClient
				self.remoteControlMode = True	# go to remote Control mode
				if command == "C":			# cancel remote Control mode
					self.remoteControlMode = False
				### do localization move ,here is a circle
				elif command=="localization":
					self.queue_ioHandler.put(["tr",'motionCommand',[90, 500]])
					time.sleep(15)
					self.queue_ioHandler.put(["tr",'motionCommand',[0, 32767]])
				else:
					self.queue_ioHandler.put(["tr",'motionCommand',receiveData])	# send data to ioHandler
					
			if sender == "pe":		#position estimator
				if self.remoteControlMode == False:
					#do calculations here
					#self.queue_ioHandler.put(["tr",[commands]])	#send data to ioHandler
					self.x_pos, self.y_pos, self.orientation = receivedData	#[x, y, phi] m,m,rad
					# approach current pending waypoint
					nextWaypoint = pathCoordinates[0]
					# distance to next waypoint
					vectPosNextWaypoint = [(self.x_pos- nextWaypoint[0]), (self.y_pos-nextWaypoint[1])]
					distToNextWaypoint = math.sqrt((self.x_pos- nextWaypoint[0])**2 + (self.y_pos-nextWaypoint[1]))					
					# direction from current position  towards next waypoint (positive in clockwise direction)
					beta = math.atan2(vectPosNextWaypoint[1], vectPosNextWaypoint[0])
					#distance to expected path (line between "lastPoint" and "nextPoint")
					if lastWaypoint !=[]:	# already passed first waypoint
						lastWaypoint = [self.x_pos, self.y_pos]
					RichtungsvektorDerGeraden = [(lastWaypoint[0]-nextWaypoint[0]),(lastWaypoint[1]-nextWaypoint[1])]	# from nextWaypoint to last waypoint
					# we already know the vector from nextWaypoint to current postition (distToNextWaypoint)
					d = math.sqrt((RichtungsvektorDerGeraden[0]*vectPosNextWaypoint[1] - RichtungsvektorDerGeraden[1]*vectPosNextWaypoint[0]) / RichtungsvektorDerGeraden[0]**2 + RichtungsvektorDerGeraden[1]**2 )
					# calculate drive commands:
					# Longitudinal speed is higher, the highe the distance to the destination point is. 
					if (distToNextWaypoint > BREAKINGDISTANCE_TO_WAYPOINT):
						distFactor = 1
					else:
						distFactor = distToNextWaypoint/BREAKINGDISTANCE_TO_WAYPOINT
					longitudinalSpeed = MAX_LOGITUDINAL_SPEED * distFactor
					# commandRadius:
					'''desired radius of the trace (int16 -32768...32767) [] mm.
					straight on:	radius = 32767 or -32768, speed positive
					to the left:	radius smaller positve numbers, speed positive
					to the right:	radius smaller negative number, speed positive
					rotate to left (no longitudinal movement): radius = 0, positive speed
					rotate to right (no longitudinal movement): radius = 0, negative speed'''
					#angel Between Heading and next Wayoint
					waypointDirection = (beta - self.orientation)
					if (cos(waypointDirection) < 0):	#Heading further away than perpendicular to destination
						commandRadius =0
						if (sin(waypointDirection) <0):	# angel is negative
							commandSpeed = 5000	#turn left
						else:		#angel is positive
							commandSpeed = -5000 # turn right
					else:	# Heading at most 90 degrees different from destination direction
						if (d < MAX_ALLOWED_DISTANCE_TO_PATH):
							if sin(waypointDirection <0):	# angel is negative
								commandRadius = 0 	#turn left
					
					directionFactor = cos(waypointDirection)	# do not at all move, if perpendicular to destination...
					commandRadius = sin(waypointDirection)		# if sin(waypointdirection) is negative (anti clockwise angel), turn right
					# if the longitudinal speed is negative, 
					
					
			if sender == "pp":	#pathplanner
				#write new path...
				pathCoordinates = [[0.5, 0.5], [2, 2],[2, 0.5],[1, 0.5]]
				pass
				
			

		