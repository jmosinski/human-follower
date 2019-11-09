#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
import numpy as np
from sklearn.cluster import DBSCAN
from scipy import optimize
import tf
import tf2_ros
from pykalman import KalmanFilter

class Person:
	#kalman filter from https://github.com/angusleigh/leg_tracker/blob/melodic/scripts/joint_leg_tracker.py
	# BSD 3-Clause License
	#Copyright (c) 2018, Angus Leigh
	#All rights reserved.
	def __init__(self, humanPosition):
		self.lastDetectionTime = 0
		self.position = humanPosition

		self.dist_travelled = 0.
		
		scan_frequency = 10
		delta_t = 1./scan_frequency
		std_process_noise = 0.05

		std_pos = std_process_noise
		std_vel = std_process_noise
		std_obs = 0.4
		var_pos = std_pos**2
		var_vel = std_vel**2
		var_obs_local = std_obs**2
		self.var_obs = (std_obs + 0.4)**2

		self.filtered_state_means = np.array([self.position.x, self.position.y, 0, 0])
		self.pos_x = self.position.x
		self.pos_y = self.position.y
		self.vel_x = 0
		self.vel_y = 0
		
		self.maxCircleRadius = 0

		self.filtered_state_covariances = 0.5*np.eye(4) 

        # Constant velocity motion model
		transition_matrix = np.array([[1, 0, delta_t,        0],
                                      [0, 1,       0,  delta_t],
                                      [0, 0,       1,        0],
                                      [0, 0,       0,        1]])

        # Oberservation model. Can observe pos_x and pos_y
		observation_matrix = np.array([[1, 0, 0, 0],
                                       [0, 1, 0, 0]])

		transition_covariance = np.array([[var_pos,       0,       0,       0],
                                          [      0, var_pos,       0,       0],
                                          [      0,       0, var_vel,       0],
                                          [      0,       0,       0, var_vel]])

		observation_covariance =  var_obs_local*np.eye(2)

		self.kf = KalmanFilter(
            transition_matrices=transition_matrix,
            observation_matrices=observation_matrix,
            transition_covariance=transition_covariance,
            observation_covariance=observation_covariance,
        )


	def update(self, humanPosition):

		observations = [humanPosition.x, humanPosition.y]

		self.filtered_state_means, self.filtered_state_covariances = (
            self.kf.filter_update(
                self.filtered_state_means,
                self.filtered_state_covariances,
                observations
            )
        )

		delta_dist_travelled = ((self.pos_x - self.filtered_state_means[0])**2 + (self.pos_y - self.filtered_state_means[1])**2)**(1./2.) 
		if delta_dist_travelled > 0.01:
			self.dist_travelled += delta_dist_travelled

		self.pos_x = self.filtered_state_means[0]
		self.pos_y = self.filtered_state_means[1]
		self.vel_x = self.filtered_state_means[2]
		self.vel_y = self.filtered_state_means[3]
		self.position.x = self.pos_x
		self.position.y = self.pos_y

		self.maxCircleRadius = math.sqrt((humanPosition.x - self.pos_x)**2 + (humanPosition.y - self.pos_y)**2)


class RosbotFollower:
	def __init__(self):
		rospy.init_node('follower')
		
		#filter scan readings, anything below minRange is treated as 
		#too close obstacle and rosbot stops
		self.minRange = 0.25
		self.maxRange = 2.4
		#readings have to be in front of rosbot in ranges
		#(maxAngle, Pi) u (-Pi, -maxAngle)
		self.maxAngle = 3*math.pi/4
		
		self.clusterizationMaxDistanceParam = 0.12
		self.clusterizationMinSamplesParam = 3
		
		#longer side of clusters bounding box have to be shorter than 
		#legWidth + dLegWidth
		self.legWidth = 0.12 #leg width read from scan readings
		self.dLegWidth = 0.12 #toleration to leg width
		#longer side to shorter side proportion read is 1.5 (not used
		# - not really reliable)
		
		self.speedPGain = 1./1.9
		self.angularSpeedPGain = 1.
		
		#distance/angle has to be larger for rosbot to start moving
		self.minHumanDistance = 0.3
		self.minHumanAngle = 0.05
		
		#if button message is not received within buttonTimeout seconds
		#rosbot is not allowed to move
		self.buttonTimeout = 1
		
		#max distance between two legs
		self.legDistanceThreshold = 0.5
		
		#max human rosbot distance in calibration
		self.calibrationDistance = 0.6
		
		#max distance between detected positions
		self.humanPositionChangeThreshold = 0.5
		
		#max time in seconds since last human detection
		self.detectionTimeout = 1

		#dest proportion = 1.6   up to 4
		self.destProportion = 1.6
		self.destArea = 0.004
		
		#scoring system weights
		self.distanceWeight = 20
		self.proportionWeight = 10
		self.areaWeight = 10
		self.widthDifferenceWeight = 1000

		self.maxScore = 200
		
		self.buttonState = False
		self.buttonTime = rospy.get_time()
		
		self.positionCalibration = True

		self.detectedPerson = Person(Point())
		
		self.legPub = rospy.Publisher('leg_to_follow', Marker, queue_size=10)
		self.ledPub = rospy.Publisher('/esp_remote/led_signal', Bool, queue_size=10)
		self.speedPub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		
		rate = rospy.Rate(10)  # 10hz
		self.scanSub = rospy.Subscriber("/scan", LaserScan, self.scanCallback)
		self.startSub = rospy.Subscriber("/esp_remote/start", Bool, self.startButtonCallback)
		self.calibrationSub = rospy.Subscriber("/esp_remote/calibrate", Bool, self.calibrationButtonCallback)
		
	def startButtonCallback(self, data):
		self.buttonState = data.data
		self.buttonTime = rospy.get_time()

	def calibrationButtonCallback(self, data):
		if data.data:
			self.positionCalibration = True

	def findClusters(self, scan):
		pointsList = np.zeros((0, 2))
		i = 0
		for r in scan.ranges:
			alfa = scan.angle_min + i * scan.angle_increment
			if r > self.minRange and r < self.maxRange:
				x = r * math.cos(alfa)
				y = r * math.sin(alfa)
				if (alfa > -math.pi and alfa < -self.maxAngle) or \
					(alfa > self.maxAngle and alfa < math.pi):
					pointsList = np.append(pointsList, [[x, y]], axis=0)
			elif r < self.minRange:
				if (alfa > -math.pi and alfa < -self.maxAngle) or \
					(alfa > self.maxAngle and alfa < math.pi):
					rospy.logerr("Obstacle detected")
					return [np.zeros((1, 2))]
			i += 1

		db = DBSCAN(eps=self.clusterizationMaxDistanceParam, 
					min_samples=self.clusterizationMinSamplesParam).fit(pointsList)
		core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
		core_samples_mask[db.core_sample_indices_] = True
		labels = db.labels_
		n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
		unique_labels = set(labels)
		clusterList = []

		for k in unique_labels:
			class_member_mask = (labels == k)
			xy = pointsList[class_member_mask & core_samples_mask]
			if xy.any():
				clusterList.append(xy)

		return clusterList
		
	def detectLegs(self, clusterList):
		i = 0
		sortedClusters = []
		sortedClustersDetails = []
		for cluster in clusterList:

			xMax = np.max(cluster[:, 0])
			xMin = np.min(cluster[:, 0])
			yMax = np.max(cluster[:, 1])
			yMin = np.min(cluster[:, 1])
		
			xDistance = xMax - xMin
			yDistance = yMax - yMin
			
			xMean = (xMax+xMin)/2
			yMean = (yMax+yMin)/2
			
			proportion = max(xDistance,yDistance)/min(xDistance, yDistance)
			area = xDistance*yDistance
			widthDifference = (max(xDistance,yDistance)-self.legWidth) - self.dLegWidth
			distanceFromRobot = math.sqrt(xMean**2 + yMean**2)

			score = 0
			score += distanceFromRobot * self.distanceWeight
			score += abs(proportion - self.destProportion) * self.proportionWeight
			score += abs(area - self.destArea) * self.areaWeight
			if widthDifference > 0:
				score += (abs(widthDifference) * self.widthDifferenceWeight)**2
			if score < self.maxScore:
				sortedClustersDetails.append([xMean, yMean, distanceFromRobot, proportion, area, widthDifference, score])
			
		sortedClustersDetails.sort(key=lambda x: x[6])
		
		for x in sortedClustersDetails:
			leg = Point()
			leg.x = x[0]
			leg.y = x[1]
			leg.z = x[6]
			sortedClusters.append(leg)
			
		return sortedClusters
		
	def detectHuman(self, sortedClusters):
		firstLeg = sortedClusters[0]
		firstLegDetected = False
		firstLeg.z = 0
		twoLegsDetected = False
		secondLeg = Point()
		humanPosition = Point()
		humanPositionTemp = Point()
		if len(sortedClusters) > 2:
			sortedClusters.sort(key=lambda x: math.sqrt((x.x-firstLeg.x)**2 + (x.y-firstLeg.y)**2) + x.z)
			secondLeg = sortedClusters[1]
			secondLeg.z = 0
			legDistance = math.sqrt((firstLeg.x - secondLeg.x)**2 + (firstLeg.y - secondLeg.y)**2)
			if legDistance < self.legDistanceThreshold:
				humanPositionTemp.x = (firstLeg.x+secondLeg.x)/2
				humanPositionTemp.y = (firstLeg.y+secondLeg.y)/2
				humanPositionTemp.z = 0
				twoLegsDetected = True
			else:
				humanPositionTemp = firstLeg
		else:
			humanPositionTemp = firstLeg
			
		if self.positionCalibration:
			r = math.sqrt( humanPositionTemp.x ** 2 + humanPositionTemp.y ** 2)
			if r < self.calibrationDistance and twoLegsDetected:
				firstLegDetected = True
				self.detectedPerson = Person(humanPositionTemp)
				humanPosition = humanPositionTemp
				self.detectedPerson.lastDetectionTime = rospy.get_time()
				self.positionCalibration = False
				led = Bool()
				led.data = True
				self.ledPub.publish(led)
		else:
			distanceChange = math.sqrt((self.detectedPerson.position.x - humanPositionTemp.x)**2 \
						+ (self.detectedPerson.position.y - humanPositionTemp.y)**2)
			if distanceChange < self.humanPositionChangeThreshold:
				self.detectedPerson.update(humanPositionTemp)
				humanPosition = self.detectedPerson.position
				self.detectedPerson.lastDetectionTime = rospy.get_time()
				firstLegDetected = True
			else: 
				if rospy.get_time() - self.detectedPerson.lastDetectionTime < self.detectionTimeout:
					humanPosition = self.detectedPerson.position
				else:
					led = Bool()
					led.data = False
					self.ledPub.publish(led)
					self.positionCalibration = True
		
		return (firstLeg, secondLeg, humanPosition, firstLegDetected, twoLegsDetected)
		
	def publishMarkers(self, firstLeg, secondLeg, humanPosition, firstLegDetected, twoLegsDetected, sortedClusters):
		legMarker = Marker()
		legMarker.header.frame_id = "laser"
		legMarker.ns = "person"
		legMarker.header.stamp = rospy.Time()
		legMarker.type = Marker.CYLINDER
		legMarker.action = Marker.ADD
		legMarker.pose.orientation.x = 0.0
		legMarker.pose.orientation.y = 0.0
		legMarker.pose.orientation.z = 0.0
		legMarker.pose.orientation.w = 1.0
		legMarker.scale.x = 0.04
		legMarker.scale.y = 0.04
		legMarker.scale.z = 0.04
		legMarker.color.a = 1.0
		legMarker.color.r = 0.0
		legMarker.color.g = 0.0
		legMarker.color.b = 1.0
		legMarker.lifetime = rospy.Duration(0.5)
		
		if not self.positionCalibration:
			#first leg
			if firstLegDetected:
				legMarker.id = 1
				legMarker.pose.position = firstLeg
				legMarker.pose.position.z = 0.02
				self.legPub.publish(legMarker)
			
			#second leg
			if twoLegsDetected:
				legMarker.id = 2
				legMarker.pose.position = secondLeg
				self.legPub.publish(legMarker)
			
			#human position
			legMarker.id = 3
			legMarker.scale.z = 0.2
			legMarker.pose.position = humanPosition
			legMarker.pose.position.z = 0.1
			legMarker.color.r = 1.0
			legMarker.color.b = 0.0
			self.legPub.publish(legMarker)
			

			legMarker.id = 4
			legMarker.color.a = 0.5
			legMarker.color.b = 1.0
			legMarker.pose.position = humanPosition
			legMarker.pose.position.z = 0
			legMarker.scale.x = 2*self.detectedPerson.maxCircleRadius
			legMarker.scale.y = 2*self.detectedPerson.maxCircleRadius
			legMarker.scale.z = 0.001
			self.legPub.publish(legMarker)


		legMarker = Marker()
		legMarker.ns = "legs"
		legMarker.header.frame_id = "laser"
		legMarker.header.stamp = rospy.Time()
		legMarker.type = Marker.SPHERE
		legMarker.action = Marker.ADD
		legMarker.pose.orientation.x = 0.0
		legMarker.pose.orientation.y = 0.0
		legMarker.pose.orientation.z = 0.0
		legMarker.pose.orientation.w = 1.0
		legMarker.scale.x = 0.04
		legMarker.scale.y = 0.04
		legMarker.scale.z = 0.04
		legMarker.color.a = 1.0
		legMarker.color.r = 0.0
		legMarker.color.g = 1.0
		legMarker.color.b = 0.0
		legMarker.lifetime = rospy.Duration(0.2)
		i = 1
		for x in sortedClusters:
			legMarker.id = i
			i += 1
			legMarker.pose.position.x = x.x
			legMarker.pose.position.y = x.y
			self.legPub.publish(legMarker)
	
	def controlRosbot(self, humanPosition):
		r = math.sqrt( humanPosition.x ** 2 + humanPosition.y ** 2)
		a = math.atan2(humanPosition.y, -humanPosition.x)
		
		if r > self.minHumanDistance:
			xSpeed = (r - self.minHumanDistance) * self.speedPGain
		else:
			xSpeed = 0
		
		if abs(a) > self.minHumanAngle:
			zAngularSpeed = -a * self.angularSpeedPGain
		else:
			zAngularSpeed = 0

		rosbotControl = Twist()
		if rospy.get_time() - self.buttonTime < self.buttonTimeout and \
				self.buttonState == True and not self.positionCalibration:
			rosbotControl.linear.x = xSpeed
			rosbotControl.angular.z = zAngularSpeed
			self.speedPub.publish(rosbotControl)
		else:
			self.speedPub.publish(rosbotControl)
	
	def scanCallback(self, scan):
		clusterList = self.findClusters(scan)
		
		if len(clusterList) == 0:
			rospy.logwarn("No clusters detected")
			if rospy.get_time() - self.detectedPerson.lastDetectionTime < self.detectionTimeout:
				humanPosition = self.detectedPerson.position
				self.controlRosbot(humanPosition)
				self.publishMarkers(Point(), Point(), humanPosition, False, False, [])
			else:
				led = Bool()
				led.data = False
				self.ledPub.publish(led)
				self.positionCalibration = True
				rosbotControl = Twist()
				self.speedPub.publish(rosbotControl)
			return
		elif (clusterList[0][0, 0] == 0) and (clusterList[0][0, 1] == 0):
			led = Bool()
			led.data = False
			self.ledPub.publish(led)
			self.positionCalibration = True
			rosbotControl = Twist()
			self.speedPub.publish(rosbotControl)
			return
			
		
		sortedClusters = self.detectLegs(clusterList)
		if len(sortedClusters) == 0:
			rospy.logwarn("No legs detected")
			if rospy.get_time() - self.detectedPerson.lastDetectionTime < self.detectionTimeout:
				humanPosition = self.detectedPerson.position
				self.controlRosbot(humanPosition)
				self.publishMarkers(Point(), Point(), humanPosition, False, False, [])
			else:
				led = Bool()
				led.data = False
				self.ledPub.publish(led)
				self.positionCalibration = True
				rosbotControl = Twist()
				self.speedPub.publish(rosbotControl)
			return
		
		(firstLeg, secondLeg, humanPosition, firstLegDetected, twoLegsDetected) = self.detectHuman(sortedClusters)
		self.publishMarkers(firstLeg, secondLeg, humanPosition, firstLegDetected, twoLegsDetected, sortedClusters)
		self.controlRosbot(humanPosition)
		
		
if __name__ == '__main__':
    try:
        rosbot = RosbotFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


