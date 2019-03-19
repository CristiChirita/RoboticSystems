#!/usr/bin/env python
import rospy
import sys
import getopt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,pi
from PyKDL import Rotation
from std_msgs.msg import String

class stdr_controller():
	def __init__(self, argv):
		rospy.init_node('stdr_controller', anonymous=True)
		self.velocity_publisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)
		self.status_publisher = rospy.Publisher('/robot0/robot_status', String, queue_size=10)
		current_pose_subscriber = rospy.Subscriber('/robot0/odom', Odometry, self.current_callback)
		current_pose = Odometry()
		self.waypointData = []
		with open(argv[0]) as f:
			for line in f:
				data = line.split()
				a = []
				a.append(int(data[0]))
				a.append(int(data[1]))
				a.append(int(data[2]))
				self.waypointData.append(a)
		self.distance_tolerance = 0.01
		self.angular_tolerance = 5

	def current_callback(self, data):
		self.current_pose = data
	
	def run(self):
		rospy.sleep(1.0)
		i = 0
		while not rospy.is_shutdown():
			vel_msg = Twist()
			vel_msg.linear.x = 0
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = 0
			pose = self.current_pose.pose.pose
			position = pose.position
			orientation = pose.orientation
			theta = self.getTheta()
			if i == 0:
				self.status_publisher.publish('STARTED x:{}, y:{}, theta:{}'.format(position.x, position.y, theta))
			rospy.loginfo('Current position, x: {}, y: {}, theta: {}'.format(position.x, position.y, theta))
			
			try:
				# targetX = float(raw_input('Enter desired X coordinate: '))
				# targetY = float(raw_input('Enter desired Y coordinate: '))
				# targetAngle = float(raw_input('Enter desired angle: '))
				targetX = self.waypointData[i][0]
				targetY = self.waypointData[i][1]
				targetAngle = self.waypointData[i][2]
				i = i + 1
				# rospy.loginfo('Next position, x: {}, y: {}, theta: {}'.format(targetX, targetY, targetAngle))
				
				# Move along X axis
				if targetX - position.x > self.distance_tolerance: #target to the right
					# rospy.loginfo('Calculating x, to the right')
					if theta != 0: 
						if theta <= 180:
							# rospy.loginfo('Adjusting angle clockwise')
							self.setAngle(5, theta, True)
						else:
							# rospy.loginfo('Adjusting angle anticlockwise')
							self.setAngle(5, theta - 180, False)
					self.move(1, targetX -  position.x)
				if targetX - position.x < - self.distance_tolerance: #target to the left
					# rospy.loginfo('Calculating x, to the left')
					if theta != 180:
						if theta <= 180:
							# rospy.loginfo('Adjusting angle anticlockwise')
							self.setAngle(5, 180 - theta, False)
						else:
							# rospy.loginfo('Adjusting angle clockwise')
							self.setAngle(5, theta - 180, True)
					self.move(1, position.x - targetX)
				
				#Get new angle value
				theta = self.getTheta()
				
				#Move along Y axis
				# rospy.loginfo('Adjusting y')
				if targetY - position.y > self.distance_tolerance: #target higher
					if theta != 90:
						if (theta < 90):
							self.setAngle(5, 90 - theta, False)
						elif theta > 270:
							self.setAngle(5, 360 - theta + 90, False)
						elif theta <= 180:
							self.setAngle(5, theta - 90, True)
						else:
							self.setAngle(5, theta - 90, True)
					self.move(1, targetY - position.y)
				if targetY - position.y < - self.distance_tolerance: #target lower
					if theta != 270:
						if theta < 90:
							self.setAngle(5, theta + 90, True)
						elif theta > 270:
							self.setAngle(5, theta - 270, True)
						else:
							self.setAngle(5, 270 - theta, False)
					self.move(1, position.y - targetY)
				
				#Adjust to specified angle
				theta = self.getTheta()

				while targetAngle < 0:
					targetAngle = targetAngle + 360

				while targetAngle >= 360:
					targetAngle = targetAngle - 360

				if targetAngle - theta > self.angular_tolerance:
					if targetAngle - theta <= 180:
						self.setAngle(5, targetAngle - theta, False)
					else:
						self.setAngle(5, targetAngle - theta - 180, True)
				if targetAngle - theta < - self.angular_tolerance:
					if theta - targetAngle <= 180:
						self.setAngle(5, theta - targetAngle, True)
					else:
						self.setAngle(5, theta - targetAngle - 180, False)
				self.status_publisher.publish('WAYPOINT x:{}, y:{}, theta:{}'.format(targetX, targetY, targetAngle))		
			except ValueError:
				rospy.loginfo('Illegal value entered')
				raise
			except KeyboardInterrupt:
				sys.exit(0)
			except IndexError:
				rospy.loginfo('Finished')
				self.status_publisher.publish('FINISHED')
				sys.exit(0)
			
			vel_msg.linear.x = 0.0
			vel_msg.linear.y = 0.0
			vel_msg.linear.z = 0.0
			self.velocity_publisher.publish(vel_msg)
	
	def getTheta(self):
		pose = self.current_pose.pose.pose
		orientation = pose.orientation
		theta = 2 * atan2(orientation.z, orientation.w) * 180 / pi

		while theta < 0:
			theta = theta + 360.0

		while theta >= 360:
			theta = theta - 360
		return theta
	
	def setAngle(self, speed, angle, clockwise):
		
		if angle < self.angular_tolerance:
			return
		
		vel_msg = Twist()
		angular_speed = speed*2*pi/360
		relative_angle = angle*2*pi/360
		
		vel_msg.linear.x=0
		vel_msg.linear.y=0
		vel_msg.linear.z=0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
	
		if clockwise:
			vel_msg.angular.z = -abs(angular_speed)
		else:
			vel_msg.angular.z = abs(angular_speed)
		t0 = rospy.Time.now().to_sec()
		current_angle = 0
	
		while(current_angle < relative_angle):
			self.velocity_publisher.publish(vel_msg)
			t1 = rospy.Time.now().to_sec()
			current_angle = angular_speed*(t1 - t0)
	
		vel_msg.angular.z = 0
		self.velocity_publisher.publish(vel_msg)
	
	def move(self, speed, distance):
		vel_msg = Twist()
		vel_msg.linear.x = speed
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 0
	
		t0 = rospy.Time.now().to_sec()
		current_distance = 0
	
		while (current_distance < distance):
			self.velocity_publisher.publish(vel_msg)
			t1=rospy.Time.now().to_sec()
			current_distance = speed * (t1-t0)
	
		vel_msg.linear.x = 0
		self.velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
	try:
		x = stdr_controller(sys.argv[1:])
		x.run()
	except : 
		rospy.signal_shutdown("Finished")
		raise
