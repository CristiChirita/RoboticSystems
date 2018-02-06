#!/usr/bin/env python
import rospy
import sys
import getopt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,pi
from PyKDL import Rotation

class stdr_controller():
	def __init__(self, argv):
		rospy.init_node('stdr_controller', anonymous=True)
		self.velocity_publisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)
		current_pose_subscriber = rospy.Subscriber('/robot0/odom', Odometry, self.current_callback)
		current_pose = Odometry()
		inputfile = ""
		
		try:
			args = getopt.getopt(argv, '')
			for arg in args:
				inputfile = arg
				break
			# fp = open(inputfile)
		except getopt.GetoptError:
			#sys.exit(2)
			pass

		distance_tolerance = 0.01
		angular_tolerance = 5

	def current_callback(self, data):
		self.current_pose = data
	
	def run(self):
		rospy.sleep(1.0)
		
		while not rospy.is_shutdown():
			vel_msg = Twist()
			
			pose = self.current_pose.pose.pose
			position = pose.position
			orientation = pose.orientation
			theta = 2 * atan2(orientation.z, orientation.w) * 180 / pi
			
			rospy.loginfo('Current position, x: {}, y: {}, theta: {}'.format(position.x, position.y, theta))
			
			try:
				targetX = float(raw_input('Enter desired X coordinate: '))
				targetY = float(raw_input('Enter desired Y coordinate: '))
				targetAngle = float(raw_input('Enter desired angle: '))
				
				# Move along X axis
				if targetX - position.x > 0: #target to the right
					if theta != 0: 
						if theta >= 180:
							self.setAngle(5, theta, True)
						else:
							self.setAngle(5, theta - 180, False)
					self.move(1, targetX -  position.x)
				else: #target to the left
					if theta != 180:
						if theta >= 180:
							self.setAngle(5, 180 - theta, False)
						else:
							self.setAngle(5, theta - 180, True)
					self.move(1, position.x - targetX)
				
				#Get new angle value
				theta = self.getTheta()
				
				#Move along Y axis
				if targetY - position.y > 0: #target higher
					if theta != 90:
						if (theta < 90):
							self.setAngle(5, 90 - theta, False)
						elif theta > 270:
							self.setAngle(5, 360 - theta + 90, False)
						else:
							self.setAngle(5, theta - 90, True)
					self.move(1, targetY - position.y)
				else: #target lower
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
				if targetAngle > theta:
					self.setAngle(5, targetAngle - theta, False)
				else:
					self.setAngle(5, theta - targetAngle, True)
						
			except ValueError:
				rospy.loginfo('Illegal value entered')
			
			vel_msg.linear.x = 0.0
			vel_msg.linear.z = 0.0
			self.velocity_publisher.publish(vel_msg)
	
	def getTheta(self):
		pose = self.current_pose.pose.pose
		orientation = pose.orientation
		theta = 2 * atan2(orientation.z, orientation.w) * 180 / pi
		return theta
	
	def setAngle(self, speed, angle, clockwise):
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
	except rospy.ROSInterruptException: pass
