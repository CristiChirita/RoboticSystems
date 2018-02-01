#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,pi
from PyKDL import Rotation

class stdr_controller():
	def __init__(self):
		rospy.init_node('stdr_controller', anonymous=True)
		velocity_publisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)
		current_pose_subscriber = rospy.Subscriber('/robot0/odom', Odometry, self.current_callback)
		current_pose = Odometry()
		distance_tolerance = 0.01
		angular_tolerance = 5

	def current_callback(self, data):
		self.current_pose = data
	
	def run():
		rospy.sleep(1.0)
		
		while not rospy.is_shutdown():
			vel_msg = Twist()
			
			pose = self.current_pose.pose.pose
			position = pose.position
			orientation = pose.orientation
			theta = 2 * atan2(orientation.z, orientation.w) * 180 / pi
			
			rospy.loginfo('Current position, x: {}, y: {}, theta: {}'.format(position.x, position.y, theta)
	
	def setAngle(speed, angle, clockwise):
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
			velocity_publisher.publish(vel_msg)
			t1 = rospy.Time.now().to_sec()
			current_angle = angular_speed*(t1 - t0)
	
		vel_msg.angular.z = 0
		velocity_publisher.publish(vel_msg)
		rospy.spin()
	
	def move(speed, distance):
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 0
	
		t0 = rospy.Time.now().to_sec()
		current_distance = 0
	
		while (current_distance < distance):
			velocity_publisher.publish(vel_msg)
			t1=rospy.Time.now().to_sec()
			current_distance = speed * (t1-t0)
	
		vel_msg.linear.x = 0
		velocity_publisher.publish(vel_msg)

if __name__ == '__main__'
	try:
		x = stdr_controller()
		x.run()
	except: rospy.ROSInterruptException: pass