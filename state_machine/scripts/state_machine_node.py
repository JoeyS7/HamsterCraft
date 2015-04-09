#!/usr/bin/env/ python

import sys
import rospy
from common_files.msg import Sonar
from common_files.srv import ReadLidar
from common_files.msg import Motor

last_sonar = Sonar()
last_x = 0.0
last_y = 0.0
motor_pub = rospy.Publisher('motor_vals', Motor, queue_size=1) 


def sonar_callback(data):	
	global last_sonar
	last_sonar = data

def motor_callback():
	global next_motor_msg
	global motor_pub
	print "Publishing motors: %f, %f" %(next_motor_msg.left, next_motor_msg.right)
	motor_pub.publish(next_motor_msg)

def turn_left():
	#attempts to turn 90 degrees
	global next_motor_msg
	next_motor_msg.left = .25
	next_motor_msg.right = .25
	rospy.sleep(.75)
	next_motor_msg.left = 0.0
	next_motor_msg.right = 0.0
	
def get_results_in_xy():
        rospy.wait_for_service('read_lidar')
        try:
                read_lidar_service = rospy.ServiceProxy('read_lidar', ReadLidar)
                response = read_lidar_service()
                return response
        except rospy.ServiceException, e:
                print "Service call failed: %s"%e

def state_machine_node():
	rospy.init_node('state_machine_node', anonymous=True)
	rospy.Timer(rospy.Duration(.1), motor_callback)

	state = 0
		
	while not rospy.is_shutdown():
		if state == 0:
			turn_left()
		else:
			rospy.sleep(2.0)


if __name__ == "__main__":
	try:
		state_machine_node()
	except rospy.ROSInteruptException: pass





	
