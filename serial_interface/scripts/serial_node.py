#!/usr/bin/env python

import rospy
import serial
from common_files.msg import Sonar
from common_files.msg import Motor

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

last_motor_msg = Motor()
last_motor_time = rospy.Time() #last time a Motor message was received
last_write_time = rospy.Time() #last time we wrote a message to serial



def callback(data):
	last_left = data.left
	last_right = data.right

def send_last_msg():
	direction = 0
	if(last_motor_msg.left >= 0.0 && last_motor_msg.right >= 0.0):
		#both forward
		direction = 0
	if(last_motor_msg.left >= 0.0 && last_motor_msg.right < 0.0):
		#left forward, right backward
		direction = 1
	if(last_motor_msg.left < 0.0 && last_motor_msg.right >= 0.0):
		#left backward, right forward
		direction = 2
	if(last_motor_msg.left < 0.0 && last_motor_msg.right < 0.0):
		#both backward
		direction = 3
	motor_write = bytes(4)
	motor_write[0] = direction
	motor_write[1]  = int(abs(last_motor_msg.left*255))
	motor_write[2] = int(abs(last_motor_msg.right*255))
	motor_write[3] = '\n'
	ser.write(motor_write)




def serial_node():
	pub = rospy.Publisher('sonar', Sonar, queue_size=10)
	rospy.init_node('serial_node', anonymous=True)

	while not rospy.is_shutdown():
		line = ser.readline()	
		print ('%d %d %d' %( ord(line[0]), ord(line[1]), ord(line[2]) ) )
		pub.publish( sonar1=ord(line[0]), sonar2=ord(line[1]), sonar3=ord(line[2]) )	

if __name__ == '__main__':
	try:
		serial_node()
	except rospy.ROSInterruptException: pass

