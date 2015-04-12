#!/usr/bin/env python

import rospy
import serial
from common_files.msg import Sonar
from common_files.msg import Motor

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

last_motor_msg = Motor()
last_write_time = 0
motor_received = 0

def motor_callback(data):
	global last_motor_msg
	global motor_received
	last_motor_msg = data
	motor_received = 1 #set flag
	motor_string = generate_string()
        ser.write(motor_string)
	print "motor callback!" 

def generate_string(): #generate string from motor message
        global last_motor_msg
        data = last_motor_msg
        direction = 0
        if(data.left >= 0.0 and data.right >= 0.0):
                #both forward
                direction = 0
        if(data.left >= 0.0 and data.right < 0.0):
                #left forward, right backward
                direction = 1
        if(data.left < 0.0 and data.right >= 0.0):
                #left backward, right forward
                direction = 2
        if(data.left < 0.0 and data.right < 0.0):
                #both backward
                direction = 3
        motor_data = (chr(direction), chr(int(abs(data.left*255))), chr(int(abs(data.right*255))), '\n')
	motor_string = ''.join(motor_data)
        return motor_string

def motor_timeout(event):
	global motor_received
	if(motor_received == 0):
		#timeout
		motor_data = [chr(4), chr(0), chr(0), '\n']
                motor_string = ''.join(motor_data)
                ser.write(motor_string)
		print "Serial Write Timeout!"
	else:
		motor_received = 0 #reset flag

def serial_node():
	pub = rospy.Publisher('sonar', Sonar, queue_size=1)
	sub = rospy.Subscriber('motor_vals', Motor, motor_callback)
	rospy.init_node('serial_node', anonymous=True)
	rospy.Timer(rospy.Duration(1.0), motor_timeout)

	while not rospy.is_shutdown():
		ser.flushInput()
		line = ser.readline()	
		if(len(line) == 3 or len(line) == 4):
			print ('Sonar: %d %d %d' %( ord(line[0]), ord(line[1]), ord(line[2]) ) )
			pub.publish( sonar1=ord(line[0]), sonar2=ord(line[1]), sonar3=ord(line[2]) )	
		
			
if __name__ == '__main__':
	try:
		serial_node()
	except rospy.ROSInterruptException: pass

