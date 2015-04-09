#!/usr/bin/env python

import rospy
from common_files.msg import Sonar
from common_files.srv import ReadLidar
from common_files.msg import Motor

#last sensor msgs
last_sonar = Sonar()
last_x = 0.0
last_y = 0.0

#next motor msg to be published
next_motor_msg = Motor()

#motor count for turns
motor_count = 0

#arena constraints
x_min = -2.0
x_max = 2.0
y_max = 2.0

#sonar constraints
front_min = 30
front_clear = 60
side_min = 15

#state machine flags
already_ramped = False
slow_down = False

#STATE DEFINITIONS
def start():
	return 0

def move_north(): #relative to LIDAR
	return 1

def follow_left():
	return 2

def follow_left_brack():
	return 3

def follow_right():
	return 4

def follow_right_brack():
	return 5

def stop_state():
	return 6





def sonar_callback(data):	
	global last_sonar
	last_sonar = data

def motor_callback(event):
	global next_motor_msg
	global motor_count
#	print "Publishing motors: %f, %f" %(next_motor_msg.left, next_motor_msg.right)
	motor_pub = rospy.Publisher('motor_vals', Motor, queue_size=1)
	motor_pub.publish(next_motor_msg)
	motor_count += 1

def turn_left():
	#attempts to turn 90 degrees
	global next_motor_msg
	global motor_count
	next_motor_msg.left = -0.32
	next_motor_msg.right = 0.32
	local_count = 0
	motor_count = 0
	#about .7 seconds of this turns 90 degrees
	while motor_count < 7 and local_count < 1000000:
		local_count += 1
	next_motor_msg.left = 0.0
	next_motor_msg.right = 0.0

def turn_right():
        #attempts to turn 90 degrees
        global next_motor_msg
        global motor_count
        next_motor_msg.left = 0.32
        next_motor_msg.right = -0.32
        local_count = 0
        motor_count = 0
	#about .7 seconds of this turns 90 degrees
        while motor_count < 7 and local_count < 1000000:
                local_count += 1
        print "Motor count: %d, local count: %d" %(motor_count, local_count)
	next_motor_msg.left = 0.0
        next_motor_msg.right = 0.0

def ramp_up():
	global next_motor_msg
	global motor_count
	local_count = 0
	motor_count = 0
	#ramp up motor speed in five steps
	for i in range(1,6):
		motor_count = 0
		next_motor_msg.left = (i*.2)*.32	
		next_motor_msg.right = (i*.2)*.32
		local_count = 0
		while local_count < 1000000 and motor_count < 1:
			local_count += 1
			#wait for this step to be published
		#then go to next step for ramp up
	#end ramp up at.32
	global already_ramped
	already_ramped = True		

def move_forward():
	global next_motor_msg
	next_motor_msg.left = .32
	next_motor_msg.right = .32

def move_forward_slowly():
	global next_motor_msg
	next_motor_msg.left = .26
	next_motor_msg.right = .26

def stop():
	global next_motor_msg
	next_motor_msg.left = 0.0
	next_motor_msg.right = 0.0

#def get_results_in_xy():
#        rospy.wait_for_service('read_lidar')
#        try:
#                read_lidar_service = rospy.ServiceProxy('read_lidar', ReadLidar)
#                response = read_lidar_service()
#                return response
#        except rospy.ServiceException, e:
#                print "Service call failed: %s"%e

def state_machine_node():
	rospy.init_node('state_machine_node', anonymous=True)
	rospy.Timer(rospy.Duration(.1), motor_callback)
	global next_motor_msg
	next_motor_msg.left = 0.0
	next_motor_msg.right = 0.0
	state = start()
			
	while not rospy.is_shutdown():
		#execute current state
		global already_ramped
		global front_min
		global front_clear
		global side_min
		global slow_down
		global last_sonar
		if state == start():
			print "Starting program"
		elif state == move_north():
			print "Moving forward"
			if(already_ramped == False and slow_down == False):
				ramp_up()
				already_ramped = True
			elif(slow_down == True):
				move_forward_slowly()
			else:
				move_forward()
		elif state == follow_left():
			print "Moving forward, following right sensor"
			if(already_ramped == False and slow_down == False):
				ramp_up()
				already_ramped = True
			elif(slow_down == True):
                                move_forward_slowly()
			else:
				move_forward()
		elif state == follow_right():
			print "Moving forward, following left sensor"
			if(already_ramped == False and slow_down == False):
				ramp_up()
				already_ramped = True
			elif(slow_down == True):
				move_forward_slowly()
			else:
				move_forward()
		elif state == follow_left_brack():
			print "Cannot continue following; turning around"
			turn_right()
			turn_right()
		elif state == follow_right_brack():
			print "Cannot continue following; turning around"
			turn_left()
			turn_left()	
		else:
			stop()

		#choose next state
		if(state == start()):
			state = move_north()
			already_ramped = False
		elif(state == move_north()):
			#while moving north torward destination, analyze sensors
			print "Determine next state from move_north"	
			print "Sonars: %d, %d, %d" %(last_sonar.sonar1, last_sonar.sonar2, last_sonar.sonar3)	
			if(last_sonar.sonar1 > side_min and last_sonar.sonar2 > side_min and last_sonar.sonar3 > front_clear):
				#nothing detected, proceed forward
				state = move_north()
				already_ramped = True
				slow_down = False
			elif(last_sonar.sonar1 > side_min and last_sonar.sonar2 > side_min and last_sonar.sonar3 > front_min and last_sonar.sonar3 <= front_clear):
				#object detected in front distance, slow down
				#no objects on sides
				state = move_north()
				slow_down = True
				already_ramped = True
			elif(last_sonar.sonar1 < side_min and last_sonar.sonar2 > side_min):
				#object detected on left, right is clear
				turn_right() #turn right 90
				state = follow_right() #begin following obstacle
				already_ramped = False
			elif(last_sonar.sonar1 > side_min and last_sonar.sonar2 < side_min):
				#object detected on right, left is clear
				turn_left() #turn left 90
				state = follow_left()
				already_ramped = False
			else:
				#by default turn left
				turn_left()
				state = follow_left()
				already_ramped = False
		else:
			state = stop_state()

				


if __name__ == "__main__":
	try:
		state_machine_node()
	except rospy.ROSInterruptException: pass	
