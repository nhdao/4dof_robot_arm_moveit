#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16
from my_hw_interface.msg import rad_msg
from my_hw_interface.msg import step_msg

#Constants
GEAR_RATIO = 188.6
STEPS_PER_REVOLUTION = 200
PI = 3.14159
NUM_JOINTS = 5

#Flag
count = 0
joint_state = 0
messageCount = 0

#Node and topic name
nodeName = "publisher"
topicName = "information"
moveitTopicName = "/joints_to_arduino"

#Steps arrays
init_arr = [0] * NUM_JOINTS
prev_arr = [0] * NUM_JOINTS


rospy.init_node(nodeName, anonymous=True)
rate = rospy.Rate(4)

rev_data = step_msg()
total_data = step_msg()
total_data_prev = step_msg()

def callBackFunc(message):
	
	global count, joint_state, total_data_prev, total_data
	
	if(count == 0):
		init_arr[0] = message.joint1
		init_arr[1] = message.joint2
		init_arr[2] = message.joint3
		init_arr[3] = message.joint4
		init_arr[4] = message.joint5

		prev_arr[0] = message.joint1
		prev_arr[1] = message.joint2
		prev_arr[2] = message.joint3
		prev_arr[3] = message.joint4
		prev_arr[4] = message.joint5
						
	
	rev_data.joint1 = int(((message.joint1) * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * PI))
	rev_data.joint2 = int(((message.joint2) * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * PI))
	rev_data.joint3 = int(((message.joint3) * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * PI))
	rev_data.joint4 = int(((message.joint4) * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * PI))
	rev_data.joint5 = int(((message.joint5) * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * PI))
	
	if(count != 0):
		prev_arr[0] = message.joint1
		prev_arr[1] = message.joint2
		prev_arr[2] = message.joint3
		prev_arr[3] = message.joint4
		prev_arr[4] = message.joint5
			
	total_data.joint1 = rev_data.joint1
	total_data.joint2 = rev_data.joint2
	total_data.joint3 = rev_data.joint3
	total_data.joint4 = rev_data.joint4 
	total_data.joint5 = rev_data.joint5

	joint_state = 1
	count = 1

publisher1 = rospy.Publisher(topicName, step_msg, queue_size = 1)

subsciber1 = rospy.Subscriber(moveitTopicName, rad_msg, callBackFunc)

while not rospy.is_shutdown():
	if(total_data_prev.joint1 != total_data.joint1 or total_data_prev.joint2 != total_data.joint2
	or total_data_prev.joint3 != total_data.joint3 or total_data_prev.joint4 != total_data.joint4 
	or total_data_prev.joint4 != total_data.joint4):
		# if(joint_state == 1):
		joint_state = 0
		publisher1.publish(total_data)
		rospy.loginfo(total_data)
		rospy.loginfo(messageCount)
		messageCount += 1

		total_data_prev.joint1 = total_data.joint1
		total_data_prev.joint2 = total_data.joint2
		total_data_prev.joint3 = total_data.joint3
		total_data_prev.joint4 = total_data.joint4
		total_data_prev.joint5 = total_data.joint5
	rate.sleep()
