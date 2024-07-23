#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16
from joint_listener.msg import steps_msg

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
moveitTopicName = "/move_group/fake_controller_joint_states"

#Steps arrays
init_arr = [0] * NUM_JOINTS
prev_arr = [0] * NUM_JOINTS


rospy.init_node(nodeName, anonymous=True)
rate = rospy.Rate(20)

rev_data = steps_msg()
total_data_prev = steps_msg()
total_data = steps_msg()

def callBackFunc(message):
	
	global count, total_data, total_data_prev, messageCount
	
	#if total_data is None:
	#	total_data.joint1 = int((message.position[1] * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * PI))
	#	total_data.joint2 = int((message.position[2] * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * PI))
	#	total_data.joint3 = int((message.position[3] * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * PI))
	#	total_data.joint4 = int((message.position[4] * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * PI))
	#	total_data.joint5 = int((message.position[0] * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * PI))
	
	if(count == 0):
		for i in range(0, NUM_JOINTS):
			init_arr[i] = message.position[i]
			prev_arr[i] = message.position[i]
						
	
	# rev_data.joint5 = int(((message.position[0] - prev_arr[0]) * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * PI))
	# rev_data.joint1 = int(((message.position[1] - prev_arr[1]) * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * PI))
	# rev_data.joint2 = int(((message.position[2] - prev_arr[2]) * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * PI))
	# rev_data.joint3 = int(((message.position[3] - prev_arr[3]) * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * PI))
	# rev_data.joint4 = int(((message.position[4] - prev_arr[4]) * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * PI))

	rev_data.joint5 = int(((message.position[0]) * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * PI))
	rev_data.joint1 = int(((message.position[1]) * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * PI))
	rev_data.joint2 = int(((message.position[2]) * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * PI))
	rev_data.joint3 = int(((message.position[3]) * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * PI))
	rev_data.joint4 = int(((message.position[4]) * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * PI))
	
	if(count != 0):
		for i in range(0, NUM_JOINTS):
			prev_arr[i] = message.position[i]
			
	total_data.joint1 = rev_data.joint1
	total_data.joint2 = rev_data.joint2
	total_data.joint3 = rev_data.joint3
	total_data.joint4 = rev_data.joint4
	total_data.joint5 = rev_data.joint5

	publisher1.publish(total_data)
	rospy.loginfo(total_data)
	messageCount += 1
	rospy.loginfo(messageCount)

	count = 1

	

publisher1 = rospy.Publisher(topicName, steps_msg, queue_size = 10)

subsciber1 = rospy.Subscriber(moveitTopicName, JointState, callBackFunc)

rospy.spin()

# while not rospy.is_shutdown():
# 	if(total_data_prev.joint1 != total_data.joint1 or total_data_prev.joint2 != total_data.joint2
# 	or total_data_prev.joint3 != total_data.joint3 or total_data_prev.joint4 != total_data.joint4 
# 	or total_data_prev.joint4 != total_data.joint4):
# 		publisher1.publish(total_data)
# 		rospy.loginfo(total_data)
# 		rospy.loginfo(messageCount)
# 		messageCount += 1

# 		total_data_prev.joint1 = total_data.joint1
# 		total_data_prev.joint2 = total_data.joint2
# 		total_data_prev.joint3 = total_data.joint3
# 		total_data_prev.joint4 = total_data.joint4
# 		total_data_prev.joint5 = total_data.joint5
# 	rate.sleep()
