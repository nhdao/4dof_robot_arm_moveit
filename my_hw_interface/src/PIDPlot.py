#!/usr/bin/env python3

import rospy
from my_hw_interface.msg import rad_msg, degree_msg
import matplotlib.pyplot as plt

# Constants
PI = 3.14159
NUM_JOINTS = 5

# Node and topic name
nodeName = "publisher"
moveitTopicName = "/joints_to_arduino"
arduinoTopicName = "/arduino_feedback"

# ROS Initialization
rospy.init_node(nodeName, anonymous=True)
rate = rospy.Rate(1)  # Slow down for debugging

# Lists to store data for plotting
time_values = []
total_data_values = [[] for _ in range(NUM_JOINTS)]
fb_data_values = [[] for _ in range(NUM_JOINTS)]

# Callback function for '/joints_to_arduino' topic
def callBackFunc1(message):
    global time_values, total_data_values
    
    rev_data = [
        int((message.joint1 * 180) / PI),
        int((message.joint2 * 180) / PI),
        int((message.joint3 * 180) / PI),
        int((message.joint4 * 180) / PI),
        int((message.joint5 * 180) / PI),
    ]

    # Record time for plotting
    current_time = rospy.get_time()
    time_values.append(current_time)

    for i in range(NUM_JOINTS):
        total_data_values[i].append(rev_data[i])
        # Ensure fb_data_values[i] has the same length by appending None if necessary
        if len(fb_data_values[i]) < len(total_data_values[i]):
            fb_data_values[i].append(None)

    print(f'Received /joints_to_arduino message at time {current_time}: {rev_data}')
    print(f'Current total_data_values: {total_data_values}')
    print(f'Current time_values: {time_values}')

# Callback function for '/arduino_feedback' topic
def callBackFunc2(message):
    global fb_data_values
    
    fb_data = [
        message.joint1,
        message.joint2,
        message.joint3,
        message.joint4,
        message.joint5,
    ]

    for i in range(NUM_JOINTS):
        # Append fb_data to fb_data_values[i]
        fb_data_values[i].append(fb_data[i])
        # Ensure total_data_values[i] has the same length by appending None if necessary
        if len(total_data_values[i]) < len(fb_data_values[i]):
            total_data_values[i].append(None)

    print(f'Received /arduino_feedback message: {fb_data}')
    print(f'Current fb_data_values: {fb_data_values}')

# Subscribers for topics
subscriber1 = rospy.Subscriber(moveitTopicName, rad_msg, callBackFunc1)
subscriber2 = rospy.Subscriber(arduinoTopicName, degree_msg, callBackFunc2)

# Main loop to publish data and plot
plt.ion()  # Turn on interactive mode for matplotlib
fig, ax = plt.subplots(figsize=(10, 6))

while not rospy.is_shutdown():
    if time_values and total_data_values[0] and fb_data_values[0]:
        ax.clear()
        for i in range(NUM_JOINTS):
            if len(time_values) == len(total_data_values[i]) == len(fb_data_values[i]):
                ax.plot(time_values, total_data_values[i], label=f'Joint {i+1} Total Data')
                ax.plot(time_values, fb_data_values[i], label=f'Joint {i+1} Feedback Data')

        ax.set_xlabel('Time')
        ax.set_ylabel('Value')
        ax.set_title('Total Data and Feedback Data Over Time')
        ax.legend()
        ax.grid(True)
        plt.draw()
        plt.pause(0.001)

    rate.sleep()

plt.ioff()  # Turn off interactive mode
plt.show()  # Show the plot
