#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float32MultiArray

def callback(data):
	graph = Float32MultiArray();
	graph.data = data.data

	for i in range(36):
		print(data.data[i])
	plt.plot(graph.data)
	plt.pause(0.05)
	
	plt.show()

if __name__ == '__main__':
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber("/graph", Float32MultiArray, callback);
	rospy.spin()
