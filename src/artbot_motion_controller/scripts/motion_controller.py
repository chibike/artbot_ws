#!/usr/bin/env python

from test import *

import json
from std_msgs.msg import String

motion_controller = MotionController()
motion_controller.start() # init node is done here

def callback(data):
	rospy.loginfo("Received data")
	data = json.loads(data.data)

	rospy.loginfo("Data as {n} paths".format(n=len(data)))

def listener():
	rospy.Subscriber("path_as_string", String, callback)
	rospy.spin()

	motion_controller.stop()

def main():
	listener()

if __name__ == '__main__':
	main()