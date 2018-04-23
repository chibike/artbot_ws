#!/usr/bin/env python

from test import *

import json
from std_msgs.msg import String

motion_controller = MotionController()
motion_controller.start() # init node is done here

x_page_length = 0.15
y_page_length = 0.20
x_offset = 0.28
y_offset = -0.5 * y_page_length

_image_row_to_a4_x_transform = lambda row : (row * x_page_length / 480.0) + x_offset
_image_col_to_a4_y_transform = lambda col : (col * y_page_length / 640.0) + y_offset

def shutdown():
	rospy.loginfo("Waiting......")
	rospy.sleep(5)

	rospy.loginfo("Exiting.")
	motion_controller.stop()

def callback(data):
	# end_effector_tests(motion_controller, .1)
	# return

	paths = json.loads(data.data)
	number_of_paths = len(paths)

	rospy.loginfo("motion_controller:: received data as {n} paths".format(n=number_of_paths))

	rospy.loginfo("motion_controller:: Moving to home position")
	if not motion_controller.home():
		rospy.loginfo("motion_controller:: ERROR: could not go home")
		return

	z_down = 0.28
	z_up   = 0.35
	index  = 0
	for path in paths:
		rows, cols = list(zip(*path))
		xs = list(map(_image_row_to_a4_x_transform, rows))
		ys = list(map(_image_col_to_a4_y_transform, cols))
		zs = list([z_down]*len(xs))

		xs = [xs[0]] + xs + [ys[-1]]
		ys = [ys[0]] + ys + [ys[-1]]
		zs = [z_up]  + zs + [z_up]

		completed = motion_controller.manual_follow(xs, ys, zs)

		if not completed:
			rospy.loginfo("motion_controller:: ERROR: could not move to point in path")
			rospy.loginfo("motion_controller:: Skipping path {path_index}".format(path_index=index))
			continue
		
		index += 1

	rospy.loginfo("motion_controller:: Moving to home position")
	if not motion_controller.home():
		rospy.loginfo("motion_controller:: ERROR: could not go home")
		return

def listener():
	rospy.Subscriber("path_as_string", String, callback)
	rospy.spin()

	motion_controller.stop()

if __name__ == '__main__':
	rospy.on_shutdown(shutdown)
	listener()

	end_effector_tests(motion_controller, .1)

'''
if not motion_controller.follow(xs, ys, zs * len(path)) :
			rospy.loginfo("ERROR: could not execute path")
			rospy.loginfo("Skipping path {index}".format(index=index))
'''