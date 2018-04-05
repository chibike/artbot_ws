#! /usr/bin/env python

import rospy
import time

import actionlib
import image_processing_pkg.msg

def state_change_client():
	client = actionlib.SimpleActionClient('image_processing_node', image_processing_pkg.msg.StateChangeRequestAction)

	client.wait_for_server()

	wait_time = 10

	goal = image_processing_pkg.msg.StateChangeRequestGoal(state="state_normal");
	client.send_goal(goal)
	print client.get_result()

	time.sleep(wait_time)

	goal = image_processing_pkg.msg.StateChangeRequestGoal(state="state_render_preview");
	client.send_goal(goal)
	print client.get_result()

	time.sleep(wait_time)

	goal = image_processing_pkg.msg.StateChangeRequestGoal(state="state_capture_image");
	client.send_goal(goal)
	print client.get_result()

	time.sleep(wait_time)

	goal = image_processing_pkg.msg.StateChangeRequestGoal(state="state_normal");
	client.send_goal(goal)
	print client.get_result()

	return client.get_result()

def main():
	rospy.init_node("state_change_client")
	result = state_change_client()
	print ("Yeah")

if __name__ == '__main__':
	main()