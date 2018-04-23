#!/usr/bin/env python

from motion_controller import *

import json
from std_msgs.msg import String

motion_controller = MotionController()
motion_controller.start() # init node is done here

def test_move(motion_controller, sleep_time=5):
	rospy.loginfo("Waiting......")
	rospy.sleep(sleep_time)

	rospy.loginfo("Testing move_to ......")
	return motion_controller.move_to()

def test_follow(motion_controller, sleep_time=5):
	rospy.loginfo("Waiting......")
	rospy.sleep(sleep_time)

	rospy.loginfo("Testing follow ......")
	xs = [0.46, 0.46, 0.46, 0.46]
	ys = [0.00, 0.00, 0.00, 0.00]
	zs = [0.45, 0.30, 0.20, 0.15]
	return motion_controller.follow(xs, ys, zs)

def test_boundaries(motion_controller, sleep_time=5):
	rospy.loginfo("Waiting......")
	rospy.sleep(sleep_time)

	rospy.loginfo("Testing boundaries x @ 5 ......")
	motion_controller.move_to(x=5, y=0, z=0)

	rospy.sleep(sleep_time * .5)

	rospy.loginfo("Testing boundaries x @ -5 ......")
	motion_controller.move_to(x=-5, y=0, z=0)

	rospy.loginfo("Waiting......")
	rospy.sleep(sleep_time)

	rospy.loginfo("Testing boundaries y @ 5 ......")
	motion_controller.move_to(x=0, y=5, z=0)

	rospy.sleep(sleep_time * .5)

	rospy.loginfo("Testing boundaries y @ -5 ......")
	motion_controller.move_to(x=0, y=-5, z=0)

	rospy.loginfo("Waiting......")
	rospy.sleep(sleep_time)

	rospy.loginfo("Testing boundaries z @ 5 ......")
	motion_controller.move_to(x=0, y=0, z=5)

	rospy.sleep(sleep_time * .5)

	rospy.loginfo("Testing boundaries z @ -5 ......")
	motion_controller.move_to(x=0, y=0, z=-5)

def test_joint_angles(motion_controller, sleep_time=5):
	rospy.loginfo("Waiting......")
	rospy.sleep(sleep_time)

	rospy.loginfo("Testing set_joint_angles ......")
	motion_controller.set_joint_angles([0]*6)

def end_effector_tests(motion_controller, sleep_time=5):
	def test_presure_without_pen():
		rospy.loginfo("end_effector_tests::test_presure_without_pen Homing")
		motion_controller.home()

		rospy.loginfo("end_effector_tests::test_presure_without_pen: Perfoming test ......")
		xs = [0.46, 0.46, 0.46, 0.46, 0.46, 0.46, 0.46]
		ys = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00]
		zs = [0.50, 0.45, 0.40, 0.35, 0.30, 0.27, 0.25]
		completed = motion_controller.manual_follow(xs, ys, zs)
		rospy.loginfo("end_effector_tests::test_presure_without_pen: Done......")

		rospy.loginfo("end_effector_tests::test_presure_without_pen Homing")
		motion_controller.home()

		return completed

	def test_presure_with_pen():
		rospy.loginfo("end_effector_tests::test_presure_with_pen Homing")
		motion_controller.home()

		rospy.loginfo("end_effector_tests::test_presure_with_pen: Perfoming test ......")
		xs = [0.46, 0.46, 0.46, 0.46, 0.46, 0.46]
		ys = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00]
		zs = [0.50, 0.45, 0.40, 0.35, 0.30, 0.29]
		completed = motion_controller.manual_follow(xs, ys, zs)
		rospy.loginfo("end_effector_tests::test_presure_with_pen: Done......")

		rospy.loginfo("end_effector_tests::test_presure_with_pen Homing")
		motion_controller.home()

		return completed

	def test_writing():
		rospy.loginfo("end_effector_tests::test_writing Homing")
		motion_controller.home()

		rospy.loginfo("end_effector_tests::test_writing Waiting for pen")
		rospy.sleep(10)

		rospy.loginfo("end_effector_tests::test_writing: moving to paper position ......")
		xs = [0.46, 0.46, 0.46, 0.46, 0.46, 0.46]
		ys = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00]
		zs = [0.50, 0.45, 0.40, 0.35, 0.30, 0.29]
		completed = motion_controller.manual_follow(xs, ys, zs)
		rospy.loginfo("end_effector_tests::test_writing: Done......")

		rospy.loginfo("end_effector_tests::test_writing Simulate writing")
		xs = [0.46,  0.30,  0.25, 0.30, 0.30, 0.25, 0.30, 0.46]
		ys = [0.00, -0.08, -0.1, -0.08, 0.08, 0.10, 0.08, 0.00]
		zs = [0.28] * len(xs)
		completed = motion_controller.manual_follow(xs, ys, zs)

		rospy.loginfo("end_effector_tests::test_writing: moving to home ......")
		xs = [0.46, 0.46, 0.46, 0.46, 0.46, 0.46]
		ys = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00]
		zs = [0.50, 0.45, 0.40, 0.35, 0.30, 0.29]
		completed = motion_controller.manual_follow_reverse(xs, ys, zs)
		rospy.loginfo("end_effector_tests::test_writing: Done......")

		rospy.loginfo("end_effector_tests::test_writing Take your pen")
		rospy.sleep(10)

		rospy.loginfo("end_effector_tests::test_writing Homing")
		motion_controller.home()

		return completed


	test_writing()
		


def main():
	motion_controller = MotionController()
	motion_controller.start()

	sleep_time = .1

	# test_move(motion_controller, sleep_time)
	# test_follow(motion_controller, sleep_time)
	# test_boundaries(motion_controller, sleep_time)
	# test_joint_angles(motion_controller, sleep_time)
	# test_move(motion_controller, sleep_time)

	end_effector_tests(motion_controller, sleep_time)
	

	rospy.loginfo("Waiting......")
	rospy.sleep(sleep_time)

	rospy.loginfo("Exiting.")
	motion_controller.stop()


if __name__ == '__main__':
	main()
