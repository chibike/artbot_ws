#!/usr/bin/env python

import sys
import copy
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

class MotionController(object):
	"""docstring for MotionController"""
	def __init__(self, group_name="manipulator"):
		self.robot = None
		self.scene = None
		self.group = None

		self.group_name = group_name
		self.display_trajectory_publisher = None

		self.target = geometry_msgs.msg.Pose()
		foo = quaternion_from_euler(0, math.pi/2.0, 0)
		self.target.orientation.x = foo[0]
		self.target.orientation.y = foo[1]
		self.target.orientation.z = foo[2]
		self.target.orientation.w = foo[3]
		self.target.position.x = 0.4  # towards you
		self.target.position.y = 0.0  # sidewards
		self.target.position.z = 0.45 # height

	def __execute(self, wait=True):
		rospy.loginfo("Executing trajectory...")
		self.group.go(wait=wait)

	def start(self):
		rospy.loginfo("Starting Commander")
		moveit_commander.roscpp_initialize(sys.argv)

		rospy.loginfo("Initializing node motion controller")
		rospy.init_node('motion_controller', anonymous=True)

		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group = moveit_commander.MoveGroupCommander(self.group_name)

		rospy.loginfo("Setting display publisher")
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)

	def publish_trajectory_display_info(self, plan):
		display_trajectory_msg = moveit_msgs.msg.DisplayTrajectory();
		display_trajectory_msg.trajectory_start = self.robot.get_current_state()
		display_trajectory_msg.trajectory.append(plan)
		self.display_trajectory_publisher.publish(display_trajectory_msg)

	def move_to(self, x=0.4, y=0.0, z=0.45, wait=True):
		self.target.position.x = x
		self.target.position.y = y
		self.target.position.z = z

		self.group.set_pose_target(self.target)

		plan = self.group.plan()
		self.publish_trajectory_display_info(plan)

		self.__execute(wait)

	def follow(self, xs=[], ys=[], zs=[], interpolate_resolution=0.01, jump_threshold=0.0, wait=True):
		rospy.loginfo("Generating waypoints from cartesian info")
		waypoints = [ self.group.get_current_pose().pose ]

		for i in xrange(min(len(xs), len(ys), len(zs))):
			self.target.position.x = xs[i]
			self.target.position.y = ys[i]
			self.target.position.z = zs[i]

			waypoints.append(copy.deepcopy(self.target))

		plan, fraction = self.group.compute_cartesian_path(waypoints, interpolate_resolution, jump_threshold)
		self.publish_trajectory_display_info(plan)

		self.__execute(wait)

	def set_joint_angle(self, joint_index=0, joint_angle=1.0, wait=True):
		self.group.clear_pose_targets()

		group_variable_values = self.group.get_current_joint_values()
		group_variable_values[joint_index] = joint_angle

		self.group.set_joint_value_target(group_variable_values)

		plan = self.group.plan()
		self.publish_trajectory_display_info(plan)

		self.__execute(wait)

	def set_joint_angles(self, joint_angles, wait=True):
		self.group.clear_pose_targets()
		self.group.set_joint_value_target(joint_angles)

		plan = self.group.plan()
		self.publish_trajectory_display_info(plan)

		self.__execute(wait)

	def get_joint_angles(self):
		return self.group.get_current_joint_values()

	def get_current_pose(self):
		return self.group.get_current_pose().pose


def main():
	motion_controller = MotionController()
	motion_controller.start()

	rospy.loginfo("Waiting......")
	rospy.sleep(10)

	rospy.loginfo("Testing move_to ......")
	motion_controller.move_to()

	rospy.loginfo("Waiting......")
	rospy.sleep(10)

	rospy.loginfo("Testing follow ......")
	xs = [0.40, 0.40, 0.40]
	ys = [0.00, 0.00, 0.00]
	zs = [0.45, 0.30, 0.45]
	motion_controller.follow(xs, ys, zs)

	rospy.loginfo("Waiting......")
	rospy.sleep(10)

	rospy.loginfo("Testing set_joint_angles ......")
	motion_controller.set_joint_angles([0]*6)

	rospy.loginfo("Waiting......")
	rospy.sleep(10)

	rospy.loginfo("Exiting.")


if __name__ == '__main__':
	main()