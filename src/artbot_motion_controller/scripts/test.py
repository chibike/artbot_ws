#!/usr/bin/env python

import sys
import copy
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

workarea_limits = {
	"x" : (0.2 ,  0.42),
	"y" : (-0.15,  0.15),
	"z" : (0.2 ,  0.5)
}

orientation_limits = {
	"r" : (0.0, 0.0),
	"p" : (math.pi/2.0, math.pi/2.0),
	"y" : (0.0, 0.0)
}


class MotionController(object):
	"""docstring for MotionController"""
	def __init__(self, name="motion_controller", group_name="manipulator", boundaries=(workarea_limits, orientation_limits)):
		self.robot = None
		self.scene = None
		self.group = None

		self.name = name

		self.group_name = group_name
		self.display_trajectory_publisher = None
		
		lower = quaternion_from_euler(boundaries[1]["r"][0], boundaries[1]["p"][0], boundaries[1]["y"][0])
		upper = quaternion_from_euler(boundaries[1]["r"][1], boundaries[1]["p"][1], boundaries[1]["y"][1])

		self.position_limits = boundaries[0]

		self.orientation_limits = {
			"x" : (lower[0], upper[0]),
			"y" : (lower[1], upper[1]),
			"z" : (lower[2], upper[2]),
			"w" : (lower[3], upper[3])
		}

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
		completed = False

		for i in range(5):
			completed = self.group.go(wait=wait)
			if completed: break

		rospy.loginfo("Finised with code {code}".format(code=completed))
		return completed

	def __constrain(self, x, x_min, x_max):
		return min(x_max, max(x, x_min))

	def __validate_pose(self, pose):
		pose.orientation.x = self.__constrain(pose.orientation.x, self.orientation_limits["x"][0], self.orientation_limits["x"][1])
		pose.orientation.y = self.__constrain(pose.orientation.y, self.orientation_limits["y"][0], self.orientation_limits["y"][1])
		pose.orientation.z = self.__constrain(pose.orientation.z, self.orientation_limits["z"][0], self.orientation_limits["z"][1])
		pose.orientation.w = self.__constrain(pose.orientation.w, self.orientation_limits["w"][0], self.orientation_limits["w"][1])

		pose.position.x = self.__constrain(pose.position.x, self.position_limits["x"][0], self.position_limits["x"][1])
		pose.position.y = self.__constrain(pose.position.y, self.position_limits["y"][0], self.position_limits["y"][1])
		pose.position.z = self.__constrain(pose.position.z, self.position_limits["z"][0], self.position_limits["z"][1])

		return pose

	def start(self):
		rospy.loginfo("Starting Commander")
		moveit_commander.roscpp_initialize(sys.argv)

		rospy.loginfo("Initializing node motion controller")
		rospy.init_node(self.name, anonymous=True)

		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group = moveit_commander.MoveGroupCommander(self.group_name)
		self.group.allow_replanning(True)

		self.group.set_planning_time(20)
		self.group.set_max_velocity_scaling_factor(0.5)
		self.group.set_max_acceleration_scaling_factor(0.5)

		rospy.loginfo("planning_time = {n}".format(n=self.group.get_planning_time()))
		rospy.loginfo("get_goal_position_tolerance = {n}".format(n=self.group.get_goal_position_tolerance()))
		rospy.loginfo("get_goal_orientation_tolerance = {n}".format(n=self.group.get_goal_orientation_tolerance()))

		# .set_goal_position_tolerance(0.0001)
		# .set_goal_orientation_tolerance(0.001)

		rospy.loginfo("Setting display publisher")
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)

	def stop(self):
		rospy.loginfo("Stopping the robot")
		self.group.stop()

		rospy.loginfo("Shutting down moveit")
		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)

	def publish_trajectory_display_info(self, plan):
		display_trajectory_msg = moveit_msgs.msg.DisplayTrajectory();
		display_trajectory_msg.trajectory_start = self.robot.get_current_state()
		display_trajectory_msg.trajectory.append(plan)
		self.display_trajectory_publisher.publish(display_trajectory_msg)

	def move_to(self, x=0.4, y=0.0, z=0.45, wait=True):
		self.target.position.x = x
		self.target.position.y = y
		self.target.position.z = z

		# print "before:", self.target

		self.target = self.__validate_pose(self.target)
		self.group.set_pose_target(self.target)

		# print "after:", self.target

		plan = self.group.plan()
		self.publish_trajectory_display_info(plan)

		self.__execute(wait)

	def follow(self, xs=[], ys=[], zs=[], interpolate_resolution=0.01, jump_threshold=0.0, wait=True):

		rospy.loginfo("Generating waypoints from cartesian info")
		waypoints = [ self.group.get_current_pose().pose ]

		# Set the internal state to the current state
		# right_arm.set_start_state_to_current_state()

		for i in xrange(min(len(xs), len(ys), len(zs))):
			self.target.position.x = xs[i]
			self.target.position.y = ys[i]
			self.target.position.z = zs[i]

			self.target = self.__validate_pose(self.target)
			waypoints.append(copy.deepcopy(self.target))

		# plan, fraction = self.group.compute_cartesian_path(waypoints, interpolate_resolution, jump_threshold)
		# self.publish_trajectory_display_info(plan)

		fraction = 0.0
		maxtries = 100
		attempts = 0

		self.group.set_start_state_to_current_state()

		while fraction < 1.0 and attempts < maxtries:
			plan, fraction = self.group.compute_cartesian_path(waypoints, interpolate_resolution, jump_threshold)
			self.publish_trajectory_display_info(plan)

			attempts += 1

		if fraction == 1.0:
			rospy.loginfo("Executing plan")
			completed = False
			for i in range(5):
				completed = self.group.execute(plan)
				if completed: break

			rospy.loginfo("Plan finished with status {completed}".format(completed=completed))
		else:
			rospy.loginfo("Path planning failed with only {fraction} success after {attempts} trails".format(attempts=attempts, fraction=fraction))


		# self.__execute(wait)

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


def test_move(motion_controller, sleep_time=5):
	rospy.loginfo("Waiting......")
	rospy.sleep(sleep_time)

	rospy.loginfo("Testing move_to ......")
	motion_controller.move_to()

def test_follow(motion_controller, sleep_time=5):
	rospy.loginfo("Waiting......")
	rospy.sleep(sleep_time)

	rospy.loginfo("Testing follow ......")
	xs = [0.40, 0.40, 0.40, 0.40]
	ys = [0.00, 0.00, 0.00, 0.00]
	zs = [0.45, 0.30, 0.20, 0.15]
	motion_controller.follow(xs, ys, zs)

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

def main():
	motion_controller = MotionController()
	motion_controller.start()

	sleep_time = 5

	test_move(motion_controller, sleep_time)
	test_follow(motion_controller, sleep_time)
	test_boundaries(motion_controller, sleep_time)
	test_joint_angles(motion_controller, sleep_time)
	

	rospy.loginfo("Waiting......")
	rospy.sleep(sleep_time)

	rospy.loginfo("Exiting.")
	motion_controller.stop()


if __name__ == '__main__':
	main()