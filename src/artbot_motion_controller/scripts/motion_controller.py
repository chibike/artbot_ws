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
	"x" : (0.24 ,  0.46),
	"y" : (-0.15,  0.15),
	"z" : (0.25 ,  0.5)
}

orientation_limits = {
	"r" : (-10*(math.pi/180.0), -10*(math.pi/180.0)),
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

		self.pen_hover_position = 0.32
		self.pen_writing_position = 0.281
		#self.pen_writing_position = 0.28
		#self.pen_hover_position = 0.35
		#self.pen_writing_position = 0.28


		self.target = geometry_msgs.msg.Pose()
		foo = quaternion_from_euler(0, math.pi/2.0, 0)
		self.target.orientation.x = foo[0]
		self.target.orientation.y = foo[1]
		self.target.orientation.z = foo[2]
		self.target.orientation.w = foo[3]
		self.target.position.x = 0.46 # towards you
		self.target.position.y = 0.0  # sidewards
		self.target.position.z = 0.45 # height

	def __execute(self, max_tries=5, wait=True):
		rospy.loginfo("Executing trajectory...")
		completed = False

		for i in range(max_tries):
			completed = self.group.go(wait=wait)
			if completed: break

		rospy.loginfo("Finised with code {code}".format(code=completed))
		return completed

	def __execute_plan(self, plan, max_tries=5, wait=True):
		rospy.loginfo("Executing trajectory...")
		completed = False

		for i in range(max_tries):
			completed = self.group.execute(plan)
			if completed:
				break
			else:
				rospy.loginfo("Could not completed plan, will retry")
				rospy.sleep(1)

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
		self.group.set_goal_position_tolerance(0.001)
		self.group.set_max_velocity_scaling_factor(1)
		self.group.set_goal_orientation_tolerance(0.01)
		self.group.set_max_acceleration_scaling_factor(0.15)
		self.group.set_workspace([self.position_limits["x"][0], self.position_limits["y"][0], self.position_limits["z"][0], self.position_limits["x"][1], self.position_limits["y"][1], self.position_limits["z"][1]])

		rospy.loginfo("planning_time = {n}".format(n=self.group.get_planning_time()))
		rospy.loginfo("get_goal_position_tolerance = {n}".format(n=self.group.get_goal_position_tolerance()))
		rospy.loginfo("get_goal_orientation_tolerance = {n}".format(n=self.group.get_goal_orientation_tolerance()))

		rospy.loginfo("Setting display publisher")
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)

		rospy.sleep(7)
		self.home()

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

	def set_plan_speed(self, plan, speed=2.0):
		new_trajectory = moveit_msgs.msg.RobotTrajectory()
		new_trajectory.joint_trajectory = plan.joint_trajectory
		n_joints = len(plan.joint_trajectory.joint_names)
		n_points = len(plan.joint_trajectory.points)

		for i in range(n_points):
			new_trajectory.joint_trajectory.points[i].time_from_start = plan.joint_trajectory.points[i].time_from_start / speed
			#new_trajectory.joint_trajectory.points[i].time_from_start = rospy.Duration(5)
			new_trajectory.joint_trajectory.points[i].velocities = tuple([(j * speed) for j in plan.joint_trajectory.points[i].velocities])

		return new_trajectory

	def home(self):
		return self.move_to(x=0.46, y=0.0, z=0.45, speed=1, wait=True)

	def move_to(self, x=0.46, y=0.0, z=0.45, speed=2, wait=True):
		self.target.position.x = x
		self.target.position.y = y
		self.target.position.z = z

		# print "before:", self.target

		self.target = self.__validate_pose(self.target)
		self.group.set_pose_target(self.target)

		# print "after:", self.target

		plan = self.group.plan()

		# print "-"*30
		# print dir(plan)
		# print type(plan)
		# print "-"*30
		# print dir(self.group)
		# print type(self.group)

		plan = self.set_plan_speed(plan, speed)
		self.publish_trajectory_display_info(plan)

		#return self.__execute(wait)
		return self.__execute_plan(plan, wait)

	def manual_follow(self, xs=[], ys=[], zs=[], speed=1.0, wait=True):
		rospy.loginfo("Generating waypoints from cartesian info")
		counter = 0
		for i in range(min(len(xs), len(ys), len(zs))):
			if self.move_to(x=xs[i], y=ys[i], z=zs[i]):
				counter += 1

		return counter / float(min(len(xs), len(ys), len(zs)))

	def manual_follow_reverse(self, xs=[], ys=[], zs=[], speed=1.0, wait=True):
		xs.reverse(); ys.reverse(); zs.reverse()
		completed = self.manual_follow(xs, ys, zs)
		return completed

	def follow(self, xs=[], ys=[], zs=[], speed=1.0, interpolate_resolution=0.3, jump_threshold=0.0, wait=True):

		rospy.loginfo("Generating waypoints from cartesian info")
		# waypoints = [ self.group.get_current_pose().pose ]
		waypoints = []

		completed = False

		for i in xrange(min(len(xs), len(ys), len(zs))):
			self.target.position.x = xs[i]
			self.target.position.y = ys[i]
			self.target.position.z = zs[i]

			self.target = self.__validate_pose(self.target)
			waypoints.append(copy.deepcopy(self.target))

		fraction  = 0.0
		max_tries = 100
		attempts  = 0

		self.group.set_start_state_to_current_state()

		while (fraction < 1.0) and (attempts < max_tries):
			plan, fraction = self.group.compute_cartesian_path(waypoints, interpolate_resolution, jump_threshold, avoid_collisions=False)
			self.publish_trajectory_display_info(plan)

			attempts += 1

		if fraction == 1.0:
			rospy.loginfo("Increasing plan speed")
			plan = self.set_plan_speed(plan, speed)

			rospy.loginfo("Executing plan")
			completed = False
			for i in range(5):
				completed = self.group.execute(plan, wait)
				if completed:
					break
				else:
					rospy.loginfo("Could not completed plan, will retry")
					rospy.sleep(1)

			rospy.loginfo("Plan finished with status {completed}".format(completed=completed))
		else:
			rospy.loginfo("Path planning failed with only {fraction} success after {attempts} trails".format(attempts=attempts, fraction=fraction))

		return completed

	def set_joint_angle(self, joint_index=0, joint_angle=1.0, wait=True):
		self.group.clear_pose_targets()

		group_variable_values = self.group.get_current_joint_values()
		group_variable_values[joint_index] = joint_angle

		self.group.set_joint_value_target(group_variable_values)

		plan = self.group.plan()
		self.publish_trajectory_display_info(plan)

		return self.__execute(wait)

	def set_joint_angles(self, joint_angles, wait=True):
		self.group.clear_pose_targets()
		self.group.set_joint_value_target(joint_angles)

		plan = self.group.plan()
		self.publish_trajectory_display_info(plan)

		return self.__execute(wait)

	def get_joint_angles(self):
		return self.group.get_current_joint_values()

	def get_current_pose(self):
		return self.group.get_current_pose().pose


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
