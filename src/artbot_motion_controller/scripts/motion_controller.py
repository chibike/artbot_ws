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

		self.pen_hover_offset = 0.04
		self.pen_writing_position = 0.281
		self.pen_hover_position = pen_writing_position + self.pen_hover_offset

		self.target = geometry_msgs.msg.Pose()
		foo = quaternion_from_euler(0, math.pi/2.0, 0)
		self.target.orientation.x = foo[0]
		self.target.orientation.y = foo[1]
		self.target.orientation.z = foo[2]
		self.target.orientation.w = foo[3]
		self.target.position.x = 0.46 # towards you
		self.target.position.y = 0.0  # sidewards
		self.target.position.z = 0.45 # height

	def __loginfo(self, name="", info=""):
		rospy.loginfo("MotionController::{name} -> {info}".format(name=name, info=info))

	def __logerror(self, name, info):
		rospy.logerror("MotionController::{name} -> {info}".format(name=name, info=info))

	def __execute(self, max_tries=5, wait=True):
		self.__loginfo("__execute", "Executing trajectory...")
		completed = False

		for i in range(max_tries):
			completed = self.group.go(wait=wait)
			if completed: break

		self.__loginfo("__execute", "Finised with code {code}".format(code=completed))
		return completed

	def __execute_plan(self, plan, max_tries=5, wait=True):
		self.__loginfo("__execute_plan", "Executing trajectory...")
		completed = False

		for i in range(max_tries):
			completed = self.group.execute(plan)
			if completed:
				break
			else:
				self.__loginfo("__execute_plan", "Could not completed plan, will retry")
				rospy.sleep(1)

		self.__loginfo("__execute_plan", "Finised with code {code}".format(code=completed))
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
		self.__loginfo("start", "Starting Commander")
		moveit_commander.roscpp_initialize(sys.argv)

		self.__loginfo("startstart", "Initializing node motion controller")
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

		self.__loginfo("start", "planning_time = {n}".format(n=self.group.get_planning_time()))
		self.__loginfo("start", "get_goal_position_tolerance = {n}".format(n=self.group.get_goal_position_tolerance()))
		self.__loginfo("start", "get_goal_orientation_tolerance = {n}".format(n=self.group.get_goal_orientation_tolerance()))

		self.__loginfo("start", "Setting display publisher")
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)

		rospy.sleep(7)
		self.home()

	def stop(self):
		self.__loginfo("stop", "Stopping the robot")
		self.group.stop()

		self.__loginfo("stop", "Shutting down moveit")
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

		self.target = self.__validate_pose(self.target)
		self.group.set_pose_target(self.target)

		plan = self.group.plan()
		plan = self.set_plan_speed(plan, speed)
		self.publish_trajectory_display_info(plan)

		#return self.__execute(wait)
		return self.__execute_plan(plan, wait)

	def manual_follow(self, xs=[], ys=[], zs=[], speed=1.0, wait=True):
		self.__loginfo("manual_follow", "Generating waypoints from cartesian info")
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

		self.__loginfo("follow", "Generating waypoints from cartesian info")
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
			self.__loginfo("follow", "Increasing plan speed")
			plan = self.set_plan_speed(plan, speed)

			self.__loginfo("follow", "Executing plan")
			completed = False
			for i in range(5):
				completed = self.group.execute(plan, wait)
				if completed:
					break
				else:
					self.__loginfo("follow", "Could not completed plan, will retry")
					rospy.sleep(1)

			self.__loginfo("follow", "Plan finished with status {completed}".format(completed=completed))
		else:
			self.__loginfo("follow", "Path planning failed with only {fraction} success after {attempts} trails".format(attempts=attempts, fraction=fraction))

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

	def change_pen(self, dropoff_xy_point, pickup_xy_point, default_z=0.45, speed_scaling=1.0, max_tries=3):
		success = False
		fraction = 0.0      # the completeness percentage
		number_of_steps = 5 # the required number of steps
		speed_scaling = max(min(1.0, speed_scaling), 0.0)

		self.__loginfo("change_pen", "Moving to home")
		for i in xrange(max_tries):
			if self.home():
				success = True
				break
		if not success:
			self.__loginfo("change_pen", "Could not go home")
			return success

		success = False
		self.__loginfo("change_pen", "Moving to dropoff point")
		x,y = pickup_xy_point
		for i in xrange(max_tries):
			if self.move_to(x,y,default_z):
				success = True
				break
		if not success:
			self.__loginfo("change_pen", "Could not move to dropoff point")
			return success

		# move downward slowly until pressure changes
		# open grippers
		# move to default_z position
		# move to pickup position
		# open grippers
		# move downward slowly until pressure changes
		# close grippers
		# move to default_z position

		self.__loginfo("change_pen", "Moving to home")
		for i in xrange(max_tries):
			if self.home():
				success = True
				break
		if not success:
			self.__loginfo("change_pen", "Could not go home")
			return success

		return fraction


	def calculate_z_depth(self, xy_point, default_z=0.45, increment=0.0025, target_pressure=0.1):
		# move to xy_point
		# current_z = default_z
		# for i in range(default_z, self.position_limits["z"][0], -increment):
		#     current_z -= increment
		#     move to xy_point
		#     if current_pressure >= target_pressure:
		#         break
		# self.pen_writing_position = current_z
		# move to default_z

		self.pen_hover_position = pen_writing_position + self.pen_hover_offset




