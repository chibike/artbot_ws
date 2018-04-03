#!/usr/bin/env python

import sys
import copy
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)

#print "Done Starting"
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")


display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)



print "============ Waiting for RVIZ..."
#rospy.sleep(10)
print "============ Starting tutorial "

print "============ Reference frame: %s" % group.get_planning_frame()

print "============ Reference frame: %s" % group.get_end_effector_link()

print "============ Robot Groups:"
print robot.get_group_names()

print "============ Printing robot state"
print robot.get_current_state()
print "============"

print "============ Generating plan 1"
pose_target = geometry_msgs.msg.Pose()
foo = quaternion_from_euler(0, math.pi/2.0, 0)
pose_target.orientation.x = foo[0]
pose_target.orientation.y = foo[1]
pose_target.orientation.z = foo[2]
pose_target.orientation.w = foo[3]
#pose_target.orientation.w = 1.0
pose_target.position.x = 0.4  # towards you
pose_target.position.y = 0.0  # sidewards
pose_target.position.z = 0.45 # height
group.set_pose_target(pose_target)


plan1 = group.plan()

print "============ Waiting while RVIZ displays plan1..."
rospy.sleep(5)


print "============ Visualizing plan1"
display_trajectory = moveit_msgs.msg.DisplayTrajectory()

display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan1)
display_trajectory_publisher.publish(display_trajectory);

print "============ Waiting while plan1 is visualized (again)..."
rospy.sleep(5)


print "Starting excuetion"
# Uncomment below line when working with a real robot
group.go(wait=True)

