#!/usr/bin/python

import rospy
import time
import thread
import math

import moveit_commander
import moveit_msgs.msg

from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose
from visualization_msgs.msg import Marker

from robohub_object_tracking import *
from robohub_object_tracking_plugins import ArucoROSMarkersPlugin
from robohub_object_tracking.msg import TrackedObjectPose, TrackedObjectPoseList

rospy.init_node('panda_demo')

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)


ts = TrackingSystem()
ts.initialize_transform_listener()

plugin = ArucoROSMarkersPlugin()
ts.add_tracking_plugin(plugin)

to1 = TrackedObject(move_group.get_planning_frame())
to1_pose = PoseStamped()
to1_pose.pose.position = Point(*(0, 0, 0))
to1_pose.pose.orientation = Quaternion(*(0, 0, 0, 1))
to1.add_tracking_point("Aruco", "0", to1_pose)



pub = rospy.Publisher("/visualization_marker", Marker, queue_size=5)


def rotation_quaternions(q1, q2):
        w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z
        x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y
        y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x
        z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w

        q = Quaterion()
        q.w = w
        q.x = x
        q.y = y
        q.z = z


while not rospy.is_shutdown():
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Available Planning Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        raw_input("Press Enter to continue...")

        quat_offset = Quaternion()
        quat_offset.w = 1

        m_pose = to1.get_pose().pose

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position = m_pose.position
        pose_goal.orientation = rotation_quaternions(m_pose.orientation, quat_offset)

        print("Goal:")
        print(pose_goal)

        move_group.set_pose_target(pose_goal)

        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()