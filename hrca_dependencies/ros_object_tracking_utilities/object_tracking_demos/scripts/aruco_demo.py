#!/usr/bin/python

import rospy
import time
import thread
import math

from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose
from visualization_msgs.msg import Marker

from robohub_object_tracking import *
from robohub_object_tracking_plugins import ArucoROSMarkersPlugin
from robohub_object_tracking.msg import TrackedObjectPose, TrackedObjectPoseList

rospy.init_node('aruco_demo')

start_t = time.time()

# Create the system and object hear for ease of use
# Note that using python's reference system, the TrackingSystem
#  will end up modifying the original TrackedObject positions
# Thus, you can initialize them in one part of the code, and
#  pass them to the part that just needs to extract a query point pose
ts = TrackingSystem()
ts.initialize_transform_listener()

# Create a plugin to respond to receiving some outside information,
#  and transform it into tracked poses
# In this case, we just pass on the message, as it is in the correct form
plugin = ArucoROSMarkersPlugin()
ts.add_tracking_plugin(plugin)


# Give this object a tracking point 1m in -x
# Have this tracked point detected at the origin, with different rotations
# Result: the object should move around in a circle, while rotating. It will be a red square
# Give this a query point 1m above the object. This will be a blue dot, that should hover just ahead of the red square
to1 = TrackedObject("world")
to1_pose = PoseStamped()
to1_pose.pose.position = Point(*(-0.07, 0, 0.08))
to1_pose.pose.orientation = Quaternion(*(0, 0, 1, 0))
to1.add_tracking_point("Aruco", "0", to1_pose)

qpose = PoseStamped()
qpose.pose.position = Point(*(-0.07,0,0.08))
qpose.pose.orientation = Quaternion(*(0,0,0,1))
to1.add_query_point("marker_point", qpose)

qpose = PoseStamped()
qpose.pose.position = Point(*(0,0,0))
qpose.pose.orientation = Quaternion(*(0,0,0,1))
to1.add_query_point("test_point", qpose)

ts.add_tracked_object(to1)


to2 = TrackedObject("world")
to2_pose = PoseStamped()
to2_pose.pose.position = Point(*(0, 0, 0.215))
to2_pose.pose.orientation = Quaternion(*(0, 0, 0, 1))
to2.add_tracking_point("Aruco", "256", to2_pose)

qpose = PoseStamped()
qpose.pose.position = Point(*(0,0,0.215))
qpose.pose.orientation = Quaternion(*(0,0,0,1))
to2.add_query_point("marker_point", qpose)

qpose = PoseStamped()
qpose.pose.position = Point(*(0,0,0.12))
qpose.pose.orientation = Quaternion(*(0,0,0,1))
to2.add_query_point("paper_center", qpose)

qpose = PoseStamped()
qpose.pose.position = Point(*(0,0,0))
qpose.pose.orientation = Quaternion(*(0,0,0,1))
to2.add_query_point("origin", qpose)

ts.add_tracked_object(to2)


period = 0.1

pub = rospy.Publisher("/visualization_marker", Marker, queue_size=5)

# Piece of Paper
m1 = Marker()
m1.header.frame_id = "world"
m1.ns = ""
m1.id = 1
m1.type = Marker.CUBE
m1.action = Marker.ADD
m1.scale.x = 0.2159
m1.scale.y = 0.001
m1.scale.z = 0.2794
m1.color.a = 1
m1.color.r = 1
m1.color.g = 1
m1.color.b = 1


# Aruco location
m2 = Marker()
m2.header.frame_id = "world"
m2.ns = "demo"
m2.id = 2
m2.type = Marker.SPHERE
m2.action = Marker.ADD
m2.scale.x = 0.03
m2.scale.y = 0.03
m2.scale.z = 0.03
m2.color.a = 1
m2.color.r = 0.1
m2.color.g = 0.1
m2.color.b = 0.1

# Some interesting point
m3 = Marker()
m3.header.frame_id = "world"
m3.ns = "demo"
m3.id = 3
m3.type = Marker.SPHERE
m3.action = Marker.ADD
m3.scale.x = 0.1
m3.scale.y = 0.1
m3.scale.z = 0.1
m3.color.a = 1
m3.color.r = 1
m3.color.g = 1
m3.color.b = 0





# Thin Piece of Paper
m4 = Marker()
m4.header.frame_id = "world"
m4.ns = ""
m4.id = 4
m4.type = Marker.CUBE
m4.action = Marker.ADD
m4.scale.x = 0.11
m4.scale.y = 0.001
m4.scale.z = 0.2794
m4.color.a = 1
m4.color.r = 1
m4.color.g = 1
m4.color.b = 1


# Aruco location
m5 = Marker()
m5.header.frame_id = "world"
m5.ns = "demo"
m5.id = 5
m5.type = Marker.SPHERE
m5.action = Marker.ADD
m5.scale.x = 0.03
m5.scale.y = 0.03
m5.scale.z = 0.03
m5.color.a = 1
m5.color.r = 0.1
m5.color.g = 0.1
m5.color.b = 0.1

'''
# Some interesting point
m3 = Marker()
m3.header.frame_id = "map"
m3.ns = "demo"
m3.id = 3
m3.type = Marker.SPHERE
m3.action = Marker.ADD
m3.scale.x = 0.1
m3.scale.y = 0.1
m3.scale.z = 0.1
m3.color.a = 1
m3.color.r = 1
m3.color.g = 1
m3.color.b = 0
'''
while not rospy.is_shutdown():

        m1.pose = to1.get_pose().pose
        m1.header.stamp = rospy.Time.now()
        pub.publish(m1)
        
        m2.pose = to1.get_query_point_pose("marker_point").pose
        m2.header.stamp = rospy.Time.now()
        pub.publish(m2)

        m3.pose = to1.get_query_point_pose("test_point").pose
        m3.header.stamp = rospy.Time.now()
        pub.publish(m3)


        m4.pose = to2.get_query_point_pose("paper_center").pose
        pub.publish(m4)

        m5.pose = to2.get_query_point_pose("marker_point").pose
        pub.publish(m5)

        print(m5.pose)

        rospy.sleep(period)
