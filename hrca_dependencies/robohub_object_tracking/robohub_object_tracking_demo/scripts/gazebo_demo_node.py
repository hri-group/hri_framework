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

start_t = time.time()

# Create the system and object hear for ease of use
# Note that using python's reference system, the TrackingSystem
#  will end up modifying the original TrackedObject positions
# Thus, you can initialize them in one part of the code, and
#  pass them to the part that just needs to extract a query point pose
ts = TrackingSystem()

# Create a plugin to respond to receiving some outside information,
#  and transform it into tracked poses
# In this case, we just pass on the message, as it is in the correct form
plugin = ArucoROSMarkersPlugin("aruco_marker_publisher/markers")
ts.add_tracking_plugin(plugin)


# Give this object a tracking point 1m in -x
# Have this tracked point detected at the origin, with different rotations
# Result: the object should move around in a circle, while rotating. It will be a red square
# Give this a query point 1m above the object. This will be a blue dot, that should hover just ahead of the red square
m1 = Marker()
m1.header.frame_id = "map"
m1.ns = "demo"
m1.id = 1
m1.type = Marker.CYLINDER
m1.action = Marker.ADD
m1.scale.x = 0.25
m1.scale.y = 0.25
m1.scale.z = 0.1
m1.color.a = 1
m1.color.r = 1

to1 = TrackedObject("map", m1)
to1_pose = PoseStamped()
to1_pose.pose.position = Point(*(0.25, 0, 0))
to1_pose.pose.orientation = Quaternion(*(0, 0.7071, 0.7071, 0))

m1 = Marker()
m1.header.frame_id = "map"
m1.ns = "demo"
m1.id = 5
m1.type = Marker.SPHERE
m1.action = Marker.ADD
m1.scale.x = 0.1
m1.scale.y = 0.1
m1.scale.z = 0.1
m1.color.a = 1
m1.color.r = 1
m1.color.g = 1
m1.color.b = 1


to1.add_tracking_point("Aruco", "0", to1_pose, m1)

m3 = Marker()
m3.header.frame_id = "map"
m3.ns = "demo"
m3.id = 2
m3.type = Marker.SPHERE
m3.action = Marker.ADD
m3.scale.x = 0.1
m3.scale.y = 0.1
m3.scale.z = 0.1
m3.color.a = 1
m3.color.r = 1

qpose = PoseStamped()
qpose.pose.position = Point(*(1,0,0))
qpose.pose.orientation = Quaternion(*(0,0,0,1))
to1.add_query_point("in_pos_x", qpose, m3)


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
m3.color.g = 1

qpose = PoseStamped()
qpose.pose.position = Point(*(0,1,0))
qpose.pose.orientation = Quaternion(*(0,0,0,1))
to1.add_query_point("in_pos_y", qpose, m3)


m3 = Marker()
m3.header.frame_id = "map"
m3.ns = "demo"
m3.id = 4
m3.type = Marker.SPHERE
m3.action = Marker.ADD
m3.scale.x = 0.1
m3.scale.y = 0.1
m3.scale.z = 0.1
m3.color.a = 1
m3.color.b = 1

qpose = PoseStamped()
qpose.pose.position = Point(*(0,0,1))
qpose.pose.orientation = Quaternion(*(0,0,0,1))
to1.add_query_point("in_pos_z", qpose, m3)


ts.add_tracked_object(to1)

# Give this object a tracking point at its origin
# Have this tracked point detected moving around a 2m radius in a circle, with constant orientation
# Result: the object should move around the circle, without rotation; It will be a green square
# Details: It just happens to rotate in the opposite direction as the first object

    

if __name__ == "__main__":
    rospy.init_node("demo_node")

    pub = rospy.Publisher("visualization_marker", Marker, queue_size=5)

    ts.initialize_transform_listener()

    while not rospy.is_shutdown():
        rospy.sleep(0.2)
        array = ts.get_markers()

        for marker in array.markers:
            pub.publish(marker)
