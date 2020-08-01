#!/usr/bin/python

import rospy
import time
import thread
import math

from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose
from visualization_msgs.msg import Marker

from robohub_object_tracking import *
from robohub_object_tracking_plugins import CustomMsgPassthroughPlugin
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
plugin = CustomMsgPassthroughPlugin()
ts.add_tracking_plugin(plugin)


# Give this object a tracking point 1m in -x
# Have this tracked point detected at the origin, with different rotations
# Result: the object should move around in a circle, while rotating. It will be a red square
# Give this a query point 1m above the object. This will be a blue dot, that should hover just ahead of the red square
m1 = Marker()
m1.header.frame_id = "map"
m1.ns = "demo"
m1.id = 1
m1.type = Marker.CUBE
m1.action = Marker.ADD
m1.scale.x = 0.5
m1.scale.y = 0.05
m1.scale.z = 0.25
m1.color.a = 1
m1.color.r = 1

to1 = TrackedObject("map", m1)
to1_pose = PoseStamped()
to1_pose.pose.position = Point(*(1, 0, 0))
to1_pose.pose.orientation = Quaternion(*(0, 0, 0, 1))


to1.add_tracking_point("Custom", "1", to1_pose)

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
m3.color.b = 1

qpose = PoseStamped()
qpose.pose.position = Point(*(0,-0.5,1))
qpose.pose.orientation = Quaternion(*(0,0,0,1))
to1.add_query_point("test_point", qpose, m3)

ts.add_tracked_object(to1)

# Give this object a tracking point at its origin
# Have this tracked point detected moving around a 2m radius in a circle, with constant orientation
# Result: the object should move around the circle, without rotation; It will be a green square
# Details: It just happens to rotate in the opposite direction as the first object
m2 = Marker()
m2.header.frame_id = "map"
m2.ns = "demo"
m2.id = 2
m2.type = Marker.CUBE
m2.action = Marker.ADD
m2.scale.x = 0.5
m2.scale.y = 0.5
m2.scale.z = 0.5
m2.color.a = 1
m2.color.g = 1

to2 = TrackedObject("map", m2)
to2_pose = PoseStamped()
to2_pose.pose.position = Point(*(0, 0, 0))
to2_pose.pose.orientation = Quaternion(*(0, 0, 0, 1))
to2.add_tracking_point("Custom", "2", to2_pose)
ts.add_tracked_object(to2)



def send_update_msgs():
    # Send custom tracking messages to update the tracking system's understanding
    #  of object positions
    # Use the plugin
    
    while not rospy.is_shutdown():
        dt = time.time() - start_t
        
        o1qw = math.cos(dt/2)
        o1qz = math.sin(dt/2)

        o2r = 2
        o2x = 2*math.sin(dt)
        o2y = 2*math.cos(dt)


        msg = TrackedObjectPoseList()


        obj1 = TrackedObjectPose()
        obj1.id = "1"
        ps1 = PoseStamped()
        ps1.header.frame_id = "map"
        ps1.pose.position = Point(*(0,0,0))
        ps1.pose.orientation = Quaternion(*(0,0, o1qz, o1qw))
        obj1.pose = ps1

        obj2 = TrackedObjectPose()
        obj2.id = "2"
        ps2 = PoseStamped()
        ps2.header.frame_id = "map"
        ps2.pose.position = Point(*(o2x,o2y,0))
        ps2.pose.orientation = Quaternion(*(0,0,0,1))
        obj2.pose = ps2

        msg.object_list = [obj1, obj2]

        #print(msg)

        plugin.detect_poses(msg)
        rospy.sleep(0.1)
    
def send_visualization_msgs():
    # Visualize the latest object positions
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=5)

    while not rospy.is_shutdown():
        
        for m in to1.get_markers().markers:
            pub.publish(m)

        for m in to2.get_markers().markers:
            pub.publish(m)

        rospy.sleep(0.1)
    

if __name__ == "__main__":
    rospy.init_node("demo_node")

    ts.initialize_transform_listener()

    print("starting threads")
    try:
        thread.start_new_thread(send_update_msgs, ())
        thread.start_new_thread(send_visualization_msgs, ())
    except Exception as e:
        print(e)

    while not rospy.is_shutdown():
        rospy.sleep(1)
