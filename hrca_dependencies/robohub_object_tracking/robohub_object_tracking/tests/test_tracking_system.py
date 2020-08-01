#!/usr/bin/env python

PKG = 'robohub_object_tracking'
import roslib; roslib.load_manifest(PKG)

import unittest
import std_msgs

from robohub_object_tracking import TrackingSystem
from robohub_object_tracking import TrackedObject

from robohub_object_tracking.msg import TrackedObjectPose, TrackedObjectPoseList

from geometry_msgs.msg import PoseStamped, Point, Quaternion

def fake_transform_pose_to_frame(pose, frame):
    return pose

class TestAll(unittest.TestCase):

    def setUp(self):
        pass
    
    def create_tracking_system(self):
        ts = TrackingSystem()
        return ts

    def test_initializes(self):
        ts = self.create_tracking_system()

    def test_add_objects(self):
        ts = self.create_tracking_system()
        self.assertEquals(len(ts._tracked_objects), 0)

        ts.add_tracked_object("mock_object")
        self.assertEquals(len(ts._tracked_objects), 1)

        ts.add_tracked_object("mock_object")
        self.assertEquals(len(ts._tracked_objects), 2)

class TestObjectUpdates(unittest.TestCase):
    
    def setUp(self):
        pass

    def assert_pose_stamped_almost_equal(self, pose, vec, quat):
        # Vec (x,y,z), Quat(x,y,z,q) are tuples
        self.assertAlmostEquals(pose.pose.position.x, vec[0])
        self.assertAlmostEquals(pose.pose.position.y, vec[1])
        self.assertAlmostEquals(pose.pose.position.z, vec[2])

        self.assertAlmostEquals(pose.pose.orientation.x, quat[0])
        self.assertAlmostEquals(pose.pose.orientation.y, quat[1])
        self.assertAlmostEquals(pose.pose.orientation.z, quat[2])
        self.assertAlmostEquals(pose.pose.orientation.w, quat[3])

    def test_accepts_empty_custom_msg(self):
        ts = TrackingSystem()
        msg = TrackedObjectPoseList()
        ts.plugin_general_tracking_callback("Custom", msg)
    
    def test_update_with_no_matches(self):
        ts = TrackingSystem()

        to = TrackedObject("map")
        to_pose = PoseStamped()
        to_pose.pose.orientation = Quaternion(*(0, 0, 0, 1))
        to.add_tracking_point("Custom", "1", to_pose)
        ts.add_tracked_object(to)

        msg = TrackedObjectPoseList()
        obj = TrackedObjectPose()
        obj.id = "2"
        obj.pose = PoseStamped()
        msg.object_list = [obj]

        ts.plugin_general_tracking_callback("Custom", msg)

        self.assert_pose_stamped_almost_equal(to.get_pose(), (0,0,0), (0,0,0,1))
        

    def test_update_with_match(self):
        ts = TrackingSystem()
        ts.transform_pose_to_frame = fake_transform_pose_to_frame

        to = TrackedObject("map")
        to_pose = PoseStamped()
        to_pose.pose.orientation = Quaternion(*(0, 0, 0, 1))

        to.add_tracking_point("Custom", "1", to_pose)
        ts.add_tracked_object(to)

        msg = TrackedObjectPoseList()
        obj = TrackedObjectPose()
        obj.id = "1"
        ps = PoseStamped()
        ps.pose.position = Point(*(3,4,5))
        ps.pose.orientation = Quaternion(*(0,0,0,1))
        obj.pose = ps
        msg.object_list = [obj]

        ts.plugin_general_tracking_callback("Custom", msg)

        self.assert_pose_stamped_almost_equal(to.get_pose(), (3,4,5), (0,0,0,1))

    def test_update_with_match_and_tracking_offset(self):
        ts = TrackingSystem()
        ts.transform_pose_to_frame = fake_transform_pose_to_frame

        to = TrackedObject("map")
        to_pose = PoseStamped()
        to_pose.pose.position = Point(*(0, 0, 2))
        to_pose.pose.orientation = Quaternion(*(0, 0, 0, 1))

        to.add_tracking_point("Custom", "1", to_pose)
        ts.add_tracked_object(to)

        msg = TrackedObjectPoseList()
        obj = TrackedObjectPose()
        obj.id = "1"
        ps = PoseStamped()
        ps.pose.position = Point(*(3,4,5))
        ps.pose.orientation = Quaternion(*(0,0,0,1))
        obj.pose = ps
        msg.object_list = [obj]

        ts.plugin_general_tracking_callback("Custom", msg)

        self.assert_pose_stamped_almost_equal(to.get_pose(), (3,4,5-2), (0,0,0,1))

    def test_multiple_updates_and_skips(self):
        ts = TrackingSystem()
        ts.transform_pose_to_frame = fake_transform_pose_to_frame
        # Track objects 1, 2, and 3
        # Get messages for objects 1, 2 and 4
        # Give object 2 a tracking point offset

        to1 = TrackedObject("map")
        to1_pose = PoseStamped()
        to1_pose.pose.orientation = Quaternion(*(0, 0, 0, 1))
        to1.add_tracking_point("Custom", "1", to1_pose)
        ts.add_tracked_object(to1)

        to2 = TrackedObject("map")
        to2_pose = PoseStamped()
        to2_pose.pose.position = Point(*(0, 0, 1))
        to2_pose.pose.orientation = Quaternion(*(0, 0, 0, 1))
        to2.add_tracking_point("Custom", "2", to2_pose)
        ts.add_tracked_object(to2)

        to3 = TrackedObject("map")
        to3_pose = PoseStamped()
        to3_pose.pose.orientation = Quaternion(*(0, 0, 0, 1))
        to3.add_tracking_point("Custom", "3", to3_pose)
        ts.add_tracked_object(to3)


        msg = TrackedObjectPoseList()

        obj1 = TrackedObjectPose()
        obj1.id = "1"
        ps1 = PoseStamped()
        ps1.pose.position = Point(*(3,4,5))
        ps1.pose.orientation = Quaternion(*(0,0,0,1))
        obj1.pose = ps1

        obj2 = TrackedObjectPose()
        obj2.id = "2"
        ps2 = PoseStamped()
        ps2.pose.position = Point(*(6,-1,10))
        ps2.pose.orientation = Quaternion(*(0,0,0,1))
        obj2.pose = ps2

        obj4 = TrackedObjectPose()
        obj4.id = "4"
        ps4 = PoseStamped()
        ps4.pose.position = Point(*(3,4,5))
        ps4.pose.orientation = Quaternion(*(0,0,0,1))
        obj4.pose = ps4

        msg.object_list = [obj1, obj2, obj4]

        ts.plugin_general_tracking_callback("Custom", msg)

        self.assert_pose_stamped_almost_equal(to1.get_pose(), (3,4,5), (0,0,0,1))
        self.assert_pose_stamped_almost_equal(to2.get_pose(), (6,-1,10-1), (0,0,0,1))


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'tracking_system_tests', TestAll)
    rostest.rosrun(PKG, 'tracking_system_tests', TestObjectUpdates)
    rostest.rosrun(PKG, 'tracking_system_tests', TestTrackingPlugins)