#!/usr/bin/env python

PKG = 'robohub_object_tracking'
import roslib; roslib.load_manifest(PKG)

import unittest
import std_msgs

from robohub_object_tracking import TrackingPointReference
from geometry_msgs.msg import Pose

class TestAll(unittest.TestCase):

    def setUp(self):
        pass

    def test_get_system(self):
        tpr = TrackingPointReference("Vicon", 1, Pose())
        self.assertEquals(tpr.get_tracking_system(), "Vicon")

        tpr = TrackingPointReference("Aruco", 2, Pose())
        self.assertEquals(tpr.get_tracking_system(), "Aruco")

    def test_get_id(self):
        tpr = TrackingPointReference("Vicon", 1, Pose())
        self.assertEquals(tpr.get_id(), 1)

        tpr = TrackingPointReference("Aruco", 2, Pose())
        self.assertEquals(tpr.get_id(), 2)

    def test_get_offset(self):
        p = Pose()

        p.position.x = 3
        tpr = TrackingPointReference("Vicon", 1, p)
        self.assertEquals(tpr.get_tracking_offset().position.x, 3)

        p.position.x = 4
        p.position.y = 5
        tpr = TrackingPointReference("Vicon", 1, p)
        self.assertEquals(tpr.get_tracking_offset().position.x, 4)
        self.assertEquals(tpr.get_tracking_offset().position.y, 5)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'tracking_point_reference_tests', TestAll)