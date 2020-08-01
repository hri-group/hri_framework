#!/usr/bin/env python

PKG = 'robohub_object_tracking'
import roslib; roslib.load_manifest(PKG)

import math
import sys
import random
import unittest
import tf

from geometry_msgs.msg import Point, Quaternion, PoseStamped

from  robohub_object_tracking import geometry_utils

ZERO_POS = (0,0,0)
IDEN_QUAT = (0,0,0,1)   

class TestTransformPose(unittest.TestCase):

    def setUp(self):
        pass

    def create_pose(self):
        p = PoseStamped()
        p.pose.position = Point(*ZERO_POS)
        p.pose.orientation = Quaternion(*IDEN_QUAT)
        return p

    def assert_pose_almost_equal(self, pose, vec, quat):
        # Vec (x,y,z), Quat(x,y,z,q) are tuples
        self.assertAlmostEquals(pose.position.x, vec[0])
        self.assertAlmostEquals(pose.position.y, vec[1])
        self.assertAlmostEquals(pose.position.z, vec[2])

        self.assertAlmostEquals(pose.orientation.x, quat[0])
        self.assertAlmostEquals(pose.orientation.y, quat[1])
        self.assertAlmostEquals(pose.orientation.z, quat[2])
        self.assertAlmostEquals(pose.orientation.w, quat[3])

    def assert_pose_stamped_almost_equal(self, pose, vec, quat):
        # Vec (x,y,z), Quat(x,y,z,q) are tuples
        self.assertAlmostEquals(pose.pose.position.x, vec[0])
        self.assertAlmostEquals(pose.pose.position.y, vec[1])
        self.assertAlmostEquals(pose.pose.position.z, vec[2])

        self.assertAlmostEquals(pose.pose.orientation.x, quat[0])
        self.assertAlmostEquals(pose.pose.orientation.y, quat[1])
        self.assertAlmostEquals(pose.pose.orientation.z, quat[2])
        self.assertAlmostEquals(pose.pose.orientation.w, quat[3])

    def test_identity_transformation_base(self):
        base = PoseStamped()
        base.pose.position = Point(*ZERO_POS)
        base.pose.orientation = Quaternion(*IDEN_QUAT)

        offset = self.create_pose()

        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, ZERO_POS, IDEN_QUAT)

    def test_identity_transformation_offset(self):
        base = self.create_pose()
        offset = self.create_pose()

        offset.pose.position.x = 1
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (1,0,0), IDEN_QUAT)

        offset.pose.position.y = 2
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (1,2,0), IDEN_QUAT)

        offset.pose.position.z = 3
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (1,2,3), IDEN_QUAT)

    def test_identity_transformation_rotated(self):
        base = self.create_pose()
        offset = self.create_pose()

        offset.pose.orientation.x = 1
        offset.pose.orientation.w = 0
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, ZERO_POS, (1,0,0,0))

        r = 1.0/math.sqrt(2)
        offset.pose.orientation.x = r
        offset.pose.orientation.y = r
        offset.pose.orientation.w = 0
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, ZERO_POS, (r,r,0,0))

        r = 1.0/math.sqrt(3)
        offset.pose.orientation.x = r
        offset.pose.orientation.y = r
        offset.pose.orientation.z = r
        offset.pose.orientation.w = 0
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, ZERO_POS, (r,r,r,0))

        r = 1.0/math.sqrt(4)
        offset.pose.orientation.x = r
        offset.pose.orientation.y = r
        offset.pose.orientation.z = r
        offset.pose.orientation.w = r
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, ZERO_POS, (r,r,r,r))
    
    def test_translation_of_base(self):
        base = self.create_pose()
        offset = self.create_pose()

        base.pose.position.x = 1
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (1,0,0), IDEN_QUAT)

        base.pose.position.y = 2
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (1,2,0), IDEN_QUAT)

        base.pose.position.z = 3
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (1,2,3), IDEN_QUAT)

    def test_translation_of_both(self):
        base = self.create_pose()
        offset = self.create_pose()

        base.pose.position = Point(*(1, 2, 3))
        offset.pose.position = Point(*(4, 5, 6))
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (5,7,9), IDEN_QUAT)

    def test_orientation_of_base_roll(self):
        base = self.create_pose()
        offset = self.create_pose()
        offset.pose.position = Point(*(1, 0, 0))

        quat = tf.transformations.quaternion_from_euler(math.pi*0.25, 0, 0)
        base.pose.orientation = Quaternion(*quat)
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (1,0,0), quat)

        quat = tf.transformations.quaternion_from_euler(-math.pi*0.25, 0, 0)
        base.pose.orientation = Quaternion(*quat)
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (1,0,0), quat)

    def test_orientation_of_base_pitch(self):
        base = self.create_pose()
        offset = self.create_pose()
        offset.pose.position = Point(*(1, 0, 0))

        quat = tf.transformations.quaternion_from_euler(0, math.pi*0.5, 0)
        base.pose.orientation = Quaternion(*quat)
        res = geometry_utils.transform_pose(offset, base)
        print(res)
        print(quat)
        self.assert_pose_almost_equal(res, (0,0,-1), quat)

        quat = tf.transformations.quaternion_from_euler(0, -math.pi*0.5, 0)
        base.pose.orientation = Quaternion(*quat)
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (0,0,1), quat)

    def test_orientation_of_base_yaw(self):
        base = self.create_pose()
        offset = self.create_pose()
        offset.pose.position = Point(*(1, 0, 0))

        quat = tf.transformations.quaternion_from_euler(0, 0, math.pi*0.5)
        base.pose.orientation = Quaternion(*quat)
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (0,1,0), quat)

        quat = tf.transformations.quaternion_from_euler(0, 0, -math.pi*0.5)
        base.pose.orientation = Quaternion(*quat)
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (0,-1,0), quat)

    def test_full_transformation(self):
        base = PoseStamped()
        base.pose.position = Point(*(1, 2, 3))
        base.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(math.pi*0.5, 0, 0))
    
        offset = PoseStamped()
        offset.pose.position = Point(*(4, 5, 6))
        offset.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(math.pi*0.5, 0, 0))

        res = geometry_utils.transform_pose(offset, base)
        quat = tf.transformations.quaternion_from_euler(math.pi, 0, 0)
        self.assert_pose_almost_equal(res, (1+4,-6+2,5+3), quat)


class TestInvertPose(unittest.TestCase):

    def setUp(self):
        pass

    def assert_pose_almost_equal(self, pose, vec, quat):
        # Vec (x,y,z), Quat(x,y,z,q) are tuples
        self.assertAlmostEquals(pose.pose.position.x, vec[0])
        self.assertAlmostEquals(pose.pose.position.y, vec[1])
        self.assertAlmostEquals(pose.pose.position.z, vec[2])

        self.assertAlmostEquals(pose.pose.orientation.x, quat[0])
        self.assertAlmostEquals(pose.pose.orientation.y, quat[1])
        self.assertAlmostEquals(pose.pose.orientation.z, quat[2])
        self.assertAlmostEquals(pose.pose.orientation.w, quat[3])

    def test_invert_identity(self):
        p1 = PoseStamped()
        v = (0, 0, 0)
        q = (0, 0, 0, 1)
        p1.pose.position = Point(*v)
        p1.pose.orientation = Quaternion(*q)

        inv = geometry_utils.calculate_inverse_pose(p1)
        self.assert_pose_almost_equal(inv, (0, 0, 0), (0, 0, 0, 1))

    def test_invert_translations(self):
        q = [0, 0, 0, 1]

        p = PoseStamped()
        p.pose.position = Point(*(1, 0, 0))
        p.pose.orientation = Quaternion(*q)
        inv = geometry_utils.calculate_inverse_pose(p)
        result = geometry_utils.transform_pose(p, inv)
        quick_fix = PoseStamped()
        quick_fix.pose = result
        self.assert_pose_almost_equal(quick_fix, (0, 0, 0), (0, 0, 0, 1))

        p = PoseStamped()
        p.pose.position = Point(*(0, 2, 0))
        p.pose.orientation = Quaternion(*q)
        inv = geometry_utils.calculate_inverse_pose(p)
        result = geometry_utils.transform_pose(p, inv)
        quick_fix = PoseStamped()
        quick_fix.pose = result
        self.assert_pose_almost_equal(quick_fix, (0, 0, 0), (0, 0, 0, 1))

        p = PoseStamped()
        p.pose.position = Point(*(0, 0, 3))
        p.pose.orientation = Quaternion(*q)
        inv = geometry_utils.calculate_inverse_pose(p)
        result = geometry_utils.transform_pose(p, inv)
        quick_fix = PoseStamped()
        quick_fix.pose = result
        self.assert_pose_almost_equal(quick_fix, (0, 0, 0), (0, 0, 0, 1))

    def test_invert_orientations(self):
        v = [0, 0, 0]

        # Value of qN for N equal non-zero components
        q2 = math.sqrt(1/2)
        q3 = math.sqrt(1/3)
        q4 = math.sqrt(1/4)

        p = PoseStamped()
        p.pose.position = Point(*v)
        p.pose.orientation = Quaternion(*(q2, 0, 0, q2))
        inv = geometry_utils.calculate_inverse_pose(p)
        result = geometry_utils.transform_pose(p, inv)
        quick_fix = PoseStamped()
        quick_fix.pose = result
        self.assert_pose_almost_equal(quick_fix, (0, 0, 0), (0, 0, 0, 1))

        p = PoseStamped()
        p.pose.position = Point(*v)
        p.pose.orientation = Quaternion(*(0, q2, 0, q2))
        inv = geometry_utils.calculate_inverse_pose(p)
        result = geometry_utils.transform_pose(p, inv)
        quick_fix = PoseStamped()
        quick_fix.pose = result
        self.assert_pose_almost_equal(quick_fix, (0, 0, 0), (0, 0, 0, 1))

        p = PoseStamped()
        p.pose.position = Point(*v)
        p.pose.orientation = Quaternion(*(0, 0, q2, q2))
        inv = geometry_utils.calculate_inverse_pose(p)
        result = geometry_utils.transform_pose(p, inv)
        quick_fix = PoseStamped()
        quick_fix.pose = result
        self.assert_pose_almost_equal(quick_fix, (0, 0, 0), (0, 0, 0, 1))

        p = PoseStamped()
        p.pose.position = Point(*v)
        p.pose.orientation = Quaternion(*(q3, q3, q3, 0))
        inv = geometry_utils.calculate_inverse_pose(p)
        result = geometry_utils.transform_pose(p, inv)
        quick_fix = PoseStamped()
        quick_fix.pose = result
        self.assert_pose_almost_equal(quick_fix, (0, 0, 0), (0, 0, 0, 1))

        p = PoseStamped()
        p.pose.position = Point(*v)
        p.pose.orientation = Quaternion(*(q4, q4, q4, q4))
        inv = geometry_utils.calculate_inverse_pose(p)
        result = geometry_utils.transform_pose(p, inv)
        quick_fix = PoseStamped()
        quick_fix.pose = result
        self.assert_pose_almost_equal(quick_fix, (0, 0, 0), (0, 0, 0, 1))

    def test_invert_a_random_pose(self):
        q = [ 0.1262852, 0.1261165, 0.2100786, 0.9612563 ]
        v = [random.random()*10 for i in range(0, 3)]
        p = PoseStamped()
        p.pose.position = Point(*v)
        p.pose.orientation = Quaternion(*q)

        inv = geometry_utils.calculate_inverse_pose(p)
        result = geometry_utils.transform_pose(p, inv)

        quick_fix = PoseStamped()
        quick_fix.pose = result
        self.assert_pose_almost_equal(quick_fix, (0, 0, 0), (0, 0, 0, 1))


    def test_invert_many_random_poses(self):
        q_base = [ 0.1262852, 0.1261165, 0.2100786, 0.9612563 ] # From random converter online

        for i in range(0, 5):
            random.shuffle(q_base)
            q = tuple(q_base)
            v = [(random.random()-0.5)*10 for i in range(0, 3)]

            p = PoseStamped()
            p.pose.position = Point(*v)
            p.pose.orientation = Quaternion(*q)

            inv = geometry_utils.calculate_inverse_pose(p)
            result = geometry_utils.transform_pose(p, inv)

            quick_fix = PoseStamped()
            quick_fix.pose = result

            self.assert_pose_almost_equal(quick_fix, (0, 0, 0), (0, 0, 0, 1))
        

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'geometry_utils_tests', TestTransformPose)
    rostest.rosrun(PKG, 'geometry_utils_tests', TestInvertPose)