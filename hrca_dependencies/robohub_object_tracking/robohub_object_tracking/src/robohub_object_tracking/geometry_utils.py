import tf
import sys
import numpy as np

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

# Euler should be RPY (https://github.com/ros/geometry/blob/hydro-devel/tf/src/tf/transformations.py)

def transform_pose(offset, base):
    # Transforms offset to be a pose with respect to base's frame
    # Keep Base's frame
    # In matrix terms: result = [base][offset]
    base_mat = create_matrix_from_pose_msg(base.pose)
    offset_mat = create_matrix_from_pose_msg(offset.pose)
    final_mat = np.dot(base_mat, offset_mat)

    return create_pose_msg_from_matrix(final_mat)


def create_pose_msg_from_matrix(matrix):
    q = tf.transformations.quaternion_from_matrix(matrix)
    t = tf.transformations.translation_from_matrix(matrix)

    p = Pose()
    p.position = Point(*t)
    p.orientation = Quaternion(*q)
    return p

def create_matrix_from_pose_msg(pose):
    pos = pose.position
    orien = pose.orientation
    t = (pos.x, pos.y, pos.z)
    o = (orien.x, orien.y, orien.z, orien.w)

    trans = tf.transformations.translation_matrix(t)
    rot = tf.transformations.quaternion_matrix(o)

    mat = np.dot(trans, rot)
    return mat

def calculate_inverse_pose(in_pose):
    mat = create_matrix_from_pose_msg(in_pose.pose)
    mat = np.linalg.inv(mat)
    p = create_pose_msg_from_matrix(mat)
    ps = PoseStamped()
    ps.pose = p
    return ps