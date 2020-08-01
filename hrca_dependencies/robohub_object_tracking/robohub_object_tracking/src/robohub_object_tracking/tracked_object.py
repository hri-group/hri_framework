import rospy
import copy

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import geometry_utils

class TrackedObject:

    def __init__(self, desired_frame, marker=None, base_pose_correction_fn=None):
        self._frame = desired_frame
        self._pose = PoseStamped()
        self._pose.header.frame_id = desired_frame
        self._pose.pose.orientation.w = 1
        
        self.marker = marker
        self.base_pose_correction_fn = base_pose_correction_fn

        self._tracking_points = {}
        self._tracking_point_markers = {}

        self._query_points = {}
        self._query_point_markers = {}

    def add_tracking_point(self, system, id, offset, marker=None):
        key = str(id)
        if system not in self._tracking_points.keys():
            self._tracking_points[system] = {key: offset}
            self._tracking_point_markers[system] = {key: marker}
        else:
            self._tracking_points[system][key] = offset
            self._tracking_point_markers[system][key] = marker

    def is_tracked_by(self, system):
        if system not in self._tracking_points.keys():
            return False
        return len(self._tracking_points[system].keys()) > 0

    def has_tracking_point(self, system, id):
        if system not in self._tracking_points.keys():
            return False
        if str(id) not in self._tracking_points[system].keys():
            return False
        
        return True

    def get_tracking_point_offset(self, system, id):
        return self._tracking_points[system][str(id)]

    def add_query_point(self, name, pose, marker=None):
        self._query_points[str(name)] = pose
        self._query_point_markers[str(name)] = marker

    def get_query_point_pose(self, name):
        pose = self._query_points[str(name)]

        out = PoseStamped()
        out.header = self.get_pose().header
        out.pose = geometry_utils.transform_pose(pose, self.get_pose())
        return out

    def update_pose(self, new_pose):
        self._pose = new_pose

    def get_pose(self):
        pose = copy.copy(self._pose)
        if self.base_pose_correction_fn == None:
            return pose
        return self.base_pose_correction_fn(pose)

    def get_frame(self):
        return self._frame

    def get_markers(self):
        marker_array = MarkerArray()

        if self.marker is not None:
            m = self.marker
            m.pose = self.get_pose().pose
            m.header.stamp = rospy.Time.now()
            marker_array.markers.append(m)

        for qp_name in self._query_points.keys():
            m = self._query_point_markers[qp_name]
            if m is not None:
                m.pose = self.get_query_point_pose(qp_name).pose
                m.header.stamp = rospy.Time.now()
            marker_array.markers.append(m)

        for tracking_system in self._tracking_points.keys():
            for tracking_point_name in self._tracking_points[tracking_system].keys():
                    m = self._tracking_point_markers[tracking_system][tracking_point_name]

                    if m is not None:
                        mp = self._tracking_points[tracking_system][tracking_point_name]
                        mp = geometry_utils.transform_pose(mp, self.get_pose())
                        m.pose = mp
                        m.header.stamp = rospy.Time.now()
                        marker_array.markers.append(m)

        return marker_array
