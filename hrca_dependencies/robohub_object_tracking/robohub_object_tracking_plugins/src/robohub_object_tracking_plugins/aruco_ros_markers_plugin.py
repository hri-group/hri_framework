import rospy

from robohub_object_tracking.pose_detection_plugin import BasePoseDetectionPlugin

from robohub_object_tracking.msg import TrackedObjectPose, TrackedObjectPoseList
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped

class ArucoROSMarkersPlugin(BasePoseDetectionPlugin):

    def __init__(self, markers_topic="/aruco/markers", tracking_system_name="Aruco"):
        self._callback = None
        self.tracking_system_name=tracking_system_name
        self._sub = rospy.Subscriber(markers_topic, MarkerArray, callback=self.detect_poses)

    def get_name(self):
        return self.tracking_system_name

    def detect_poses(self, msg):

        out_msgs = TrackedObjectPoseList()
        out_msgs.object_list = []

        for marker in msg.markers:
            obj = TrackedObjectPose()
            obj.id = str(marker.id)
            obj.pose = PoseStamped()
            obj.pose.pose = marker.pose.pose
            obj.pose.header = marker.header

            out_msgs.object_list.append(obj)

        self.forward_tracked_object_pose_list(out_msgs)
