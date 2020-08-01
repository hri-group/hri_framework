import rospy

from tf import TransformListener
import geometry_utils
from geometry_msgs.msg import PoseStamped
from robohub_object_tracking.msg import TrackedObjectPoseList

from visualization_msgs.msg import MarkerArray

class TrackingSystem:

    def __init__(self):
        self._tf_listener = None
        self._tracked_objects = []
        self._tracking_plugins = {}

    def initialize_transform_listener(self):
        self._tf_listener = TransformListener()

    def add_tracked_object(self, tracked_object):
        self._tracked_objects.append(tracked_object)

    def add_tracking_plugin(self, plugin):
        plugin.set_callback(self.plugin_general_tracking_callback)
        self._tracking_plugins[plugin.get_name()] = plugin

    def remove_tracking_plugin(self, name):
        self._tracking_plugins[name] = None

    def plugin_general_tracking_callback(self, name, msg):
        objects = [(tracked.id, tracked.pose) for tracked in msg.object_list]
        self.perform_updates_for_tracked_objects_list(name, objects)

    def perform_updates_for_tracked_objects_list(self, system, objects):
        # Expect list of tuples (id, pose)
        for obj in objects:
            self.update_objects(system, obj[0], obj[1])

    def update_objects(self, system, tracked_id, new_pose):
        for obj in self._tracked_objects:
            if obj.has_tracking_point(system, tracked_id):
                self.update_object_pose(obj, system, tracked_id, new_pose)

    def update_object_pose(self, obj, system, tracked_id, new_pose):
        tracking_offset = obj.get_tracking_point_offset(system, tracked_id)
        obj_base_pose = self.calculate_base_pose(new_pose, tracking_offset)
        obj_pose = self.transform_pose_to_frame(obj_base_pose, obj.get_frame())
        obj.update_pose(obj_pose)


    def calculate_base_pose(self, tracked_pose, offset):
        m_new = geometry_utils.transform_pose(geometry_utils.calculate_inverse_pose(offset), tracked_pose)
        new_pose = PoseStamped()
        new_pose.header.frame_id = tracked_pose.header.frame_id
        new_pose.pose = m_new

        return new_pose

    def transform_pose_to_frame(self, pose, frame):
        return self._tf_listener.transformPose(frame, pose)

    def get_markers(self):
        marker_array = MarkerArray()
        marker_array.markers = []

        for obj in self._tracked_objects:
            markers = obj.get_markers().markers
            for marker in markers:
                marker_array.markers.append(marker)
        
        return marker_array
