from robohub_object_tracking.pose_detection_plugin import BasePoseDetectionPlugin

class CustomMsgPassthroughPlugin(BasePoseDetectionPlugin):

    def __init__(self):
        self._callback = None

    def get_name(self):
        return "Custom"

    def detect_poses(self, msgs):
        self.forward_tracked_object_pose_list(msgs)
