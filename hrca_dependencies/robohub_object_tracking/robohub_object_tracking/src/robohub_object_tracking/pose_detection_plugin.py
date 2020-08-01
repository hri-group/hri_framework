

class BasePoseDetectionPlugin:

    def __init__(self):
        self._callback = None

    def get_name(self):
        raise NotImplementedError

    def set_callback(self, callback):
        self._callback = callback

    def forward_tracked_object_pose_list(self, msgs):
        if self._callback is not None:
            self._callback(self.get_name(), msgs)