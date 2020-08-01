
class TrackingPointReference:

    def __init__(self, tracking_system_name, id, offset):
        self._system_name = tracking_system_name
        self._id = id
        self._offset = offset
    
    def get_tracking_system(self):
        return self._system_name
    
    def get_id(self):
        return self._id
    
    def get_tracking_offset(self):
        return self._offset