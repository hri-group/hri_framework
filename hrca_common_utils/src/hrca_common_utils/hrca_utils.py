import os
import rospy
import actionlib

class HRCAUtils(object):

    def make_service(self, service_name, srv_type, callback):
        return rospy.Service(service_name, srv_type, callback)

    def make_action_server(self, action_name, action_type, execute_cb, auto_start=False):
        return actionlib.SimpleActionServer(action_name, action_type, execute_cb, auto_start)
    
    def make_action_client(self, name, action_type)
        return actionlib.SimpleActionClient(name, action_type)

    def make_publisher(self, publisher_name, msg_type):
        return rospy.Publisher(publisher_name, msg_type)

    def make_service_proxy(self, service_name, srv_type):
        rospy.logerr("Connecting to service with name: %s"%service_name)
        rospy.wait_for_service(service_name)
        rospy.logerr("Connected to service successfully.")
        return rospy.ServiceProxy(service_name,srv_type)

    def __init__(self):
        pass
