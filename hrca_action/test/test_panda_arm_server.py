#!/usr/bin/python
import sys
import os.path
import rospy
import actionlib
import time

from hrca_action.panda_arm import PandaArm

from hrca_action.utilities import *

from panda_toolbox_task.srv import *

from hrca_msgs.srv import *

from hrca_msgs.msg import RobotTaskAction, RobotTaskFeedback, RobotTaskResult, RobotTaskGoal

MARKER_SIZE = 0.062
PERIOD = 0.1

CM_TO_M = 0.01
INCHES_TO_CM = 2.54

def feedback_cb(msg):
    print msg

def call_server(action, pose1, pose2, object_name):
    client = actionlib.SimpleActionClient("panda_arm_server", RobotTaskAction)

    client.wait_for_server()

    goal = RobotTaskGoal()

    goal.action = action
    goal.pose1 = pose1
    goal.pose2 = pose2
    goal.object_name = object_name

    client.send_goal(goal, feedback_cb=feedback_cb)

    client.wait_for_result()

    result = client.get_result()

    return result

def main():
    rospy.init_node('test_panda_arm', log_level=rospy.WARN)
    
    #Initialize perception service
    rospy.loginfo("Connecting to Perception server")
    perception_srv = rospy.ServiceProxy('tbt_perception_data_request_server', ContextDataRequest)
    perception_srv.wait_for_service()
    rospy.loginfo("Connected!")

    def context_data_request_client(object_id, query_point, tracking_system):
        try:
            context_data_response = perception_srv(int(object_id), str(query_point), str(tracking_system))
            return context_data_response.object_pose
        except rospy.ServiceException, e:
            print ("Service call failed: %s"%e)
    
    marker_pose = context_data_request_client(1, "marker_point", "Aruco")

    #Initialize get_robot_pose service
    rospy.loginfo("Connecting to /panda_arm_server")
    try:
        get_robot_pose_srv = rospy.ServiceProxy("/panda_arm_server/get_robot_pose", GetRobotPose)
        rospy.wait_for_service("/panda_arm_server/get_robot_pose")
    except rospy.ServiceException as e:
        rospy.logerr("panda_arm_server 'get_robot_pose' service initialization failed: %s" % e)
        shutdown_msg = "Shutting down %s node because %s service connection failed." % (rospy.get_name(),get_robot_pose_srv.resolved_name)
        rospy.logerr(shutdown_msg)
        sys.exit(0)
    rospy.loginfo("Connected!")

    current_gripper_orientation = get_robot_pose_srv("world").pose_stamped.pose.orientation

    print(current_gripper_orientation)

    pick_pose = offset_and_rotate_goal(context_data_request_client(1, "grasp_point_1", "Aruco"), current_gripper_orientation, -1*CM_TO_M, 0, 0*CM_TO_M, 0, 0, 0)

    pick_pose = create_pose_stamped(pick_pose, "panda_link0")
    print(pick_pose)

    try:
        result = call_server("plan_to_pose_safe", pick_pose, pick_pose, "part_1")
        print result
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    raw_input("Press Enter to grasp...")
    try:
        result = call_server("grasp", pick_pose, pick_pose, "")
        print result
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

    try:
        result = call_server("open_gripper", pick_pose, pick_pose, "")
        print result
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e
    
    print("Done testing!")

if __name__ == "__main__":
    main()




