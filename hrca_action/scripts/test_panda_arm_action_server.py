#! /usr/bin/env python
import copy
import rospy
import actionlib
from hrca_msgs.msg import RobotTaskAction, RobotTaskGoal

import math
from math import radians, pi

from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point

from hrca_action.panda_arm import PandaArm 

from hrca_msgs.msg import RobotTaskAction, RobotTaskFeedback, RobotTaskResult, RobotTaskGoal
#from hrca_common_utils import hrca_utils
from hrca_action.utilities import MoveitObjectHandler

from panda_toolbox_task.srv import *

from tf import transformations
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, unit_vector, quaternion_multiply

def feedback_cb(msg):
    print msg

def call_server(action, pose1, pose2):
    client = actionlib.SimpleActionClient("Arm_Task_Server", RobotTaskAction)

    client.wait_for_server()

    goal = RobotTaskGoal()

    goal.action = action
    goal.pose1 = pose1
    goal.pose2 = pose2

    client.send_goal(goal, feedback_cb=feedback_cb)

    client.wait_for_result()

    result = client.get_result()

    return result

if __name__ == '__main__':
    
    rospy.init_node('test_panda_arm_server')
    
    #access instance of PandaArm for some of the member functions -> separate from the panda_arm_action_server
    panda_arm = PandaArm(simulation=True)
    
    panda_arm.set_end_effector_link("panda_gripper_center")
    panda_arm.moveit_arm_group.set_pose_reference_frame("/panda_link0")
    print(panda_arm.moveit_arm_group.get_pose_reference_frame())
    panda_arm.open_gripper()
    panda_arm.move_to_home()
    panda_arm.start_robot_task_action_server()

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
    
    pick_pose = context_data_request_client(1, "grasp_point_1", "Aruco")

    pose_goal = Pose()
    pose_goal = copy.deepcopy(pick_pose)
    pose_goal.orientation = panda_arm.get_current_pose().pose.orientation
    q_orientation = [pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]

    #Orient the gripper (THIS ASSUMES part is standing somewhat perfectly vertical)
    q_rotated = quaternion_from_euler(math.radians(0), math.radians(0), math.radians(0))
    q = quaternion_multiply(q_orientation, q_rotated)
    roll, pitch, yaw = euler_from_quaternion(q)

    q = quaternion_from_euler(roll, pitch, yaw)

    x = pose_goal.position.x + 0.01 #thickness of part is 0.01m
    y = pose_goal.position.y
    z = pose_goal.position.z + 0.05 #+ FRAME_EE_LINK8_OFFSET_Z
    pose_goal = Pose(Point(x, y, z), Quaternion(*q))

    print(pick_pose)
    print(pose_goal)

    moh = MoveitObjectHandler()
    moh.remove_all_objects()
    bottom_part_size = (0.3, 0.2, 0.01)
    rotation = (0, 0, 0)

    print(panda_arm.robot.get_planning_frame())

    #panda_arm.move_to_pose(pose_goal)
    #moh.add_box_object("bottom_part", pick_pose, size=bottom_part_size, rotation=rotation, frame="panda_link0")
    
    try:
        result = call_server("pick_and_place", pose_goal, pose_goal)
        print result
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e

