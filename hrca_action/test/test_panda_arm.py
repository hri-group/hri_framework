#!/usr/bin/python
import sys
import os.path
import rospy

import time

from hrca_action.panda_arm import PandaArm

from hrca_action.utilities import *

from panda_toolbox_task.srv import *

MARKER_SIZE = 0.062
PERIOD = 0.1

CM_TO_M = 0.01
INCHES_TO_CM = 2.54

def main():
    rospy.init_node('test_panda_arm', log_level=rospy.WARN)

    panda_arm = PandaArm(simulation=False)
    panda_arm.set_end_effector_link("panda_gripper_center")
    panda_arm.recover()
    panda_arm.open_gripper()

    home = raw_input("Move to home? y/n: ") == "y"
    if home:
        panda_arm.move_to_home()

    
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

    current_gripper_orientation = panda_arm.get_current_pose().pose.orientation
    pick_pose = offset_and_rotate_goal(context_data_request_client(1, "grasp_point_1", "Aruco"), current_gripper_orientation, -1*CM_TO_M, 0, 0*CM_TO_M, 0, 0, 0)

    pick_pose = create_pose_stamped(pick_pose, "panda_link0")

    moh = MoveitObjectHandler()
    moh.remove_all_objects()
    bottom_part_size = (0.3, 0.2, 0.01)
    rotation = (0, 0, 0)
    moh.add_box_object("bottom_part", marker_pose, size=bottom_part_size, rotation=rotation, frame="panda_link0")

    print(pick_pose)
    plan = panda_arm.plan_to_pose_optimized(pick_pose, 5)
    panda_arm.execute_plan_safe(plan)

    grasp = raw_input("Grasp? y/n: ") == "y"
    if grasp:
        panda_arm.grasp(width = 0.01, force=30)

    panda_arm.open_gripper()

    print("Done testing!")

if __name__ == "__main__":
    main()




