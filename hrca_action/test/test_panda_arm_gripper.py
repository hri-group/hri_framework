#!/usr/bin/python
import sys
import os.path
import rospy

import time

from hrca_action.panda_arm import PandaArm

from hrca_action.utilities import MoveitObjectHandler

def main():
    rospy.init_node('test_panda_arm_gripper', log_level=rospy.WARN)


    panda_arm = PandaArm(simulation=False)

    #panda_arm.home_gripper()
    
    #time.sleep(1)

    #panda_arm.close_gripper()
    
    #time.sleep(1)
    
    panda_arm.open_gripper()

    #panda_arm.set_end_effector_link("panda_gripper_center")

    #panda_arm.move_to_home()

    #time.sleep(1)

    #panda_arm.move_to_joint_positions(poses["B"])

    #time.sleep(1)

    #panda_arm.move_to_joint_positions(poses["C"])

    #time.sleep(1)

    #panda_arm.move_to_home()

    print("Done testing!")


if __name__ == "__main__":
    main()




