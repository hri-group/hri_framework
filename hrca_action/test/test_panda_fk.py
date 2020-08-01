#!/usr/bin/python
import sys
import os.path
import rospy

import time

from hrca_action.panda_arm import PandaArm

from hrca_action.utilities import MoveitObjectHandler

def main():
    rospy.init_node('test_panda_fk', log_level=rospy.WARN)

    panda_arm = PandaArm(simulation=True)

    panda_arm.set_end_effector_link("panda_gripper_center")

    current_pose = panda_arm.get_current_fk_pose()

    print(current_pose)

    panda_arm.move_to_pose(current_pose)

    print(panda_arm.get_ik(current_pose,"panda_gripper_center"))

    print("Done testing!")


if __name__ == "__main__":
    main()




