#!/usr/bin/python
from hrca_action.panda_arm import PandaArm
from hrca_action.utilities import *


home_pose  = [0, -0.78, 0.0, -2.36, 0, 1.57, 0.78]
ready_pose  = [0.06, -0.34, 0.3, -2.35, 0.1, 2.0, 1.2]
far_push   = [0.1, 0.54, 0.2, -2.2, -0.25, 2.75, 1.25]
close_push = [0.14, 0.3, 0.25, -2.75, -0.25, 3.06, 1.3]


def main():
    rospy.init_node('test_panda_arm_traj_replace', log_level=rospy.WARN)
    panda_arm = PandaArm(simulation=True)
    panda_arm.set_end_effector_link("panda_gripper_center")
    panda_arm.open_gripper()

    panda_arm.move_to_joint_positions(ready_pose, wait=False)
    rospy.sleep(0.5)
    panda_arm.move_to_joint_positions(close_push, wait=False)

if __name__ == "__main__":
    main()