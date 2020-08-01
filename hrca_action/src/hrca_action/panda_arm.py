import rospy
import actionlib
import copy

from arm import Arm

import franka_gripper.msg
from franka_control.msg import ErrorRecoveryActionGoal

from hrca_msgs.msg import RobotTaskAction, RobotTaskFeedback, RobotTaskResult

from hrca_action.utilities import MoveitObjectHandler

from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)

import smach
import smach_ros
from smach import CBState
from smach_ros import MonitorState

from std_msgs.msg import Empty

#Some functions here taken directly from: #https://github.com/dougsm/mvp_grasp/blob/master/franka_control_wrappers/src/franka_control_wrappers/panda_commander.py
class PandaArm(Arm):

    def __init__(self, 
                 simulation, 
                 home_position = [0, -0.78, 0.0, -2.36, 0, 1.57, 0.78],
                 joint_names = ["panda_joint1", 
                                "panda_joint2", 
                                "panda_joint3", 
                                "panda_joint4", 
                                "panda_joint5",
                                "panda_joint6",
                                "panda_joint7"],
                 arm_planning_group="panda_arm",
                 gripper_planning_group="hand"):

        super(PandaArm, self).__init__(home_position=home_position, 
                                       joint_names=joint_names,
                                       arm_planning_group=arm_planning_group,
                                       gripper_planning_group=gripper_planning_group,
                                       simulation=simulation)

        self.reset_publisher = rospy.Publisher('/franka_control/error_recovery/goal', ErrorRecoveryActionGoal, queue_size=1)

    def home_gripper(self):
        """
        Home and initialise the gripper
        :return: Bool success
        """
        if self.simulation:
            raise ValueError('Cannot use franka_gripper action in simulation, '
                                'set simulation=True if you want to run on the real robot.')
        client = actionlib.SimpleActionClient('franka_gripper/homing', franka_gripper.msg.HomingAction)
        client.wait_for_server()
        client.send_goal(franka_gripper.msg.HomingGoal())
        return client.wait_for_result()

    def set_gripper(self, width, speed=0.1, wait=True):
        """
        Set gripper with.
        :param width: Width in metres of the distance between the two fingers
        :param speed: Move velocity (m/s)
        :param wait: Wait for completion if True
        :return: Bool success
        """

        if self.simulation:
            raise ValueError('Cannot use franka_gripper action in simulation, '
                                'set simulation=True if you want to run on the real robot.')
        client = actionlib.SimpleActionClient('franka_gripper/move', franka_gripper.msg.MoveAction)
        client.wait_for_server()
        client.send_goal(franka_gripper.msg.MoveGoal(width, speed))
        if wait:
            return client.wait_for_result()
        else:
            return True

    def open_gripper(self):
        if self.simulation:
            joint_goal = self.gripper.get_current_joint_values()
            joint_goal[0] = 0.03
            joint_goal[1] = 0.03
            self.gripper.go(joint_goal, wait=True)
            self.gripper.stop()
        else:
            self.set_gripper(width=0.08)

    def close_gripper(self):
        if self.simulation:
            joint_goal = self.gripper.get_current_joint_values()
            joint_goal[0] = 0.0
            joint_goal[1] = 0.0
            self.gripper.go(joint_goal, wait=True)
            self.gripper.stop()
        else:
            self.set_gripper(width=0.0)
        
    def grasp(self, width=0, e_inner=0.1, e_outer=0.1, speed=0.1, force=1):
        """
        Wrapper around the franka_gripper/grasp action.
        http://docs.ros.org/kinetic/api/franka_gripper/html/action/Grasp.html
        :param width: Width (m) to grip
        :param e_inner: epsilon inner
        :param e_outer: epsilon outer
        :param speed: Move velocity (m/s)
        :param force: Force to apply (N)
        :return: Bool success
        """
        if self.simulation:
            raise ValueError('Cannot use franka_gripper action in simulation, '
                                'set simulation=True if you want to run on the real robot.')
        client = actionlib.SimpleActionClient('franka_gripper/grasp', franka_gripper.msg.GraspAction)
        client.wait_for_server()
        client.send_goal(
            franka_gripper.msg.GraspGoal(
                width,
                franka_gripper.msg.GraspEpsilon(e_inner, e_outer),
                speed,
                force
            )
        )
        return client.wait_for_result()

    def recover(self):
        """
        Call the error reset action server.
        """
        if self.simulation:
            raise ValueError('Simulations do not require error recovery, '
                            'set simulation=True if you want to run on the real robot.')
        self.reset_publisher.publish(ErrorRecoveryActionGoal())
        rospy.sleep(3.0)