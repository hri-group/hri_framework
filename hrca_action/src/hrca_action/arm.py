import rospy
import sys
import numpy as np
from abc import ABCMeta, abstractmethod
from sensor_msgs.msg import JointState
import moveit_commander
from moveit_commander.conversions import list_to_pose
import moveit_msgs.msg

import actionlib

from std_srvs.srv import Empty, EmptyRequest

from hrca_msgs.msg import RobotTaskAction, RobotTaskFeedback, RobotTaskResult
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from moveit_msgs.msg import PositionIKRequest

from moveit_msgs.msg import MoveItErrorCodes

#from hrca_common_utils import HRCAUtils

# parts of code adapted from: https://github.com/frankaemika/icra18-fci-tutorial/blob/master/icra18/scripts/demo.py and 
# https://github.com/pal-robotics/tiago_tutorials/blob/kinetic-devel/tiago_pick_demo/scripts/pick_and_place_server.py
# https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py
# https://github.com/dougsm/mvp_grasp/blob/master/franka_control_wrappers/src/franka_control_wrappers/panda_commander.py
# https://github.com/uts-magic-lab/moveit_python_tools/blob/master/src/moveit_python_tools/get_fk.py

class Arm(object):
    """
    Abstract Base Class with the core arm functionality that other arms 
    should build on top of via inheritance. This includes the gripper if the 
    arm has one.
    """

    __metaclass__ = ABCMeta
    
    def __init__(self,
            simulation, 
            ns="/hrca",
            use_moveit=True,
            moveit_planner='ESTkConfigDefault', 
            arm_planning_group=None,
            planning_time=10, 
            max_accel=1.0, 
            max_vel=1.0, 
            goal_tolerance=0.01,
            dof=7,
            joint_names=None,
            home_position=None,
            has_gripper=True,
            gripper_planning_group=None,
            has_wrist_camera=False
            ):

        self.ns = ns
        self.use_moveit = use_moveit
        self.moveit_planner = moveit_planner
        self.arm_planning_group = arm_planning_group
        self.planning_time = planning_time
        self.max_accel = max_accel
        self.max_vel = max_vel
        self.goal_tolerance = goal_tolerance
        self.dof = dof
        self.joint_names =joint_names
        self.home_position = home_position
        self.has_gripper = has_gripper
        self.gripper_planning_group = gripper_planning_group
        self.has_wrist_camera = has_wrist_camera
        self.simulation=simulation

        #subscribers
        self.js_sub = rospy.Subscriber('/joint_states',
                                       JointState,
                                       self.js_cb,
                                       queue_size=1)

        self.last_joint_states = None

        if self.use_moveit is True:
            self.initialize_moveit()
            #create a dictionary of moveit errors
            moveit_error_dict = {}
            for name in MoveItErrorCodes.__dict__.keys():
               if not name[:1] == '_':
                   code = MoveItErrorCodes.__dict__[name]
                   moveit_error_dict[code] = name

        if has_wrist_camera is True:
            #any camera setup
            pass

    def get_current_pose(self):
        """
        """
        if not self.use_moveit:
            raise ValueError('MoveIt! is not initialized, '
                        'did you pass in use_moveit=True?')
        
        return self.moveit_arm_group.get_current_pose()

    def move_to_home(self):
        """
        Move the arm to its home position
        """
        return self.move_to_joint_positions(self.home_position)

    def set_end_effector_link(self, link_name):
        if not self.use_moveit:
            raise ValueError('MoveIt! is not initialized, '
                             'did you pass in use_moveit=True?')
        
        self.moveit_arm_group.set_end_effector_link(str(link_name))


    def get_end_effector_link(self):
        if not self.use_moveit:
            raise ValueError('MoveIt! is not initialized, '
                             'did you pass in use_moveit=True?')
        
        return self.moveit_arm_group.get_end_effector_link()
        
    def set_goal_tolerance(self, goal_tolerance=0.02):
        if not self.use_moveit:
            raise ValueError('MoveIt! is not initialized, '
                             'did you pass in use_moveit=True?')
        
        self.moveit_arm_group.set_goal_tolerance(goal_tolerance)

    def move_to_joint_positions(self, joint_goals, plan=True, wait=True, **kwargs):
        """
        Move the arm to the specified joint angles.

        :param joint_goals: array of desired joint positions 
        :param plan: whether to plan to the joint_goals or not (e.g. using MoveIt!)
        :param wait_for_result: whether to wait for the arm to finish execution
        :type joint_goals: 
        :type plan: bool
        :type wait_for_result: bool
        :return: success
        :rtype: bool
        """

        success = False
        #https://github.com/facebookresearch/pyrobot/blob/master/src/pyrobot/core.py
        if isinstance(joint_goals, np.ndarray):
            joint_goals = joint_goals.flatten().tolist()
        if plan: 
            if not self.use_moveit:
                raise ValueError('MoveIt! is not initialized, '
                                 'did you pass in use_moveit=True?')

            rospy.loginfo('MoveIt! Motion Planning...')

            success = self.moveit_arm_group.go(joint_goals, wait)
            self.moveit_arm_group.stop()
        else:
            pass #TODO: implement non-moveit joint positions

        return success
    
    def move_to_pose(self, pose, vel=1.0, accel=1.0, wait=True):
        """
        Move to pose
        :param pose: Array of positions and orientations [x, y, z, qx, qy, qz, qw]
        :param vel: Velocity (fraction of max) [0.0, 1.0]
        :param wait: to wait for completition of motion or not
        :
        :type pose: 
        :type vel:
        :type accel: 
        :type wait: bool
        :
        :return: success
        :rtype: bool
        """

        if type(pose) is list:
            pose = list_to_pose(pose)
        
        self.moveit_arm_group.set_max_velocity_scaling_factor(vel)
        self.moveit_arm_group.set_max_acceleration_scaling_factor(accel)
        self.moveit_arm_group.set_pose_target(pose)
        success = self.moveit_arm_group.go(wait=wait)
        self.moveit_arm_group.stop()
        self.moveit_arm_group.clear_pose_targets()
        return success

    def plan_to_pose_optimized(self, pose, iterations=5):
        # Initialize local variables
        plans = []
        points = []

        # Set the target position
        self.moveit_arm_group.set_pose_target(pose)
        self.desired_pose = pose.pose

        # Perform planning
        # Perform multiple time to get the best path.
        for i in range(iterations):
            plans.append(self.moveit_arm_group.plan())
            points.append(len(plans[i].joint_trajectory.points))

        # Find shortest path
        current_plan = plans[points.index(min(points))]

        # Validate whether planning was successful
        #if plan_exists(self.current_plan):
        #    rospy.loginfo("Plan to pose planning successful.")
        return current_plan
        #else:
        #    rospy.logwarn("Plan to pose planning failed.")
        #    return current_plan

    def plan_to_pose(self, pose):
        self.moveit_arm_group.set_pose_target(pose)
        return self.moveit_arm_group.plan()

    def plan_to_pose_cartesian(self, pose, num_retries=50):
        #if type(pose) is list:
        #    pose = list_to_pose(pose)

        fraction = 0.0
        attempts = 0
        plan = None
        while fraction < 1.0 and attempts < num_retries:
            (plan, fraction) = self.moveit_arm_group.compute_cartesian_path(
                                                    [pose.pose],   # waypoints to follow
                                                    0.01,    # eef_step
                                                    0.0)      # jump_threshold
            attempts += 1

        if fraction < 0.75:
            rospy.logwarn("compute_cartesian_path was only able to compute %s of the path! :(", fraction)
            raise ValueError('Unable to plan entire path!')

        return plan

    def execute_plan_safe(self, p, wait=True):
        self.display_trajectory(p)
        plan_not_good = raw_input("Press Enter if the plan is good. If not, type 'n' and press Enter...") == "n"
        success = False
        if plan_not_good:
            success = False
            rospy.logwarn("Plan was not good, returning success = False...")
        else:
            success = self.moveit_arm_group.execute(p, wait)
        self.moveit_arm_group.stop()
        self.moveit_arm_group.clear_pose_targets()
        return success

    def execute_plan_unsafe(self, p, wait=True):
        self.display_trajectory(p)
        success = self.moveit_arm_group.execute(p, wait)
        self.moveit_arm_group.stop()
        self.moveit_arm_group.clear_pose_targets()
        return success

    def move_to_pose_cartesian(self, pose, wait=True): #no support for vel, accel scaling for cartesian https://answers.ros.org/question/291887/moveit-acceleration-scaling-factor-not-working/
        """
        Move to pose following a cartesian path
        :param pose: geometry_msgs Pose
        :param wait: to wait for completition of motion or not
        :
        :type pose: 
        :type wait: bool
        :
        :return: success
        :rtype: bool
        """

        if type(pose) is list:
            pose = list_to_pose(pose)

        (plan, fraction) = self.moveit_arm_group.compute_cartesian_path(
                                                [pose],   # waypoints to follow
                                                0.005,    # eef_step
                                                0.0)      # jump_threshold
        if fraction != 1.0:
            raise ValueError('Unable to plan entire path!')

        success = self.moveit_arm_group.execute(plan, wait=wait)
        self.moveit_arm_group.stop()
        self.moveit_arm_group.clear_pose_targets()
        return success

    def js_cb(self, msg):
        """
        Subscriber callback for arm joint state (position, velocity)
        
        :param msg: message published on "/joint_states" topic
        :type msg: sensor_msgs/JointState
        """
        self.last_joint_states = msg

    def get_current_joint_states(self):
        return self.last_joint_states

    def initialize_moveit(self):
        """
        Initialize MoveIt! 
        """
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()


        if self.arm_planning_group not in self.robot.get_group_names():
            raise ValueError('Group name %s is not valid. Options are %s' % (self.arm_planning_group, self.robot.get_group_names()))
        else:
            self.robot = moveit_commander.RobotCommander()
            self.moveit_arm_group = moveit_commander.MoveGroupCommander(self.arm_planning_group)
            self.moveit_arm_group.set_planner_id(self.moveit_planner)
            self.moveit_arm_group.set_planning_time(self.planning_time)
            self.moveit_arm_group.set_max_velocity_scaling_factor(self.max_vel)
            self.moveit_arm_group.set_max_acceleration_scaling_factor(self.max_accel)

            self.set_goal_tolerance(self.goal_tolerance)

            self.gripper = moveit_commander.MoveGroupCommander("hand")

            rospy.wait_for_service('compute_ik')
            self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

            rospy.wait_for_service('compute_fk')
            self.compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
            
            #self.scene = PlanningSceneInterface()
            #rospy.loginfo("Connecting to /get_planning_scene service")
            #self.scene_srv = rospy.ServiceProxy(
            #    '/get_planning_scene', GetPlanningScene)
            #self.scene_srv.wait_for_service()
            #rospy.loginfo("Connected.")

            # rospy.loginfo("Connecting to clear octomap service...")
            # self.clear_octomap_srv = rospy.ServiceProxy(
            #     '/clear_octomap', Empty)
            # self.clear_octomap_srv.wait_for_service()
            # rospy.loginfo("Connected!")

            #Publishers
            self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,
                                      queue_size=20)

    def display_trajectory(self, traj):
        if not self.use_moveit:
            raise ValueError('Cannot display trajectory because MoveIt! is not initialized, '
                            'did you pass in use_moveit=True?')
        else: 
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(traj)
            self.display_trajectory_publisher.publish(display_trajectory)
    
    def all_close(self, goal, actual, tolerance):
        """
        Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
        :param goal:       A list of floats, a Pose or a PoseStamped
        :param actual:     A list of floats, a Pose or a PoseStamped
        :param tolerance:  A float
        :returns: bool
        """

        all_equal = True

        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False

        elif type(goal) is PoseStamped:
            return all_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is Pose:
            return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

        return True

    def get_ik(self, target_pose, end_effector_link=None):
        
        if self.use_moveit:
            service_request = PositionIKRequest()
            service_request.group_name = self.arm_planning_group
            service_request.ik_link_name = end_effector_link
            service_request.pose_stamped = target_pose
            service_request.timeout.secs = self.planning_time
            service_request.avoid_collisions = True

            try:
                resp = self.compute_ik(ik_request = service_request)
                print(resp)
                return resp
            except rospy.ServiceException, e:
                print ("Service call failed: %s"%e)
        else:
            #Implement non-moveit IK
            pass

    def get_fk_pose(self, joint_states, fk_links, frame_id):

        if self.use_moveit:

            req = GetPositionFKRequest()
            req.header.frame_id = frame_id
            req.fk_link_names = [fk_links]
            req.robot_state.joint_state = joint_states
            try:
                resp = self.compute_fk.call(req)
                return resp.pose_stamped[0]
            except rospy.ServiceException as e:
                rospy.logerr("Service exception: " + str(e))
                resp = GetPositionFKResponse()
                resp.error_code = 99999  # Failure
                return resp
        else:
            #Implement non-moveit FK
            pass

    def get_current_fk_pose(self):
        while not rospy.is_shutdown() and self.last_joint_states is None:
            rospy.logwarn("Waiting for a /joint_states message...")
            rospy.sleep(0.1)
        
        return self.get_fk_pose(self.get_current_joint_states(), "panda_gripper_center", "panda_link0")

    #@abstractmethod
    #def create_static_planning_scene(self, **kwargs):
    #    pass

    @abstractmethod
    def open_gripper(self, **kwargs):
        pass

    @abstractmethod
    def close_gripper(self, **kwargs):
        pass

    #@abstractmethod
    #def get_rgb_image(self, **kwargs):
    #    pass
    