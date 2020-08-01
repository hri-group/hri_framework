#! /usr/bin/env python
import rospy
import actionlib
from hrca_action.panda_arm import PandaArm 
from hrca_msgs.msg import RobotTaskAction, RobotTaskFeedback, RobotTaskResult
from hrca_msgs.srv import GetRobotPose
from hrca_action.utilities import *

import smach
import smach_ros
from smach import CBState
from smach_ros import MonitorState

from std_msgs.msg import Empty
#import copy 

class ArmServer(object):
    # create messages that are used to publish feedback/result
    _feedback = RobotTaskFeedback()
    _result = RobotTaskResult()


    def __init__(self, name, arm, moveitobjecthandler):
        self._action_name = name
        self._arm = arm
        self._moh = moveitobjecthandler
        
        rospy.Service("%s/get_robot_pose" % rospy.get_name()[1:], GetRobotPose, self.get_robot_pose_service)
        #rospy.Service("%s/move_to_home" % rospy.get_name()[1:], MoveToPose, self.move_to_home)
        #rospy.Service("%s/pick_and_pass" % rospy.get_name()[1:], MoveToPose, self.move_to_pose)

        self._as = actionlib.SimpleActionServer(self._action_name, RobotTaskAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.logwarn("Starting %s", self._action_name)


    def arm_action(self, arm, action, pose1, pose2, object_name, hover_distance=0.1):

        #Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['done', 'failed', 'aborted'])

        sm.userdata.sm_pick_object_pose = pose1#Pose()
        sm.userdata.sm_place_object_pose = pose2#Pose()
        sm.userdata.sm_hover_distance = hover_distance
        sm.userdata.sm_object_name = object_name

        @smach.cb_interface(outcomes=['succeeded', 'aborted', 'failed'], output_keys=['pick_object_pose','hover_distance','place_object_pose', 'object_name'])
        def set_pick_place_pose(userdata):
            userdata.pick_object_pose = pose1
            userdata.hover_distance = hover_distance
            userdata.place_object_pose = pose2
            userdata.object_name = object_name
            return 'succeeded'

        @smach.cb_interface(outcomes=['succeeded', 'aborted', 'failed_hover'], input_keys=['pick_object_pose','hover_distance'])
        def move_to_pick_hover_pose(userdata):
            
            pick_object_pose = copy.deepcopy(userdata.pick_object_pose)
            pick_object_pose.pose.position.z = pick_object_pose.pose.position.z + userdata.hover_distance
            
            plan = self._arm.plan_to_pose_optimized(pick_object_pose)
            #hover_success = self._arm.execute_plan_safe(plan)
            hover_success = self._arm.execute_plan_unsafe(plan)

            #print(pick_object_pose)
            #hover_success = self._arm.move_to_pose(pick_object_pose)
            #print(hover_success)

            if hover_success:
                return 'succeeded'
            else:
                return 'failed_hover'

        @smach.cb_interface(outcomes=['succeeded', 'aborted', 'failed_pick_pose'], input_keys=['pick_object_pose'])
        def move_to_pick_pose(userdata):
            pick_object_pose = copy.deepcopy(userdata.pick_object_pose)

            plan = self._arm.plan_to_pose_optimized(pick_object_pose, 10)
            #pick_pose_success = self._arm.execute_plan_safe(plan)
            pick_pose_success = self._arm.execute_plan_unsafe(plan)
            #pick_pose_success = self._arm.move_to_pose(userdata.pick_object_pose)

            if pick_pose_success:
                return 'succeeded'
            else:
                return 'failed_pick_pose'

        @smach.cb_interface(outcomes=['succeeded', 'aborted', 'failed_grasp', 'failed'], input_keys=['pick_object_pose','hover_distance', 'object_name'])
        def grasp_and_hover(userdata):

            grasp_success = False

            #raw_input("Press Enter to grasp...")
            if arm.simulation:
                self._arm.close_gripper()
                grasp_success = True
            else:
                grasp_success = self._arm.grasp(width=0.01, force = 45)

            if grasp_success:
                rospy.sleep(0.5)
                print(userdata.object_name)
                self._moh.attach_gripper_object(userdata.object_name, self._arm, "hand")
                pick_object_pose = copy.deepcopy(userdata.pick_object_pose)
                pick_object_pose.pose.position.z = pick_object_pose.pose.position.z + userdata.hover_distance + 0.1

                plan = self._arm.plan_to_pose_cartesian(pick_object_pose)
                #hover_success = self._arm.execute_plan_safe(plan)
                hover_success = self._arm.execute_plan_unsafe(plan)
                #hover_success = self._arm.move_to_pose(pick_object_pose)
            else:
                return 'failed_grasp'

            if hover_success:
                return 'succeeded'
            else:
                return 'failed'
        
        @smach.cb_interface(outcomes=['succeeded', 'aborted', 'failed_hover'], input_keys=['place_object_pose','hover_distance'])
        def move_to_place_hover_pose(userdata):
            place_object_pose = copy.deepcopy(userdata.place_object_pose)
            place_object_pose.pose.position.z = place_object_pose.pose.position.z + userdata.hover_distance

            plan = self._arm.plan_to_pose_optimized(place_object_pose,10)
            #hover_success = self._arm.execute_plan_safe(plan)
            hover_success = self._arm.execute_plan_unsafe(plan)
            
            #hover_success = True

            #hover_success = self._arm.move_to_pose(place_object_pose)

            if hover_success:
                return 'succeeded'
            else:
                return 'failed_hover'

        @smach.cb_interface(outcomes=['succeeded', 'aborted', 'failed_place_pose'], input_keys=['place_object_pose'])
        def move_to_place_pose(userdata):
            place_object_pose = copy.deepcopy(userdata.place_object_pose)

            #plan = self._arm.plan_to_pose_optimized(place_object_pose)
            #place_pose_success = self._arm.execute_plan_safe(plan)
            #place_pose_success = self._arm.execute_plan_unsafe(plan)

            #place_pose_success = self._arm.move_to_pose(userdata.place_object_pose)

            #if place_pose_success:
            return 'succeeded'
            #else:
            #    return 'failed_place_pose'

        @smach.cb_interface(outcomes=['done', 'aborted', 'failed_place', 'failed'], input_keys=['place_object_pose','hover_distance'])
        def place_and_hover(userdata):

            place_success = False
            #raw_input("Press Enter to open gripper...")
            self._arm.open_gripper() #need some error checking

            rospy.sleep(0.5)
            self._moh.detach_gripper_object(userdata.object_name, self._arm, True)
            place_object_pose = copy.deepcopy(userdata.place_object_pose)
            place_object_pose.pose.position.z = place_object_pose.pose.position.z + userdata.hover_distance

            plan = self._arm.plan_to_pose_optimized(place_object_pose)
            #hover_success = self._arm.execute_plan_safe(plan)
            hover_success = self._arm.execute_plan_unsafe(plan)

            #finished pipeline
            if hover_success:
                return 'done'
            else:
                return 'failed'

        def monitor_cb(ud, msg):
            return False

        @smach.cb_interface(outcomes=['done', 'aborted', 'failed_place', 'failed'], input_keys=['object_name'])
        def open_gripper(userdata):
            #raw_input("Press Enter to open gripper...")
            self._arm.open_gripper()
            self._moh.detach_gripper_object(userdata.object_name, self._arm, True)
            return "done"

        def loginfo(info):
            rospy.logwarn(str(info))

        with sm:
            smach.StateMachine.add('SET_PICK_POSE', CBState(set_pick_place_pose, cb_args=[]), 
                                    transitions={'succeeded':'MOVE_TO_PICK_HOVER_POSE', 'aborted': 'aborted'},
                                    remapping={'pick_object_pose':'sm_pick_object_pose', 
                                                'place_object_pose':'sm_place_object_pose', 
                                                'hover_distance':'sm_hover_distance'})

            smach.StateMachine.add('MOVE_TO_PICK_HOVER_POSE', CBState(move_to_pick_hover_pose, cb_args=[]), 
                                    transitions={'succeeded':'MOVE_TO_PICK_POSE', 'failed_hover':'failed', 'aborted': 'aborted'}, 
                                    remapping={'pick_object_pose':'sm_pick_object_pose', 
                                                'hover_distance':'sm_hover_distance'})

            smach.StateMachine.add('MOVE_TO_PICK_POSE', CBState(move_to_pick_pose, cb_args=[]), 
                                    transitions={'succeeded':'GRASP_AND_HOVER', 'failed_pick_pose': 'failed', 'aborted': 'aborted'},
                                    remapping={'pick_object_pose':'sm_pick_object_pose'})

            smach.StateMachine.add('GRASP_AND_HOVER', CBState(grasp_and_hover, cb_args=[]), 
                                    transitions={'succeeded':'MOVE_TO_PLACE_HOVER_POSE', 'failed_grasp': 'failed', 'aborted': 'aborted'},
                                    remapping={'pick_object_pose':'sm_pick_object_pose',
                                                'hover_distance':'sm_hover_distance'})
            
            if action == "pick_and_place":
                smach.StateMachine.add('MOVE_TO_PLACE_HOVER_POSE', CBState(move_to_place_hover_pose, cb_args=[]), 
                                        transitions={'succeeded':'MOVE_TO_PLACE_POSE', 'failed_hover': 'failed', 'aborted': 'aborted'},
                                        remapping={'place_object_pose':'sm_place_object_pose', 
                                                    'hover_distance':'sm_hover_distance'})

                smach.StateMachine.add('MOVE_TO_PLACE_POSE', CBState(move_to_place_pose, cb_args=[]), 
                                        transitions={'succeeded':'PLACE_AND_HOVER', 'failed_place_pose': 'failed', 'aborted': 'aborted'},
                                        remapping={'place_object_pose':'sm_place_object_pose'})

                smach.StateMachine.add('PLACE_AND_HOVER', CBState(place_and_hover, cb_args=[]), 
                                        transitions={'done':'done', 'failed_place': 'failed', 'aborted': 'aborted'},
                                        remapping={'place_object_pose':'sm_place_object_pose', 
                                                    'hover_distance':'sm_hover_distance'})
            elif action == "pick_and_pass":
                smach.StateMachine.add('MOVE_TO_PLACE_HOVER_POSE', CBState(move_to_place_hover_pose, cb_args=[]), 
                                        transitions={'succeeded':'MOVE_TO_PLACE_POSE', 'failed_hover': 'failed', 'aborted': 'aborted'},
                                        remapping={'place_object_pose':'sm_place_object_pose', 
                                                    'hover_distance':'sm_hover_distance'})

                smach.StateMachine.add('MOVE_TO_PLACE_POSE', CBState(move_to_place_pose, cb_args=[]), 
                                        transitions={'succeeded':'OPEN_GRIPPER', 'failed_place_pose': 'failed', 'aborted': 'aborted'},
                                        remapping={'place_object_pose':'sm_place_object_pose'})

                #smach.StateMachine.add('WAIT_TO_PASS_PART', MonitorState("/pass_part", Empty, monitor_cb), 
                #                        transitions={'invalid':'OPEN_GRIPPER', 'valid':'failed', 'preempted':'failed'})

                smach.StateMachine.add('OPEN_GRIPPER', CBState(open_gripper, cb_args=[]), 
                                        transitions={'done':'done', 'failed_place': 'failed', 'aborted': 'aborted'},
                                        remapping={'place_object_pose':'sm_place_object_pose', 
                                                    'hover_distance':'sm_hover_distance'})
            elif action == "pick_and_hold":
                smach.StateMachine.add('MOVE_TO_PLACE_HOVER_POSE', CBState(move_to_place_hover_pose, cb_args=[]), 
                                        transitions={'succeeded':'MOVE_TO_PLACE_POSE', 'failed_hover': 'failed', 'aborted': 'aborted'},
                                        remapping={'place_object_pose':'sm_place_object_pose', 
                                                    'hover_distance':'sm_hover_distance'})

                smach.StateMachine.add('MOVE_TO_PLACE_POSE', CBState(move_to_place_pose, cb_args=[]), 
                                        transitions={'succeeded':'done', 'failed_place_pose': 'failed', 'aborted': 'aborted'},
                                        remapping={'place_object_pose':'sm_place_object_pose'})
            else:
                raise ValueError('Please ensure that action is one of the predefined strings')

        outcome = sm.execute()
        if outcome == "done":
            return True
        else:
            return False
            
    def execute_cb(self, goal):
        success = True
        status = 'running'
        feedback = RobotTaskFeedback()
        result = RobotTaskResult()
        rate = rospy.Rate(10)

        pick_pose = None
        place_pose = None
        handoff_pose = None
        move_pose = None
        hold_pose = None

        object_name = goal.object_name

        if goal.action == "pick_and_place":
            pick_pose = goal.pose1
            place_pose = goal.pose2
            success = self.arm_action(self._arm, "pick_and_place", pick_pose, place_pose, object_name, 0.05)
        elif goal.action == "pick_and_pass":
            pick_pose = goal.pose1
            place_pose = goal.pose2
            success = self.arm_action(self._arm, "pick_and_pass", pick_pose, place_pose, object_name, 0.05)
        elif goal.action == "move_to_pose" or goal.action == "move_to_pose_cartesian":
            move_pose = goal.pose1
            success = self._arm.move_to_pose(move_pose)
        
        elif goal.action == "hold":

            success = False
            move_pose_hover = copy.deepcopy(goal.pose1)
            move_pose_hover.pose.position.z = move_pose_hover.pose.position.z + 0.15

            move_pose = goal.pose1

            #object_pose = goal.pose1.pose
            #object_pose.position.x = object_pose.position.x + 0.12 #offset by query point
            #self._moh.add_box_object(object_name, object_pose, size=(0.282, 0.101, 0.01), frame="panda_link0")
            plan = self._arm.plan_to_pose_optimized(move_pose_hover)
            move_success = self._arm.execute_plan_unsafe(plan)

            plan = self._arm.plan_to_pose_cartesian(move_pose)
            move_success = self._arm.execute_plan_unsafe(plan)
            
            if move_success:
                if self._arm.simulation:
                    success = self.close_gripper()
                else:
                    success = self._arm.grasp(width = 0.01, force=50)

            raw_input("Press Enter to open gripper...")
            self._arm.open_gripper()

            plan = self._arm.plan_to_pose_optimized(move_pose_hover)
            move_success = self._arm.execute_plan_unsafe(plan)

            if move_success:
                success = True
            else:
                success = False

        elif goal.action == "pick_and_hold":
            pick_pose = goal.pose1
            place_pose = goal.pose2
            success = self.arm_action(self._arm, "pick_and_hold", pick_pose, place_pose, object_name, 0.05)
        elif goal.action == "home":
            success = self._arm.move_to_home()
        elif goal.action == "open_gripper":
            self._arm.open_gripper()
            success = True
        elif goal.action == "close_gripper":
            self.close_gripper()
            success = True
        elif goal.action == "grasp":
            success = self._arm.grasp(width = 0.01, force=30)
        elif goal.action == "plan_to_pose_safe":
            move_pose = goal.pose1
            plan = self._arm.plan_to_pose(move_pose)
            success = self._arm.execute_plan_safe(plan)
        else:
            raise ValueError('Please ensure that goal.action is one of the predefined strings')

        result.success = success
        feedback.status = status
        self._as.publish_feedback(feedback)
        rate.sleep()

        if success:
            self._as.set_succeeded(result)
        else:
            self._as.set_aborted(result)

    def get_robot_pose_service(self, req):
        current_pose_world = self._arm.get_current_pose()

        if req.frame != self.strip_leading_slash(current_pose_world.header.frame_id):
            get_transform(current_pose_world, self.strip_leading_slash(current_pose_world.header.frame_id), str(req.frame))
        else:
            return self._arm.get_current_pose()

    

    def strip_leading_slash(self, s):
        return s[1:] if s.startswith("/") else s

if __name__ == '__main__':
    rospy.init_node('panda_arm_server')
    panda_arm = PandaArm(simulation=False)
    panda_arm.set_end_effector_link("panda_gripper_center")
    panda_arm.moveit_arm_group.set_pose_reference_frame("/panda_link0")
    panda_arm.move_to_home()
    panda_arm.open_gripper()

    moh = MoveitObjectHandler()

    server = ArmServer(rospy.get_name(), panda_arm, moh)

    rospy.spin()
