#!/usr/bin/python
import sys
import os.path
import copy
#sys.path.append('/home/sagarrajendran/hrca_ws/src/hrca_action/robots/panda')
import time

import rospy
sys.path.append(os.path.abspath(os.path.join('..', 'robots/panda')))
sys.path.append(os.path.abspath(os.path.join('..', 'src')))

#from panda_arm import PandaArm
from utilities import MoveitObjectHandler

from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)

import smach
import smach_ros
from smach import CBState

"""
Simple pick and place pipeline
"""
#remapping={'pick_object_pose':'sm_pick_object_pose', 'place_object_pose':'sm_place_object_pose'}

class PickAndPlace(object):

    def __init__(self, panda_arm, pick_pose, place_pose, hover_distance = 0.0, num_retries=2):

        self.panda_arm = panda_arm
        self.num_retries = num_retries

        self.pick_pose = pick_pose
        self.place_pose = place_pose

        #Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['done', 'failed', 'aborted'])

        self.sm.userdata.sm_pick_object_pose = Pose()
        self.sm.userdata.sm_place_object_pose = Pose()
        self.sm.userdata.sm_hover_distance = hover_distance

        with self.sm:
            smach.StateMachine.add('SET_PICK_POSE', CBState(self.set_pick_place_pose, cb_args=[self]), 
                                    transitions={'succeeded':'MOVE_TO_PICK_HOVER_POSE', 'aborted': 'aborted'},
                                    remapping={'pick_object_pose':'sm_pick_object_pose', 
                                                'place_object_pose':'sm_place_object_pose', 
                                                'hover_distance':'sm_hover_distance'})

            smach.StateMachine.add('MOVE_TO_PICK_HOVER_POSE', CBState(self.move_to_pick_hover_pose, cb_args=[self]), 
                                    transitions={'succeeded':'MOVE_TO_PICK_POSE', 'failed_hover':'failed', 'aborted': 'aborted'}, 
                                    remapping={'pick_object_pose':'sm_pick_object_pose', 
                                                'hover_distance':'sm_hover_distance'})

            smach.StateMachine.add('MOVE_TO_PICK_POSE', CBState(self.move_to_pick_pose, cb_args=[self]), 
                                    transitions={'succeeded':'GRASP_AND_HOVER', 'failed_pick_pose': 'failed', 'aborted': 'aborted'},
                                    remapping={'pick_object_pose':'sm_pick_object_pose'})

            smach.StateMachine.add('GRASP_AND_HOVER', CBState(self.grasp_and_hover, cb_args=[self]), 
                                    transitions={'succeeded':'MOVE_TO_PLACE_HOVER_POSE', 'failed_grasp': 'failed', 'aborted': 'aborted'},
                                    remapping={'hover_distance':'sm_hover_distance'})
            
            smach.StateMachine.add('MOVE_TO_PLACE_HOVER_POSE', CBState(self.move_to_place_hover_pose, cb_args=[self]), 
                                    transitions={'succeeded':'MOVE_TO_PLACE_POSE', 'failed_hover': 'failed', 'aborted': 'aborted'},
                                    remapping={'place_object_pose':'sm_place_object_pose', 
                                                'hover_distance':'sm_hover_distance'})

            smach.StateMachine.add('MOVE_TO_PLACE_POSE', CBState(self.move_to_place_pose, cb_args=[self]), 
                                    transitions={'succeeded':'PLACE_AND_HOVER', 'failed_place_pose': 'failed', 'aborted': 'aborted'},
                                    remapping={'place_object_pose':'sm_place_object_pose'})

            smach.StateMachine.add('PLACE_AND_HOVER', CBState(self.place_and_hover, cb_args=[self]), 
                                    transitions={'succeeded':'done', 'failed_grasp': 'failed', 'aborted': 'aborted'},
                                    remapping={'place_object_pose':'sm_place_object_pose', 
                                                'hover_distance':'sm_hover_distance'})


    @smach.cb_interface(outcomes=['succeeded', 'aborted', 'failed'], output_keys=['pick_object_pose','hover_distance','place_object_pose'])
    def set_pick_place_pose(userdata, self):
        userdata.pick_object_pose = self.pick_pose
        #userdata.hover_distance = 0.15
        userdata.place_object_pose = self.place_pose
        return 'succeeded'

    @smach.cb_interface(outcomes=['succeeded', 'aborted', 'failed_hover'], input_keys=['pick_object_pose','hover_distance'])
    def move_to_pick_hover_pose(userdata, self):
        
        pick_object_pose = copy.deepcopy(userdata.pick_object_pose)
        pick_object_pose.position.z = pick_object_pose.position.z + userdata.hover_distance

        print(pick_object_pose)
        hover_success = self.panda_arm.move_to_pose(pick_object_pose)

        print(hover_success)

        if hover_success:
            return 'succeeded'
        else:
            return 'failed_hover'

    @smach.cb_interface(outcomes=['succeeded', 'aborted', 'failed_pick_pose'], input_keys=['pick_object_pose'])
    def move_to_pick_pose(userdata, self):
        pick_object_pose = copy.deepcopy(userdata.pick_object_pose)

        pick_pose_success = self.panda_arm.move_to_pose(userdata.pick_object_pose)

        if pick_pose_success:
            return 'succeeded'
        else:
            return 'failed_pick_pose'

    @smach.cb_interface(outcomes=['succeeded', 'aborted', 'failed_grasp', 'failed'], input_keys=['pick_object_pose','hover_distance'])
    def grasp_and_hover(userdata, self):

        grasp_success = False
        if self.panda_arm.simulation:
            self.panda_arm.close_gripper()
            grasp_success = True
        else:
            grasp_success = self.panda_arm.grasp(width=0.01)

        if grasp_success:
            rospy.sleep(0.5)
            pick_object_pose = copy.deepcopy(userdata.pick_object_pose)
            pick_object_pose.position.z = pick_object_pose.position.z + userdata.hover_distance
            hover_success = self.panda_arm.move_to_pose(pick_object_pose)
        else:
            return 'failed_grasp'

        if hover_success:
            return 'succeeded'
        else:
            return 'failed'
    
    @smach.cb_interface(outcomes=['succeeded', 'aborted', 'failed_hover'], input_keys=['place_object_pose','hover_distance'])
    def move_to_place_hover_pose(userdata, self):
        place_object_pose = copy.deepcopy(userdata.place_object_pose)
        place_object_pose.position.z = place_object_pose.position.z + userdata.hover_distance

        hover_success = self.panda_arm.move_to_pose(place_object_pose)

        if hover_success:
            return 'succeeded'
        else:
            return 'failed_hover'

    @smach.cb_interface(outcomes=['succeeded', 'aborted', 'failed_place_pose'], input_keys=['place_object_pose'])
    def move_to_place_pose(userdata, self):
        place_object_pose = copy.deepcopy(userdata.place_object_pose)

        place_pose_success = self.panda_arm.move_to_pose(userdata.place_object_pose)

        if place_pose_success:
            return 'succeeded'
        else:
            return 'failed_place_pose'

    @smach.cb_interface(outcomes=['succeeded', 'aborted', 'failed_grasp', 'failed'], input_keys=['place_object_pose','hover_distance'])
    def place_and_hover(userdata, self):

        place_success = False
        if self.panda_arm.simulation:
            self.panda_arm.open_gripper()
            place_success = True
        else:
            place_success = self.panda_arm.grasp(width=0.01)

        if place_success:
            rospy.sleep(0.5)
            place_object_pose = copy.deepcopy(userdata.place_object_pose)
            place_object_pose.position.z = place_object_pose.position.z + userdata.hover_distance
            hover_success = self.panda_arm.move_to_pose(place_object_pose)
        else:
            return 'failed_place'

        #finished pipeline
        if hover_success:
            return 'done'
        else:
            return 'failed'

    def execute(self):
        outcome = self.sm.execute()
        if outcome == "done":
            return True
        else:
            return False

    def get_sm(self):
        return self.sm

    def loginfo(self, info):
        rospy.logwarn(str(info))
"""
if __name__ == "__main__":
    rospy.init_node('pick_place_pipeline', log_level=rospy.WARN)

    from panda_toolbox_task.srv import *
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

    panda_arm = PandaArm(simulation=True)

    panda_arm.set_end_effector_link("panda_gripper_center")

    panda_arm.move_to_home()
    panda_arm.open_gripper()

    moh = MoveitObjectHandler()
    bottom_part_size = (0.01, 0.2,0.3)
    rotation = (0, 90, 0)
    moh.add_box_object("bottom_part", pick_pose, bottom_part_size, rotation)

    try:
        pick_and_place = PickAndPlace(panda_arm=panda_arm, pick_pose=pick_pose, place_pose=pick_pose)

        sis = smach_ros.IntrospectionServer('PickAndPlace', pick_and_place.get_sm(), '/PickAndPlace')
        sis.start()

        pick_and_place.execute()

        rospy.spin()
        sis.stop()
    except rospy.ROSInterruptException:
        pass
"""
