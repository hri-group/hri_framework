#!/usr/bin/env python
import rospy
from pi_trees_ros.pi_trees_ros import *
#from pi_trees_lib.task_setup import *
import time
from std_msgs.msg import String

from hrca_msgs.msg import RobotTaskAction, RobotTaskFeedback, RobotTaskResult, RobotTaskGoal
from hrca_msgs.srv import *

from panda_toolbox_task.srv import *

#from hrca_action.panda_arm import PandaArm
from hrca_action.utilities import *

from std_msgs.msg import Empty

#implement behaviour tree sequencing here
#think about how to handle failures
#implement human tracking somehow

#Handoff joint_states: [0.02724037481402535, 0.01846249802655966, 0.5576693427471922, -0.628088489635652, 0.5986106369494069, -2.1793691500195997, 0.3389351049167706, 1.6314189066026232, 0.2321968310216178]

#Handoff PoseStamped:
"""
header: 
  seq: 0
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: "panda_link0"
pose: 
  position: 
    x: 0.028444374064
    y: 0.473179560566
    z: 0.71309619749
  orientation: 
    x: 0.999999651932
    y: 0.00039838079135
    z: 0.000721627033823
    w: 0.000129162858922
"""

#Handle PoseStamped:
"""
header: 
  seq: 0
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: "panda_link0"
pose: 
  position: 
    x: 0.506354169585
    y: -0.0197519056065
    z: 0.0362904433437
  orientation: 
    x: 0.725532640124
    y: -0.688159606267
    z: -0.000597473537218
    w: -0.00619576012251
"""

CM_TO_M = 0.01
INCHES_TO_CM = 2.54

pass_pose = create_pose_stamped(create_pose(0.156796253944, 0.25346570163, 0.50487982588, 1, 0, 0, 0), "panda_link0")
handle_pick_pose = create_pose_stamped(offset_and_rotate_goal(create_pose(0.506354169585, -0.0197519056065, 0.0362904433437, 1, 0, 0, 0), 0 , 0, 0, 0, 0, 0, 90, replace_orientation = False), "panda_link0")

class TaskStatus(object):
    """ A class for enumerating task statuses """
    FAILURE = 0
    SUCCESS = 1
    RUNNING = 2


#store items that we want to track throughout the task
class BlackBoard():
    def __init__(self):
        self.battery_level = None
        self.charging = None

class ToolboxTaskTree():
    def __init__(self):

        rospy.loginfo("Connecting to Perception server")
        self.perception_srv = rospy.ServiceProxy('tbt_perception_data_request_server', ContextDataRequest)
        self.perception_srv.wait_for_service()
        rospy.loginfo("Connected!")

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

        # How frequently do we "tic" the tree?
        rate = rospy.get_param('~rate', 10)

        # Convert tic rate to a ROS rate
        tic = rospy.Rate(rate)

        # Where should the DOT file be stored. Default location is $HOME/.ros/tree.dot
        dotfilepath = rospy.get_param('~dotfilepath', None)

        # Create a list to hold the PICK_HANDOFF tasks
        PICK_PASS_TASKS = list()
        
        human_screw_parts_1 = Count("H: Screw Parts Together 1", 1, 5, 1)
        human_screw_parts_2 = Count("H: Screw Parts Together 2", 1, 4, 1)
        human_pick_screws_tools_1 = Count("H: P, screws and tools 1", 1, 3, 1)
        human_pick_screws_tools_2 = Count("H: P, screws and tools 2", 1, 3, 1)

        HUMAN_TASKS = list()

        HUMAN_TASKS.append(human_screw_parts_1)
        HUMAN_TASKS.append(human_screw_parts_2)
        HUMAN_TASKS.append(human_pick_screws_tools_1)
        HUMAN_TASKS.append(human_pick_screws_tools_2)

        # Create simple action pick and handoff tasks
        for i in range(1, 6):
            goal = RobotTaskGoal()
            goal.action = "pick_and_pass"
            current_gripper_orientation = get_robot_pose_srv("world").pose_stamped.pose.orientation
            goal.pose1 = create_pose_stamped(offset_and_rotate_goal(self.context_data_request_client(i, "grasp_point_1", "Aruco"), current_gripper_orientation, -1*CM_TO_M, 0, 0, 0, 0, 0), "panda_link0")
            goal.pose2 = pass_pose
            goal.object_name = "part_" + str(i)
            
            pick_pass_obj = SimpleActionTask("R: PICK_PASS_OBJ_" + str(i), "panda_arm_server", RobotTaskAction, goal, reset_after=False, result_timeout=90)
            
            PICK_PASS_TASKS.append(pick_pass_obj)

        goal = RobotTaskGoal()
        goal.action = "pick_and_pass"
        current_gripper_orientation = get_robot_pose_srv("world").pose_stamped.pose.orientation
        goal.pose1 = handle_pick_pose
        goal.pose2 = pass_pose
        goal.object_name = ""
        pick_pass_handle_obj = SimpleActionTask("R: PICK_PASS_OBJ_" + "HANDLE", "panda_arm_server", RobotTaskAction, goal, reset_after=False, result_timeout=90)
        
        PICK_PASS_TASKS.append(pick_pass_handle_obj)
        
        # Create a list to hold the HOLD tasks
        HOLD_TASKS = list()

        # Create a list to hold the PICK and HOLD tasks
        PICK_HOLD_TASKS = list()

        # Create simple action hold tasks
        #for i in range(2, 4):
            # goal = RobotTaskGoal()
            # goal.action = "hold"
            # current_gripper_orientation = get_robot_pose_srv("world").pose_stamped.pose.orientation
            # goal.pose1 = create_pose_stamped(offset_and_rotate_goal(self.context_data_request_client(i, "hold_point_1", "Aruco"), current_gripper_orientation, 0, 0, 0, 0, 0, 0), "panda_link0")
            # goal.pose2 = pass_pose

            # hold_obj = SimpleActionTask("R: HOLD_OBJ_" + str(i), "panda_arm_server" , RobotTaskAction, goal, reset_after=False)

            # HOLD_TASKS.append(hold_obj)

        # Create simple action hold tasks
        """
        for i in range(2, 4):
            goal = RobotTaskGoal()
            goal.action = "pick_and_hold"
            current_gripper_orientation = get_robot_pose_srv("world").pose_stamped.pose.orientation
            goal.pose1 = create_pose_stamped(offset_and_rotate_goal(self.context_data_request_client(i, "hold_point_1", "Aruco"), current_gripper_orientation, 0, 0, 0, 0, 0, 0), "panda_link0")
            goal.pose2 = create_pose_stamped(offset_and_rotate_goal(self.context_data_request_client(2, "grasp_point_1", "Aruco"), current_gripper_orientation, 0, 0, 0, 0, 0, 0), "panda_link0")

            hold_obj = SimpleActionTask("R: PICK_HOLD_OBJ_" + str(i), "panda_arm_server" , RobotTaskAction, goal, reset_after=False)

            PICK_HOLD_TASKS.append(hold_obj)
        """

        #PICK_PASS_TASKS[1],
        #PICK_PASS_TASKS[3],
        #PICK_PASS_TASKS[4], 
        HUMAN_DONE_1 = MonitorTask("HUMAN_TASK_1", "human_done_task_1", Empty, self.human_done, wait_for_message = False)
        HUMAN_DONE_2 = MonitorTask("HUMAN_TASK_2", "human_done_task_2", Empty, self.human_done, wait_for_message = False)
        HUMAN_DONE_3 = MonitorTask("HUMAN_TASK_3", "human_done_task_3", Empty, self.human_done, wait_for_message = False)
        HUMAN_DONE_4 = MonitorTask("HUMAN_TASK_4", "human_done_task_4", Empty, self.human_done, wait_for_message = False)
        HUMAN_DONE_5 = MonitorTask("HUMAN_TASK_5", "human_done_task_5", Empty, self.human_done, wait_for_message = False)
        HUMAN_DONE_6 = MonitorTask("HUMAN_TASK_6", "human_done_task_6", Empty, self.human_done, wait_for_message = False)
        HUMAN_DONE_7 = MonitorTask("HUMAN_TASK_7", "human_done_task_7", Empty, self.human_done, wait_for_message = False)

        ACTION_OVERRIDE_1 = MonitorTask("Action Override Obj 4", "action_override_obj_4", Empty, self.human_done, wait_for_message = False)
        ACTION_OVERRIDE_2 = MonitorTask("Action Override Obj 5", "action_override_obj_5", Empty, self.human_done, wait_for_message = False)
        ACTION_OVERRIDE_3 = MonitorTask("Action Override Obj Handle", "action_override_obj_handle", Empty, self.human_done, wait_for_message = False)

        Bottom_Side_1_Gathering = Sequence("Bottom Part and Side 1 Gathering Sequence", [PICK_PASS_TASKS[0], 
                                                                                    PICK_PASS_TASKS[2],
                                                                                    #HOLD_TASKS[0],
                                                                                    HUMAN_DONE_1])

        Bottom_Side_2_Assembly = Sequence("Bottom Part and Side 2 Assembly Sequence", [#PICK_HOLD_TASKS[0], 
                                                                            HUMAN_TASKS[1]])

        Bottom_Side_1_Assembly_Final = Sequence("Bottom Part and Side 1 Assembly Sequence")

        Bottom_Side_2_Assembly_Final = Sequence("Bottom Part and Side 2 Assembly Sequence")

        Bottom_Side_Front_Assembly_Final = Sequence("Bottom Part and Side Front Assembly Sequence")

        Bottom_Side_Back_Assembly_Final = Sequence("Bottom Part and Side Back Assembly Sequence")

        ParallelOverride_1 = ParallelOne("Pick and Pass Object 4")
        ParallelOverride_2 = ParallelOne("Pick and Pass Object 5")
        ParallelOverride_3 = ParallelOne("Pick and Pass Object Handle")
        
        goal = RobotTaskGoal()
        goal.action = "open_gripper"
        open_gripper_obj_4 = SimpleActionTask("R: Open_Gripper_Obj_4", "panda_arm_server", RobotTaskAction, goal, reset_after=False, result_timeout=120)
        open_gripper_obj_5 = SimpleActionTask("R: Open_Gripper_Obj_5", "panda_arm_server", RobotTaskAction, goal, reset_after=False, result_timeout=120)

        open_gripper_obj_handle = SimpleActionTask("R: Open_Gripper_Obj_HANDLE", "panda_arm_server", RobotTaskAction, goal, reset_after=False, result_timeout=120)


        override1_seq = Sequence("Override Pick and Pass Object 4", [ACTION_OVERRIDE_1, open_gripper_obj_4])
        override2_seq = Sequence("Override Pick and Pass Object 5", [ACTION_OVERRIDE_2, open_gripper_obj_5])
        override3_seq = Sequence("Override Pick and Pass Object Handle", [ACTION_OVERRIDE_3, open_gripper_obj_handle])

        ParallelOverride_1.add_child(PICK_PASS_TASKS[3])
        ParallelOverride_1.add_child(override1_seq)

        ParallelOverride_2.add_child(PICK_PASS_TASKS[4])
        ParallelOverride_2.add_child(override2_seq)

        ParallelOverride_3.add_child(PICK_PASS_TASKS[5])
        ParallelOverride_3.add_child(override3_seq)

        #Bottom_Side_Front_Assembly_Final.add_child(ParallelOverride_1)

        #Bottom_Side_Back_Assembly_Final.add_child(ParallelOverride_2)

        #PARALLEL_1 = ParallelAll("Assemble Bottom Part and Side 1")

        #PARALLEL_1.add_child(Bottom_Side_1_Assembly)
        #PARALLEL_1.add_child(HUMAN_TASKS[2])

        #PARALLEL_2 = ParallelAll("Assemble Bottom Part and Side 2")

        #PARALLEL_2.add_child(Bottom_Side_2_Assembly)
        #PARALLEL_2.add_child(HUMAN_TASKS[3])

        # Create the root node
        BEHAVE = Sequence("Toolbox Task")
        BEHAVE.add_child(Bottom_Side_1_Gathering)
        #BEHAVE.add_child(PARALLEL_2)

        dot_tree = print_dot_tree(BEHAVE, dotfilepath)

        dot_tree_pub = rospy.Publisher('tbt_dot_tree', String, queue_size=10)
        dot_tree_pub.publish(dot_tree)
        rospy.sleep(2.0)

        raw_input("Press Enter to start the Toolbox Task...")

        first_hold = True
        second_hold = True
        # Run the tree
        while True:
            status = BEHAVE.run()
            tic.sleep()
            dot_tree = print_dot_tree(BEHAVE, dotfilepath)
            dot_tree_pub.publish(dot_tree)
            
            if status == TaskStatus.SUCCESS:
                if first_hold:
                    BEHAVE.set_status(TaskStatus.RUNNING)
                    goal = RobotTaskGoal()
                    goal.action = "hold"
                    current_gripper_orientation = get_robot_pose_srv("world").pose_stamped.pose.orientation
                    goal.pose1 = create_pose_stamped(offset_and_rotate_goal(self.context_data_request_client(3, "hold_point_1", "Aruco"), current_gripper_orientation, 0, 0, 0, 0, 0, 0), "panda_link0")
                    goal.object_name = "part_3"
                    hold_obj_3 = SimpleActionTask("R: HOLD_OBJ_" + "3", "panda_arm_server" , RobotTaskAction, goal, reset_after=False, result_timeout=60)

                    #goal = RobotTaskGoal()
                    #goal.action = "open_gripper"
                    #open_gripper_1 = SimpleActionTask("R: Open Gripper", "panda_arm_server" , RobotTaskAction, goal, reset_after=False, result_timeout=60)

                    Bottom_Side_1_Assembly_Final.add_child(hold_obj_3)
                    Bottom_Side_1_Assembly_Final.add_child(HUMAN_DONE_2)
                    BEHAVE.add_child(Bottom_Side_1_Assembly_Final)
                    Bottom_Side_2_Assembly_Final.add_child(PICK_PASS_TASKS[1])
                    Bottom_Side_2_Assembly_Final.add_child(HUMAN_DONE_3)
                    BEHAVE.add_child(Bottom_Side_2_Assembly_Final)
                    
                    first_hold = False
                elif second_hold:
                    BEHAVE.set_status(TaskStatus.RUNNING)
                    goal = RobotTaskGoal()
                    goal.action = "hold"
                    current_gripper_orientation = get_robot_pose_srv("world").pose_stamped.pose.orientation
                    goal.pose1 = create_pose_stamped(offset_and_rotate_goal(self.context_data_request_client(2, "hold_point_1", "Aruco"), current_gripper_orientation, 0, 0, 0, 0, 0, 0), "panda_link0")
                    goal.object_name = "part_2"
                    hold_obj_2 = SimpleActionTask("R: HOLD_OBJ_" + "2", "panda_arm_server" , RobotTaskAction, goal, reset_after=False, result_timeout=120)
                    Bottom_Side_2_Assembly_Final.add_child(hold_obj_2)
                    Bottom_Side_2_Assembly_Final.add_child(HUMAN_DONE_4)

                    goal = RobotTaskGoal()
                    goal.action="home"
                    go_home = SimpleActionTask("R: MOVE_TO_HOME" + str(i), "panda_arm_server", RobotTaskAction, goal, reset_after=False, result_timeout=90)
                    Bottom_Side_2_Assembly_Final.add_child(go_home)
                    BEHAVE.add_child(Bottom_Side_2_Assembly_Final)

                    Bottom_Side_Front_Assembly_Final.add_child(PICK_PASS_TASKS[3])
                    Bottom_Side_Front_Assembly_Final.add_child(HUMAN_DONE_5)
                    Bottom_Side_Back_Assembly_Final.add_child(PICK_PASS_TASKS[4])
                    Bottom_Side_Back_Assembly_Final.add_child(HUMAN_DONE_6)

                    BEHAVE.add_child(Bottom_Side_Front_Assembly_Final)
                    BEHAVE.add_child(Bottom_Side_Back_Assembly_Final)
                    BEHAVE.add_child(PICK_PASS_TASKS[5])
                    #BEHAVE.add_child(ParallelOverride_3)
                    BEHAVE.add_child(HUMAN_DONE_7)

                    second_hold = False
                else:
                    print "Finished running tree."
                    break




                """
                robot wait until enter pressed
                robot open gripper
                robot 
                robot pick and pass obj 2
                """


    def context_data_request_client(self, object_id, query_point, tracking_system):
        try:
            context_data_response = self.perception_srv(int(object_id), str(query_point), str(tracking_system))
            return context_data_response.object_pose
        except rospy.ServiceException, e:
            print ("Service call failed: %s"%e)

    def human_done(self, msg):
        return TaskStatus.SUCCESS

# A simple counting task to simulate placeholder tasks
class Count(Task):
    def __init__(self, name, start, stop, step, *args, **kwargs):
        super(Count, self).__init__(name, *args, **kwargs)
        
        self.name = name
        self.start = start
        self.stop = stop
        self.step = step
        self.count = self.start
        print "Creating task Count", self.start, self.stop, self.step
 
    def run(self):
        if abs(self.count - self.stop - self.step) <= 0:
            return TaskStatus.SUCCESS
        else:
            print self.name, self.count
            time.sleep(0.5)
            self.count += self.step
            if abs(self.count - self.stop - self.step) <= 0:
                return TaskStatus.SUCCESS
            else:
                return TaskStatus.RUNNING

    def reset(self):
        self.count = self.start

if __name__ == '__main__':
    rospy.init_node("toolbox_task_tree")
    tree = ToolboxTaskTree()


