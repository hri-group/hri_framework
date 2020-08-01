import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rbx2_msgs.srv import *
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

#implement behaviour tree sequencing here
#think about how to handle failures
#implement human tracking somehow


#blocking wait state
class Wait(smach.State):
    def __init__(self, time):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.wait_time = time
    def execute(self, userdata):
        rospy.sleep(self.wait_time)
        return 'succeeded'


#open gripper
#setup static planning scene

@smach.cb_interface(outcomes=['succeeded', 'aborted'])
def move_to_home(user_data, self):
    rospy.sleep(2)
    self.loginfo('State machine transitioning: MOVE_TO_HOME: succeeded --> MOVE_TO_HOVER_POSE_PART1')
    return 'succeeded'
