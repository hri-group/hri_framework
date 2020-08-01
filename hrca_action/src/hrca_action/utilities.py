import rospy
import tf
import numpy as np
import copy

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

from moveit_commander import PlanningSceneInterface
from moveit_commander.conversions import list_to_pose
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, GetPlanningSceneResponse

import numpy as np
import math
from math import radians, pi

import tf2_ros
import tf
import tf_conversions

from tf import transformations
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, unit_vector, quaternion_multiply

from std_srvs.srv import Empty, EmptyRequest

def create_pose_stamped(pose, frame_id):
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose = pose
    return ps

def create_pose(x, y, z, qx, qy, qz, qw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    #q = euler_to_quat(r, p, y)

    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose

def get_transform(pose_stamped, source_frame, target_frame):
    try:
        tfBuffer = tf2_ros.Buffer()
        transform = tfBuffer.lookup_transform(target_frame,
                                    source_frame,
                                    rospy.Time(0), #get the tf at first available time
                                    rospy.Duration(1.0)) #wait for 1 second

        pose_transformed = do_transform_pose(pose_stamped, transform)
        return pose_transformed
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Error getting transform")

#https://github.com/facebookresearch/pyrobot/blob/master/src/pyrobot/utils/util.py
def euler_to_quat(r, p, y):
    return tf.transformations.quaternion_from_euler(r, p, y)

def offset_and_rotate_goal(pose, quat, x, y, z, roll, pitch, yaw, replace_orientation=True):
    pose_goal = Pose()
    pose_goal = copy.deepcopy(pose)

    if replace_orientation:
        pose_goal.orientation = quat
    q_orientation = [pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]

    q_rotated = quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))
    q = quaternion_multiply(q_orientation, q_rotated)
    roll, pitch, yaw = euler_from_quaternion(q)

    q = quaternion_from_euler(roll, pitch, yaw)

    x = pose_goal.position.x + x
    y = pose_goal.position.y + y
    z = pose_goal.position.z + z
    return Pose(Point(x, y, z), Quaternion(*q))

#Known issues with meshes: https://answers.ros.org/question/316515/cannot-add-mesh-files-to-moveit/,
# https://github.com/ros-planning/moveit/issues/86
#https://github.com/facebookresearch/pyrobot/blob/master/src/pyrobot/utils/util.py
class MoveitObjectHandler(object):
    '''
    Use this class to create objects that reside in moveit environments
    '''
    def __init__(self):
        '''
        Constructor of the MoveitObjectHandler class.
        '''

        self.planning_scene_interface = PlanningSceneInterface()
        rospy.loginfo("Connecting to /get_planning_scene service")
        self.scene_srv = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        self.scene_srv.wait_for_service()
        rospy.loginfo("Connected.")

        rospy.loginfo("Connecting to clear octomap service...")
        self.clear_octomap_srv = rospy.ServiceProxy(
          '/clear_octomap', Empty)
        self.clear_octomap_srv.wait_for_service()
        rospy.loginfo("Connected!")

        self.scene_objects = []
        self.attached_objects = []
    
    def clear_octomap(self):
        rospy.loginfo("Clearing octomap")
        self.clear_octomap_srv.call(EmptyRequest())
        rospy.sleep(2.0)

    def add_mesh_object(self, id_name, pose, size = (1, 1, 1), rotation = (0, 0, 0), frame='/world'):
        pass
    
    def add_box_object(self, id_name, pose, size = (1, 1, 1), rotation = (0, 0, 0), frame='/world'):
        '''
        Adds the particular BOX TYPE objects to the moveit planning scene
        
        :param id_name: unique id that object should be labeled with
        :param pose: pose of the object
        :param size: size of the object
        :param rotation: rotation offset in r, p, y
        :param frame: frame in which the object pose is passed
        :type id_name: string
        :type pose: list of double of length 7 (x, y, z, q_x, q_y, q_z, q_w)
        :type size: tuple of length 3
        :type frame: string
        '''
        assert type(size) is tuple, 'size should be tuple'
        assert len(size)==3, 'size should be of length 3'
        assert not id_name in self.scene_objects, \
                                'Object with the same name already exists!'

        self.scene_objects.append(id_name)
        
        object_pose = PoseStamped()
        object_pose.header.frame_id = frame

        if type(pose) is list:
            pose = list_to_pose(pose)

        (r, p, y) = rotation
        q_object_rotation = quaternion_from_euler(math.radians(r), math.radians(p), math.radians(y))
        q_object = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        q_object_final = quaternion_multiply(q_object, q_object_rotation)
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        object_pose.pose = Pose(Point(x, y, z), Quaternion(*q_object_final))

        #Add object description in scene
        self.clear_octomap() #need to sleep before adding part!! https://answers.ros.org/question/209030/moveit-planningsceneinterface-addbox-not-showing-in-rviz/
        self.planning_scene_interface.add_box(id_name, object_pose, size)
        self.wait_for_planning_scene_object(id_name)

    def remove_world_object(self, object_name):
        '''
        Removes a specified object from the MoveIt! planning scene
        :param object_name: unique name for the object
        :type object_name: string
        ''' 
        assert object_name is not None, 'Please pass in an object_name for the object!'
        self.planning_scene_interface.remove_world_object(object_name)
        self.clear_octomap()

    def remove_all_objects(self):
        '''
        Removes all the objects in the current MoveIt! planning scene
        '''
        self.clear_octomap() #need to sleep before or else it gets skipped over...
        self.planning_scene_interface.remove_world_object(None)

    def attach_gripper_object(self, object_name, arm, gripper_planning_group):
        '''
        Attaches an object to the robot gripper
        :param name: name of the object in the planning scene
        :param arm: the Arm object
        :param gripper_planning_group: the gripper planning group in MoveIt!
        
        :type name: string
        :type arm: Arm
        :type gripper_planning_group: string
        '''
        object_name = str(object_name)
        eef_link = arm.get_end_effector_link()
        touch_links = arm.robot.get_link_names(group=gripper_planning_group)
        self.clear_octomap()
        self.planning_scene_interface.attach_box(eef_link, object_name, touch_links=touch_links)

    def detach_gripper_object(self, object_name, arm, remove_from_world=False):
        '''
        Detaches an object earlier attached to the robot gripper
        :param name: name of the object in the planning scene
        :param arm: the Arm object
        :param remove_from_world: if true, object also deleted from world
        
        :type name: string
        :type arm: Arm
        :type remove_from_world: bool
        '''
        
        eef_link = arm.get_end_effector_link()
        self.planning_scene_interface.remove_attached_object(eef_link, object_name)

        if remove_from_world is True:
            self.remove_world_object(object_name)
        
        self.clear_octomap()


    def wait_for_planning_scene_object(self, object_name):
        '''
        Waits for object to appear in planning scene
        :param object_name: name of the object in the planning scene
        
        :type object_name: string
        '''
        rospy.loginfo("Waiting for object '" + object_name + "'' to appear in planning scene...")
        gps_req = GetPlanningSceneRequest()
        gps_req.components.components = gps_req.components.WORLD_OBJECT_NAMES

        part_in_scene = False
        while not rospy.is_shutdown() and not part_in_scene:
            # This call takes a while when rgbd sensor is set
            gps_resp = self.scene_srv.call(gps_req)
            # check if object_name is in the answer
            for collision_obj in gps_resp.scene.world.collision_objects:
                if collision_obj.id == object_name:
                    part_in_scene = True
                    break
                else:
                    rospy.sleep(1.0)

        rospy.loginfo("'" + object_name + "'' is in scene!")
