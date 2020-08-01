#!/usr/bin/env python
import rospy
from hrca_action.utilities import *
from panda_toolbox_task.srv import *

#from collections import OrderedDict

from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point

def context_data_request_client(object_id, query_point, tracking_system):
    try:
        context_data_response = perception_srv(int(object_id), str(query_point), str(tracking_system))
        return context_data_response.object_pose
    except rospy.ServiceException, e:
        print ("Service call failed: %s"%e)


wall_height = 3
wall_width = 3
wall_depth = 0.1
wall_offset_z = 0.45

wall_pose = PoseStamped()
wall_pose.header.frame_id = "panda_link0"

wall_rotated = quaternion_from_euler(math.radians(0), math.radians(0), math.radians(90))
wall_pose.pose = Pose(Point(0, -wall_offset_z, wall_height/2), Quaternion(*wall_rotated))
wall_name = "wall"

bottom_part_size = (0.3, 0.2, 0.01)
side1_part_size = (0.282, 0.101, 0.01)
side2_part_size = (0.282, 0.101, 0.01)
front_part_size = (0.214, 0.201, 0.01)
back_part_size = (0.214, 0.201, 0.01)
#part_sizes_dict = {'bottom_part': (0.3, 0.2, 0.01), 'side1_part': (0.3, 0.2, 0.01), 'side2_part': (0.3, 0.2, 0.01), 'front_part': (0.3, 0.2, 0.01), 'back_part': (0.3, 0.2, 0.01)}
#part_sizes_od = OrderedDict(part_sizes_dict)

part_sizes = list()
part_sizes.append(bottom_part_size) #part_1
part_sizes.append(side1_part_size) #part_2
part_sizes.append(side2_part_size) #part_3
part_sizes.append(back_part_size) #part_4
part_sizes.append(front_part_size) #part_5

if __name__ == '__main__':
    rospy.init_node("moveit_collision_objects")

    rospy.loginfo("Connecting to Perception server")
    perception_srv = rospy.ServiceProxy('tbt_perception_data_request_server', ContextDataRequest)
    perception_srv.wait_for_service()
    rospy.loginfo("Connected!")

    moh = MoveitObjectHandler()
    moh.remove_all_objects()
    moh.add_box_object(wall_name, wall_pose.pose, size=(wall_depth, wall_width, wall_height), frame="panda_link0")
    for i in range(1, 6):
        marker_pose = context_data_request_client(i, "marker_point", "Aruco")
        rotation = (0, 0, 0)
        moh.add_box_object("part_" + str(i), marker_pose, size=part_sizes[i-1], rotation=rotation, frame="panda_link0")

    #rospy.spin()

