# hri_framework
Human Robot Interaction Framework

TODO:
- Add correct aruco_ros, librealsense, OpenPose, realsense_gazebo_plugin, realsense_ros, ros_openpose as submodules
- remirror panda_simulator from Gitlab
- Add instructions for Docker and simulating example Toolbox Task
- Rename `Arm` class to `Robot`

## Getting Started:

### Setup:
- You'll need a ROS workspace to pull this repo into. e.g., [panda_docker](https://github.com/hri-group/panda_docker.git)
- `git clone --recursive https://github.com/hri-group/hri_framework.git` into your catkin workspace `src` folder
- `catkin build` or `catkin_make` from the root of your catkin workspace to build all the packages
- `source devel/setup.bash`

### Instantiating a new Robot Action Module:
- Extend the `Arm` class to a new robot and implement any of the abstract methods. See [panda_arm.py](https://github.com/hri-group/hri_framework/blob/master/hrca_action/src/hrca_action/panda_arm.py) for an example. If your robot is MoveIt-enabled the main ones to implement will be any gripper functionality.
- ...

### Random Commands
- `rosrun hrca_action panda_arm_server.py`
