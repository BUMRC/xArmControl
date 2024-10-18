# roverArmSim

This is a demo to simulate our robot arm in ROS. 

### Installation/setup
First, install ros2-humble by following the guide at: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Additionally, install the following packages:
```
sudo apt install ros-dev-tools '~nros-humble-rqt*' ignition-fortress ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-controller-manager ros-humble-joint-trajectory-controller ros-humble-joint-state-broadcaster ros-humble-joint-state-publisher-gui ros-humble-ros-gz ros-humble-ign-ros2-control ros-humble-moveit ros-humble-moveit-visual-tools
```

Build the workspace:
```
colcon build --symlink-install
```

### Running the simulation
```
cd /src/launch
ros2 launch roverArmSim gazebo.launch.py
```


test cmd:
```
"xarm_6_joint", "xarm_5_joint", "xarm_4_joint", "xarm_3_joint", "xarm_2_joint"
ros2 topic pub --once /xarm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['xarm_6_joint', 'xarm_5_joint', 'xarm_4_joint', 'xarm_3_joint', 'xarm_2_joint'], points: [{positions: [0.0, 1.0, 1.0, 1.0, 1.0], time_from_start: {sec: 0, nanosec: 0}}]}"

```
