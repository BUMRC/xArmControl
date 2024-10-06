from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('xarm_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'XArm.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # Declare the render engine as a launch argument
    render_engine_arg = DeclareLaunchArgument(
        'render_engine',
        default_value='ogre',
        description='Specify the render engine: ogre or ogre2'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Use Ignition Gazebo with specified render engine
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'ign_args': ['-r empty.sdf --render-engine ', LaunchConfiguration('render_engine')]
        }.items()
    )

    # Spawn the robot in Ignition Gazebo
    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-name', 'XArm',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # Load controllers
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )
    
        # Add ign_ros2_control node
    ign_ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_urdf},
                    os.path.join(share_dir, 'config', 'my_controllers.yaml')],
        output='screen',
    )

    return LaunchDescription([
        render_engine_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        ign_gazebo,
        spawn_entity,
        ign_ros2_control_node,
        TimerAction(
            period=4.0,
            actions=[
                load_joint_state_broadcaster,
                load_arm_controller
            ]
        )
    ])
