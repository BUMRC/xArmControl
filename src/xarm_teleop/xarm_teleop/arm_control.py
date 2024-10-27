#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from pymoveit2 import MoveIt2
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Joy
import sys
import termios
import tty
import select
from math import pi, sin, cos
import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_from_euler

class ArmTeleopController(Node):
    def __init__(self):
        super().__init__('arm_teleop_controller')
        
        # Create callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],  # Update with your joint names
            base_link_name="base_link",  # Update with your base link name
            end_effector_name="tool0",   # Update with your end effector link name
            group_name="manipulator",     # Update with your planning group name
            callback_group=self.callback_group
        )
        
        # Initialize tf2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Teleop settings
        self.linear_scale = 0.01  # meters per keypress/joystick input
        self.angular_scale = 0.1  # radians per keypress/joystick input
        
        # Subscribe to joystick input
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Create timer for continuous movement updates
        self.update_timer = self.create_timer(0.1, self.update_position)  # 10Hz update rate
        
        # Create timer for keyboard input
        self.keyboard_timer = self.create_timer(0.1, self.get_keyboard_input)  # 10Hz keyboard check
        
        # Initialize movement flags
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.angular_x = 0.0
        self.angular_y = 0.0
        self.angular_z = 0.0
        
        # Store current pose
        self.current_pose = None
        
        # Store terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        
        self.get_logger().info('Arm teleop controller initialized')
        
    def __del__(self):
        # Restore terminal settings
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        except:
            pass

    def joy_callback(self, msg: Joy):
        """Handle joystick input."""
        # Map joystick axes to movement
        self.linear_x = msg.axes[1] * self.linear_scale   # Left stick up/down
        self.linear_y = msg.axes[0] * self.linear_scale   # Left stick left/right
        self.linear_z = msg.axes[4] * self.linear_scale   # Right stick up/down
        self.angular_x = msg.axes[3] * self.angular_scale # Right stick left/right
        self.angular_y = msg.buttons[4] * self.angular_scale  # LB button
        self.angular_z = msg.buttons[5] * self.angular_scale  # RB button

    def get_keyboard_input(self):
        """Get keyboard input for teleop control."""
        try:
            # Check if input is available
            if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                
                # Reset movement values
                self.linear_x = self.linear_y = self.linear_z = 0.0
                self.angular_x = self.angular_y = self.angular_z = 0.0
                
                # Map keyboard input to movement
                if ch == 'w':
                    self.linear_x = self.linear_scale
                elif ch == 's':
                    self.linear_x = -self.linear_scale
                elif ch == 'a':
                    self.linear_y = self.linear_scale
                elif ch == 'd':
                    self.linear_y = -self.linear_scale
                elif ch == 'q':
                    self.linear_z = self.linear_scale
                elif ch == 'e':
                    self.linear_z = -self.linear_scale
                elif ch == 'u':
                    self.angular_x = self.angular_scale
                elif ch == 'j':
                    self.angular_x = -self.angular_scale
                elif ch == 'i':
                    self.angular_y = self.angular_scale
                elif ch == 'k':
                    self.angular_y = -self.angular_scale
                elif ch == 'o':
                    self.angular_z = self.angular_scale
                elif ch == 'l':
                    self.angular_z = -self.angular_scale
                elif ch == 'h':  # Home position
                    self.move_to_home()
                elif ch == '\x03':  # Ctrl+C
                    raise KeyboardInterrupt
                
        except (termios.error, IOError) as e:
            self.get_logger().error(f'Keyboard input error: {str(e)}')

    async def update_position(self):
        """Update end-effector position based on current movement values."""
        try:
            # Get current pose
            current_pose = await self.moveit2.get_current_pose()
            if current_pose is None:
                return
            
            # Create target pose
            target_pose = Pose()
            
            # Update position
            target_pose.position.x = current_pose.position.x + self.linear_x
            target_pose.position.y = current_pose.position.y + self.linear_y
            target_pose.position.z = current_pose.position.z + self.linear_z
            
            # Get current RPY
            current_rpy = await self.moveit2.get_current_rpy()
            if current_rpy is None:
                return
                
            # Update orientation
            target_rpy = [
                current_rpy[0] + self.angular_x,
                current_rpy[1] + self.angular_y,
                current_rpy[2] + self.angular_z
            ]
            
            # Convert RPY to quaternion using ROS tf_transformations
            quat = quaternion_from_euler(target_rpy[0], target_rpy[1], target_rpy[2])
            target_pose.orientation.x = quat[0]
            target_pose.orientation.y = quat[1]
            target_pose.orientation.z = quat[2]
            target_pose.orientation.w = quat[3]
            
            # Move to target pose if there's any movement
            if any([self.linear_x, self.linear_y, self.linear_z,
                   self.angular_x, self.angular_y, self.angular_z]):
                await self.moveit2.move_to_pose(target_pose, wait=False)
                
        except Exception as e:
            self.get_logger().error(f'Position update failed: {str(e)}')

    async def move_to_home(self):
        """Move the robot to its home position."""
        try:
            # Define home joint positions - adjust these values for your robot
            home_positions = [0.0, -pi/2, 0.0, -pi/2, 0.0, 0.0]
            await self.moveit2.move_to_configuration(home_positions)
            self.get_logger().info('Moved to home position')
        except Exception as e:
            self.get_logger().error(f'Failed to move to home position: {str(e)}')

def main():
    rclpy.init()
    
    # Create and spin the node with MultiThreadedExecutor
    arm_controller = ArmTeleopController()
    executor = MultiThreadedExecutor()
    executor.add_node(arm_controller)
    
    try:
        print("\nTeleop Control Active:")
        print("---------------------")
        print("Movement Controls:")
        print("  w/s - X axis")
        print("  a/d - Y axis")
        print("  q/e - Z axis")
        print("Rotation Controls:")
        print("  u/j - Roll")
        print("  i/k - Pitch")
        print("  o/l - Yaw")
        print("Other Controls:")
        print("  h   - Move to home position")
        print("  Ctrl+C to exit")
        print("---------------------")
        
        executor.spin()
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, arm_controller.old_settings)
        arm_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()