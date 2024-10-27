#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit2 import MoveGroupInterface
from moveit2.robot_state import RobotState
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from sensor_msgs.msg import Joy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import sys
import termios
import tty
import threading
import numpy as np

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # Create callback group for move group interface
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize tf2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create move group interface
        self.move_group = MoveGroupInterface(
            node=self,
            name="xarm_controller",  # todo: might need to replace with actual move group name
            callback_group=self.callback_group
        )
        
        # Wait for required services
        self.move_group.wait_for_servers()
        
        # Teleop settings
        self.linear_scale = 0.1  # meters per keypress/joystick input
        self.angular_scale = 0.1  # radians per keypress/joystick input
        self.current_pose = None
        
        # Subscribe to joystick input
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Create timer for continuous movement updates
        self.update_timer = self.create_timer(0.1, self.update_position)  # 10Hz update rate
        
        # Initialize movement flags
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.angular_x = 0.0
        self.angular_y = 0.0
        self.angular_z = 0.0
        
        self.get_logger().info('Arm controller initialized')

    def move_to_pose(self, target_pose: Pose):
        """Move the end effector to the target pose."""
        try:
            # Set the target pose
            self.move_group.set_pose_target(target_pose)
            
            # Create a plan
            plan_result = self.move_group.plan()
            
            if plan_result.planning_time > 0.0:
                self.get_logger().info('Plan succeeded. Executing...')
                # Execute the plan
                success = self.move_group.execute(plan_result.planning_time, wait=True) #todo: might set to false to reduce latency
                return success
            else:
                self.get_logger().error('Planning failed')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Movement failed: {str(e)}')
            return False
        finally:
            # Clear targets
            self.move_group.clear_pose_targets()

    def move_to_joint_positions(self, joint_positions):
        """Move to specified joint positions."""
        try:
            # Set joint value target
            self.move_group.set_joint_value_target(joint_positions)
            
            # Plan and execute
            success = self.move_group.plan_and_execute()
            return success
            
        except Exception as e:
            self.get_logger().error(f'Joint movement failed: {str(e)}')
            return False
        finally:
            self.move_group.clear_pose_targets()

    def move_to_named_position(self, position_name: str):
        """Move to a named position (e.g., 'home', 'ready')."""
        try:
            self.move_group.set_named_target(position_name)
            success = self.move_group.plan_and_execute()
            return success
            
        except Exception as e:
            self.get_logger().error(f'Named position movement failed: {str(e)}')
            return False

    def joy_callback(self, msg: Joy):
        """Handle joystick input."""
        # Map joystick axes to movement
        # Adjust these mappings based on your joystick
        self.linear_x = msg.axes[1] * self.linear_scale  # Left stick up/down
        self.linear_y = msg.axes[0] * self.linear_scale  # Left stick left/right
        self.linear_z = msg.axes[4] * self.linear_scale  # Right stick up/down
        self.angular_x = msg.axes[3] * self.angular_scale  # Right stick left/right
        self.angular_y = msg.buttons[4] * self.angular_scale  # LB button
        self.angular_z = msg.buttons[5] * self.angular_scale  # RB button

    def get_keyboard_input(self):
        """Get keyboard input for teleop control."""
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            # Set terminal to raw mode
            tty.setraw(sys.stdin.fileno())
            # Get character
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
            
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def update_position(self):
        """Update end-effector position based on current movement values."""
        try:
            # Get current end-effector pose
            current_pose = self.move_group.get_current_pose()
            
            # Create new target pose
            target_pose = Pose()
            target_pose.position.x = current_pose.position.x + self.linear_x
            target_pose.position.y = current_pose.position.y + self.linear_y
            target_pose.position.z = current_pose.position.z + self.linear_z
            
            # Update orientation using euler angles
            # Note: This is a simplified approach. For production, consider using proper quaternion operations
            current_rpy = self.move_group.get_current_rpy()
            target_rpy = [
                current_rpy[0] + self.angular_x,
                current_rpy[1] + self.angular_y,
                current_rpy[2] + self.angular_z
            ]
            
            # Convert back to quaternion
            self.move_group.set_rpy_target(target_rpy)
            
            # Move to new pose
            if any([self.linear_x, self.linear_y, self.linear_z, 
                   self.angular_x, self.angular_y, self.angular_z]):
                self.move_group.go(wait=False)
                
        except Exception as e:
            self.get_logger().error(f'Position update failed: {str(e)}')


def getKeyboardInput(arm_controller: ArmController):
    while rclpy.ok():
        arm_controller.get_keyboard_input()
        


def main():
    rclpy.init()
    
    # Create and spin the node with MultiThreadedExecutor
    arm_controller = ArmController()
    executor = MultiThreadedExecutor()
    executor.add_node(arm_controller)
    
    # Create thread for keyboard input
    keyboard_thread = threading.Thread(target=lambda ac: getKeyboardInput(ac))
    keyboard_thread.daemon = True
    keyboard_thread.start()
    
    try:
        print("Teleop Control Active:")
        print("w/s - X axis")
        print("a/d - Y axis")
        print("q/e - Z axis")
        print("u/j - Roll")
        print("i/k - Pitch")
        print("o/l - Yaw")
        print("Ctrl+C to exit")
        
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    finally:
        arm_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()