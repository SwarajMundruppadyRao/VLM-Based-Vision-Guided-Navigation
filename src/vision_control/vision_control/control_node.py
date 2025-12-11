#!/usr/bin/env python3
"""
Control Node for Vision-Guided Navigation

This node subscribes to navigation commands from the VLM and translates
them into robot velocity commands.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class ControlNode(Node):
    """ROS 2 node for robot motion control."""

    def __init__(self):
        super().__init__('control_node')
        
        # Declare parameters
        self.declare_parameter('command_topic', '/vlm/navigation_command')
        self.declare_parameter('velocity_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('command_timeout', 2.0)
        
        # Get parameters
        command_topic = self.get_parameter('command_topic').value
        velocity_topic = self.get_parameter('velocity_topic').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.command_timeout = self.get_parameter('command_timeout').value
        
        # Initialize state
        self.current_command = None
        self.last_command_time = self.get_clock().now()
        
        # Create subscriber and publisher
        self.command_sub = self.create_subscription(
            String,
            command_topic,
            self.command_callback,
            10
        )
        
        self.vel_pub = self.create_publisher(
            Twist,
            velocity_topic,
            10
        )
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f'Control Node started')
        self.get_logger().info(f'Subscribing to commands: {command_topic}')
        self.get_logger().info(f'Publishing velocities to: {velocity_topic}')
        self.get_logger().info(f'Linear speed: {self.linear_speed} m/s')
        self.get_logger().info(f'Angular speed: {self.angular_speed} rad/s')

    def command_callback(self, msg):
        """Process incoming navigation command."""
        self.current_command = msg.data
        self.last_command_time = self.get_clock().now()
        self.get_logger().info(f'Received command: {self.current_command}')

    def get_velocity_from_command(self, command):
        """
        Convert navigation command to velocity command.
        
        Args:
            command: String command from VLM
            
        Returns:
            Twist message with appropriate velocities
        """
        twist = Twist()
        
        if command == 'STOP':
            # Stop the robot
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        elif command == 'MOVE_FORWARD':
            # Move forward
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            
        elif command == 'MOVE_BACKWARD':
            # Move backward
            twist.linear.x = -self.linear_speed
            twist.angular.z = 0.0
            
        elif command == 'TURN_LEFT':
            # Turn left in place
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
            
        elif command == 'TURN_RIGHT':
            # Turn right in place
            twist.linear.x = 0.0
            twist.angular.z = -self.angular_speed
            
        elif command.startswith('MOVE_TO'):
            # For complex commands, move forward slowly
            twist.linear.x = self.linear_speed * 0.5
            twist.angular.z = 0.0
            
        elif command == 'ANALYZE':
            # Stop and wait for more information
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        else:
            # Unknown command - stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().warn(f'Unknown command: {command}')
        
        return twist

    def control_loop(self):
        """Main control loop that publishes velocity commands."""
        twist = Twist()
        
        # Check if we have a recent command
        time_since_command = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        
        if self.current_command and time_since_command < self.command_timeout:
            # Execute current command
            twist = self.get_velocity_from_command(self.current_command)
        else:
            # No recent command - stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if self.current_command and time_since_command >= self.command_timeout:
                self.get_logger().warn('Command timeout - stopping robot')
                self.current_command = None
        
        # Publish velocity
        self.vel_pub.publish(twist)


def main(args=None):
    """Main entry point for the control node."""
    rclpy.init(args=args)
    
    try:
        control_node = ControlNode()
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
