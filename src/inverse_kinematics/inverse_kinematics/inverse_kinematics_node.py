#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from inverse_kinematics.inverse_kinematics import LegKinematics  # Import your LegKinematics class
from inverse_kinematics.movement import Movement  # Import your Movement class

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')

        # Initialize kinematics values (in cm)
        torso_length = 30.6
        femur_length = 49.5
        foot_length = 100.2
        offsets = {
            "FL": (6, 4.35),
            "FR": (6, -4.35),
            "BL": (-6, 4.35),
            "BR": (-6, -4.35)
        }
        
        # Initialize LegKinematics and Movement classes
        leg_kinematics = LegKinematics(torso_length, femur_length, foot_length, offsets)
        self.movement = Movement(leg_kinematics)

        # Create subscriber to receive commands
        self.subscription = self.create_subscription(
            String,
            'command',
            self.command_callback,
            10
        )

        # Create publisher for joint angles
        self.publisher = self.create_publisher(Float64MultiArray, 'joint_angles', 10)
        self.get_logger().info("MovementNode started, waiting for commands...")

    def command_callback(self, msg):
        command = msg.data

        try:
            # Calculate joint angles based on command
            results = self.movement.execute_command(command)

            # Create Float64MultiArray message to publish the angles
            angles_msg = Float64MultiArray()
            for angles in results.values():
                angles_msg.data.extend(angles)  # Add (hip, femur, foot) angles for each leg

            # Publish the joint angles
            self.publisher.publish(angles_msg)
            self.get_logger().info(f'Published angles for command "{command}": {angles_msg.data}')

        except ValueError as e:
            self.get_logger().error(f'Error in movement execution: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
