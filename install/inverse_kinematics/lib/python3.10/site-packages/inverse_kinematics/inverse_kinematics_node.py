#!/usr/bin/env python
# leg_kinematics_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from inverse_kinematics.inverse_kinematics import LegKinematics

class LegKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')

        #values in CM
        torso_length = 30.6 
        femur_length = 49.5
        foot_length = 100.2
        offsets = {
            "FL": (6, 4.35),
            "FR": (6, -4.35),
            "BL": (-6, 4.35),
            "BR": (-6, -4.35)
        }
        self.kinematics = LegKinematics(torso_length, femur_length, foot_length, offsets)

        
        self.subscription = self.create_subscription(
            Point, 'target_position', self.position_callback, 10)

        
        self.publisher = self.create_publisher(Float64MultiArray, 'joint_angles', 10)

    def position_callback(self, msg):
        
        roll, pitch, yaw = 0.0, 0.0, 0.0

        try:
            results = self.kinematics.calculate_angles(msg.x, msg.y, msg.z, roll, pitch, yaw)

            
            angles_msg = Float64MultiArray()
            for angles in results.values():
                angles_msg.data.extend(angles)  

            # Publish the joint angles
            self.publisher.publish(angles_msg)
            self.get_logger().info(f'Published angles: {angles_msg.data}')

        except ValueError as e:
            self.get_logger().error(f'Error in kinematics calculation: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LegKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

