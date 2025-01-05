import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from geometry_msgs.msg import Point
from inverse_kinematics.inverse_kinematics import LegKinematics

class LegKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')

        # Define limb lengths and parameters (in cm)
        femur_length = 4.0
        foot_length = 8.0
        coxa_length = 3.0
        distance_from_ground = 8.5

        self.kinematics = LegKinematics(femur_length, foot_length, coxa_length, distance_from_ground)

        # ROS 2 subscriber for the target position
        self.subscription = self.create_subscription(
            Point, 'target_position', self.position_callback, 10)

        # Publishers for each servo
        self.servo_publishers = [
            self.create_publisher(Int8, f'/servo{i}', 10) for i in range(12)
        ]

    def position_callback(self, msg):
        roll, pitch, yaw = 0.0, 0.0, 0.0  # Adjust for actual usage

        try:
            # Calculate joint angles using the kinematics
            angles = self.kinematics.calculate_angles(msg.x, msg.y, msg.z, roll, pitch, yaw)

            # Publish each joint angle to its corresponding topic
            for i, angle in enumerate(angles):
                angle_msg = Int8()
                angle_msg.data = int(angle)  # Ensure angle is an integer
                self.servo_publishers[i].publish(angle_msg)

            self.get_logger().info(f'Published angles: {angles}')

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
