import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point
from inverse_kinematics.inverse_kinematics import LegKinematics
from inverse_kinematics.movement import Movement

class LegKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')

        # Define limb lengths and parameters (in cm)
        femur_length = 4.0
        foot_length = 8.0
        coxa_length = 3.0
        distance_from_ground = 8.5

        self.kinematics = LegKinematics(femur_length, foot_length, coxa_length, distance_from_ground)
        self.movement = Movement(self.kinematics)

        # ROS 2 subscriber for the command topic
        self.command_subscription = self.create_subscription(
            String, 'command', self.command_callback, 10)

        # Publishers for each joint angle
        self.angle_publishers = [
            self.create_publisher(Float32, f'/leg_{i}_angle_{j}', 10) for i in range(4) for j in range(3)
        ]

    def command_callback(self, msg):
        try:
            command = msg.data

            if command == "stand":
                angles_dict = self.movement.execute_command("stand")
            elif command == "move_forward":
                angles_sequence = self.movement.execute_command("move_forward")
                for angles_dict in angles_sequence:
                    self.publish_angles(angles_dict)
                    self.get_logger().info(f'Published angles for step: {angles_dict}')
                return

            self.publish_angles(angles_dict)

        except ValueError as e:
            self.get_logger().error(f'Error in kinematics calculation: {e}')

    def publish_angles(self, angles_dict):
        for leg, angles in angles_dict.items():
            for j, angle in enumerate(angles):
                angle_msg = Float32()
                angle_msg.data = float(angle)
                leg_index = ["FL", "FR", "BL", "BR"].index(leg)
                self.angle_publishers[leg_index * 3 + j].publish(angle_msg)

        self.get_logger().info(f'Published angles: {angles_dict}')

def main(args=None):
    rclpy.init(args=args)
    node = LegKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
