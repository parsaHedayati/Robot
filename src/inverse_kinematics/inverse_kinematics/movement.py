#!/usr/bin/env python
import math
from inverse_kinematics.inverse_kinematics import LegKinematics  

class Movement:
    def __init__(self, leg_kinematics):
        self.leg_kinematics = leg_kinematics

    def execute_command(self, command):
        if command == "stand":
            stand_angles = {
                "FL": (90.0, 90.0, 90.0),  
                "FR": (90.0, 90.0, 90.0),  
                "BL": (90.0, 90.0, 90.0),  
                "BR": (90.0, 90.0, 90.0),  
            }
            return stand_angles
        elif command == "move_forward":
            # Define the step height and stride length
            step_h = 2.0
            stride_length = 4.0

            # Define the positions for the crawl gait sequence
            gait_sequence = [
                {"FL": (stride_length, 0, step_h), "BR": (0, 0, 0), "FR": (0, 0, 0), "BL": (0, 0, 0)}, 
                {"FL": (stride_length, 0, 0), "BR": (0, 0, 0), "FR": (0, 0, 0), "BL": (0, 0, 0)},         
                {"FL": (0, 0, 0), "BR": (stride_length, 0, step_h), "FR": (0, 0, 0), "BL": (0, 0, 0)}, 
                {"FL": (0, 0, 0), "BR": (stride_length, 0, 0), "FR": (0, 0, 0), "BL": (0, 0, 0)},         
                {"FL": (0, 0, 0), "BR": (0, 0, 0), "FR": (stride_length, 0, step_h), "BL": (0, 0, 0)}, 
                {"FL": (0, 0, 0), "BR": (0, 0, 0), "FR": (stride_length, 0, 0), "BL": (0, 0, 0)},         
                {"FL": (0, 0, 0), "BR": (0, 0, 0), "FR": (0, 0, 0), "BL": (stride_length, 0, step_h)}, 
                {"FL": (0, 0, 0), "BR": (0, 0, 0), "FR": (0, 0, 0), "BL": (stride_length, 0, 0)},         
            ]

            # Calculate the joint angles for each step in the gait sequence
            angles_sequence = []
            for step in gait_sequence:
                angles = {}
                for leg, pos in step.items():
                    x, y, z = pos
                    roll, pitch, yaw = 0.0, 0.0, 0.0  # Adjust for actual usage
                    angles[leg] = self.leg_kinematics.calculate_angles(x, y, z, roll, pitch, yaw)
                angles_sequence.append(angles)

            return angles_sequence
        else:
            raise ValueError(f"Unknown command: {command}")

# Usage example
if __name__ == "__main__":
    femur_length = 4.0
    foot_length = 8.0
    coxa_length = 3.0
    distance_from_ground = 8.5
    leg_kinematics = LegKinematics(femur_length, foot_length, coxa_length, distance_from_ground)
    
    movement = Movement(leg_kinematics)
    forward_gait = movement.execute_command("move_forward")
    for step in forward_gait:
        print(step)
