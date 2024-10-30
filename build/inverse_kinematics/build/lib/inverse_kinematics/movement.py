#!/usr/bin/env python
import math
from inverse_kinematics.inverse_kinematics import LegKinematics  

class Movement:
    def __init__(self, leg_kinematics):
        self.leg_kinematics = leg_kinematics

    def execute_command(self, command):
        if command == "stand":
            # Set all joint angles to zero for each leg
            return {leg: (0.0, 0.0, 0.0) for leg in self.leg_kinematics.offsets}
        elif command == "move_forward":
                pass
        else:
            raise ValueError("Unknown command")