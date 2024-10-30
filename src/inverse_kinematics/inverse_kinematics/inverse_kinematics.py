#!/usr/bin/env python
import math
import numpy as np

class LegKinematics:
    def __init__(self, torso_length, femur_length, foot_length, offsets):
        self.torso_length = torso_length  # L1
        self.femur_length = femur_length  # L2
        self.foot_length = foot_length    # L3
        self.offsets = offsets  # Offsets for each leg: {FL, FR, BL, BR}

    def rotation_matrix(self, roll, pitch, yaw):
        Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        return np.dot(Rz, np.dot(Ry, Rx))

    def calculate_angles(self, x, y, z, roll, pitch, yaw):
        results = {}
        R = self.rotation_matrix(roll, pitch, yaw)

        for leg, (offset_x, offset_y) in self.offsets.items():
            rotated_offset = np.dot(R, np.array([offset_x, offset_y, 0]))
            leg_pos = np.dot(R, np.array([x, y, z])) - rotated_offset
            x_adj, y_adj, z_adj = leg_pos

            c = math.sqrt(z_adj**2 + y_adj**2)
            d = math.sqrt(c**2 + self.torso_length**2)
            hip_angle = math.degrees(math.atan2(z_adj, y_adj))
            g = math.sqrt(d**2 + x_adj**2)

            cos_foot_angle = (self.femur_length**2 + self.foot_length**2 - g**2) / (2 * self.femur_length * self.foot_length)
            cos_foot_angle = min(1, max(-1, cos_foot_angle))
            foot_angle = math.degrees(math.acos(cos_foot_angle))

            cos_femur_angle = (self.femur_length**2 + g**2 - self.foot_length**2) / (2 * self.femur_length * g)
            cos_femur_angle = min(1, max(-1, cos_femur_angle))
            femur_angle = math.degrees(math.acos(cos_femur_angle))

            alpha_1 = math.degrees(math.atan2(x_adj, d))
            femur_angle += alpha_1

            results[leg] = (hip_angle, femur_angle, foot_angle)

        return results

