import math
import numpy as np

class LegKinematics:
    def __init__(self, femur_length, foot_length, coxa_length, distance_from_ground, offsets=None):
        self.femur_length = femur_length  # L2
        self.foot_length = foot_length    # L3
        self.coxa_length = coxa_length    # Offset of the hip joint
        self.distance_from_ground = distance_from_ground

    def rotation_matrix(self, roll, pitch, yaw):
        Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        return np.dot(Rz, np.dot(Ry, Rx))

    def calculate_angles(self, x, y, z, roll, pitch, yaw):
        # Transform coordinates based on roll, pitch, yaw
        R = self.rotation_matrix(roll, pitch, yaw)
        leg_pos = np.dot(R, np.array([x, y, z]))
        x_adj, y_adj, z_adj = leg_pos

        D = math.sqrt(x_adj**2 + z_adj**2)
        theta1 = math.degrees(math.atan2(x_adj, z_adj))
        d = D - self.coxa_length
        y_offset = self.distance_from_ground - y_adj
        R_total = math.sqrt(D**2 + y_offset**2)

        # Singularity checks
        if R_total > (self.femur_length + self.foot_length):
            raise ValueError("Singularity detected: Target position is out of reach.")
        if R_total < abs(self.femur_length - self.foot_length):
            raise ValueError("Singularity detected: Target position is too close to the base.")

        # Avoid invalid domain for math.acos
        if not (-1 <= y_offset / R_total <= 1):
            raise ValueError("Singularity detected: Invalid arc cosine value for y_offset / R_total.")
        if not (-1 <= (self.femur_length**2 + R_total**2 - self.foot_length**2) / (2 * self.femur_length * R_total) <= 1):
            raise ValueError("Singularity detected: Invalid arc cosine value for femur angle.")
        if not (-1 <= (self.femur_length**2 + self.foot_length**2 - R_total**2) / (2 * self.femur_length * self.foot_length) <= 1):
            raise ValueError("Singularity detected: Invalid arc cosine value for foot angle.")

        # Calculate angles
        alpha1 = math.degrees(math.acos(y_offset / R_total))
        alpha2 = math.degrees(math.acos((self.femur_length**2 + R_total**2 - self.foot_length**2) / (2 * self.femur_length * R_total)))
        theta2 = alpha1 + alpha2
        theta3 = math.degrees(math.acos((self.femur_length**2 + self.foot_length**2 - R_total**2) / (2 * self.femur_length * self.foot_length)))

        return [theta1, theta2, theta3]
