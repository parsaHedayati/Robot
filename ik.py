import math

class LegKinematics:
    def __init__(self, torso_length, femur_length, foot_length):
        self.torso_length = torso_length
        self.femur_length = femur_length
        self.foot_length = foot_length

    def calculate_angles(self, x, y, z):
        # Step 1: Calculate planar distance from the hip to the target (in y-z plane)
        c = math.sqrt(z**2 + y**2)

        # Step 2: Distance from torso to target considering hip offset
        d = math.sqrt(c**2 + self.torso_length**2)

        # Step 3: Calculate hip angles
        alpha = math.degrees(math.atan2(z, y))  # Hip rotation in the y-z plane
        beta = math.degrees(math.atan2(c, self.torso_length))  # Hip tilt in x-z plane
        hip_angle = alpha + beta  # Total hip joint angle

        # Step 4: Calculate distance from hip joint to the foot target (3D distance)
        g = math.sqrt(d**2 + x**2)

        # Step 5: Calculate the foot joint angle using the law of cosines
        cos_angle = (g**2 - self.femur_length**2 - self.foot_length**2) / (-2 * self.femur_length * self.foot_length)

        # Clamp the value to avoid math domain errors
        cos_angle = min(1, max(-1, cos_angle))  # Ensures cos_angle is between -1 and 1
        foot_angle = math.degrees(math.acos(cos_angle))

        # Step 6: Calculate femur angle components
        alpha_1 = math.degrees(math.atan2(x, d))  # Angle in x-z plane
        beta_1 = math.degrees(math.asin((self.foot_length * math.sin(math.radians(foot_angle))) / g))

        # Total femur angle
        femur_angle = alpha_1 + beta_1

        return hip_angle, femur_angle, foot_angle

def main():
    # Link lengths (torso to hip, femur, and foot)
    torso_length = 40.1  # Distance between torso and hip joint (L1)
    femur_length = 49.5  # Length of the femur link (L2)
    foot_length = 100.2  # Length of the foot link (L3)

    kinematics = LegKinematics(torso_length, femur_length, foot_length)

    # Inputs: Target position for the foot in 3D space (x, y, z)
    try:
        x = float(input("Enter x: "))
        y = float(input("Enter y: "))
        z = float(input("Enter z: "))
    except ValueError:
        print("Invalid input. Please enter numeric values.")
        return

    hip_angle, femur_angle, foot_angle = kinematics.calculate_angles(x, y, z)

    # Print calculated joint angles
    print(f"Hip angle: {hip_angle:.2f} degrees")
    print(f"Femur angle: {femur_angle:.2f} degrees")
    print(f"Foot angle: {foot_angle:.2f} degrees")

if __name__ == "__main__":
    main()
