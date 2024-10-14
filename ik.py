import math

class LegKinematics:
    def __init__(self, torso_length, femur_length, foot_length, offsets):
        self.torso_length = torso_length  # L1
        self.femur_length = femur_length  # L2
        self.foot_length = foot_length    # L3
        self.offsets = offsets  # Offsets for each leg: {FL, FR, BL, BR}

    def calculate_angles(self, x, y, z):
        results = {}
        for leg, (offset_x, offset_y) in self.offsets.items():
            # Apply coordinate offsets for each leg
            x_adj = x - offset_x
            y_adj = y - offset_y

            # Step 1: Planar distance from the hip to the target (y-z plane)
            c = math.sqrt(z**2 + y_adj**2)

            # Step 2: 3D distance from torso to target position
            d = math.sqrt(c**2 + self.torso_length**2)

            # Step 3: Hip angle (side-to-side rotation)
            hip_angle = math.degrees(math.atan2(z, y_adj))

            # Step 4: 3D distance from hip joint to the target foot position
            g = math.sqrt(d**2 + x_adj**2)

            # Step 5: Foot angle using the law of cosines
            cos_foot_angle = (self.femur_length**2 + self.foot_length**2 - g**2) / (2 * self.femur_length * self.foot_length)
            cos_foot_angle = min(1, max(-1, cos_foot_angle))  # Clamp to avoid errors
            foot_angle = math.degrees(math.acos(cos_foot_angle))

            # Step 6: Femur angle using the law of cosines
            cos_femur_angle = (self.femur_length**2 + g**2 - self.foot_length**2) / (2 * self.femur_length * g)
            cos_femur_angle = min(1, max(-1, cos_femur_angle))
            femur_angle = math.degrees(math.acos(cos_femur_angle))

            # Adjust femur angle by adding the angle in the x-z plane
            alpha_1 = math.degrees(math.atan2(x_adj, d))
            femur_angle += alpha_1

            results[leg] = (hip_angle, femur_angle, foot_angle)

        return results

def main():
    # Link lengths (L1, L2, L3)
    torso_length = 30.6
    femur_length = 49.5
    foot_length = 100.2

    # Offsets for each leg relative to the body frame
    offsets = {
        "FL": (6, 4.35),  # Front-left leg
        "FR": (6, -4.35),  # Front-right leg
        "BL": (-6, 4.35),  # Back-left leg
        "BR": (-6, -4.35)  # Back-right leg
    }

    kinematics = LegKinematics(torso_length, femur_length, foot_length, offsets)

    try:
        x = float(input("Enter x: "))
        y = float(input("Enter y: "))
        z = float(input("Enter z: "))
    except ValueError:
        print("Invalid input. Please enter numeric values.")
        return

    try:
        results = kinematics.calculate_angles(x, y, z)
        
        # Print calculated joint angles for all legs
        for leg, angles in results.items():
            print(f"\n{leg} Leg:")
            print(f"  Hip angle: {angles[0]:.2f} degrees")
            print(f"  Femur angle: {angles[1]:.2f} degrees")
            print(f"  Foot angle: {angles[2]:.2f} degrees")

    except ValueError as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
