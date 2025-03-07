import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from .gen3lite_pymoveit2 import Gen3LiteArm

def main():
    rclpy.init()
    arm = Gen3LiteArm()

    # **Step 1: Move to Vertical Position First**
    print("Moving to Vertical Position...")
    arm.go_vertical()

    # Get current pose after resetting to home
    current_pose = arm.get_end_effector_pose()
    print(f"Current Pose after Home: {current_pose.position}")

    # **Step 2: Define a Slight Movement**
    new_pose = Pose()
    new_pose.position = Point(
        x=0.35,
        y=0.1,
        z=0.65
    )
    new_pose.orientation = current_pose.orientation

    print(f"Moving to: {new_pose.position}")
    arm.inverse_kinematic_movement(new_pose)

    # **Step 3: Move Back to Home Position**
    print("Returning to Home Position...")
    # arm.go_home()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
