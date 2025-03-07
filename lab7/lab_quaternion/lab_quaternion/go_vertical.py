import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from .gen3lite_pymoveit2 import Gen3LiteArm

def main():
    rclpy.init()
    arm = Gen3LiteArm()

    print("Moving to Vertical Position...")
    arm.go_vertical()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
