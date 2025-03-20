#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from geometry_msgs.msg import Pose, Point, Quaternion
from pyquaternion import Quaternion as PyQuaternion
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper  
import numpy as np


def pick_and_place(arm, gripper, pre_pick_pose, pick_pose, pre_put_pose1, pre_put_pose2, put_pose1, put_pose2, put_pose3):
    """Executes the pick and place sequence with the Kinova arm."""
    arm.inverse_kinematic_movement(pre_pick_pose)
    time.sleep(0.5)
    gripper.move_to_position(0.0)
    
    arm.inverse_kinematic_movement(pick_pose)
    gripper.move_to_position(0.8)
    print("Got the cube")
    time.sleep(0.5)
    
    arm.inverse_kinematic_movement(pre_put_pose1)
    time.sleep(0.5)
    arm.inverse_kinematic_movement(pre_put_pose2)
    time.sleep(0.5)
    arm.inverse_kinematic_movement(put_pose1)
    time.sleep(0.5)
    arm.inverse_kinematic_movement(put_pose2)
    time.sleep(0.5)

    gripper.move_to_position(0.0)
    print("Finished placing!")


def set_pose(position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w):
    """Creates and returns a pose object with given coordinates and orientation."""
    return_pose = Pose()
    return_pose.position.x = position_x
    return_pose.position.y = position_y
    return_pose.position.z = position_z
    return_pose.orientation.x = orientation_x
    return_pose.orientation.y = orientation_y
    return_pose.orientation.z = orientation_z
    return_pose.orientation.w = orientation_w
    return return_pose


class TurtleBotController(Node):
    """Controls the TurtleBot's movement."""
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def move_sequence(self, movements):
        """Executes a sequence of TurtleBot movements."""
        for move in movements:
            twist = Twist()
            twist.linear.x = move["linear_vel"]
            twist.angular.z = move["angular_vel"]

            print(f"Moving: linear={move['linear_vel']} m/s, angular={move['angular_vel']} rad/s for {move['duration']} sec")

            start_time = time.time()
            while (time.time() - start_time) < move["duration"]:
                self.cmd_vel_publisher.publish(twist)
                time.sleep(0.1)

            # Stop after each step
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.5)  # Pause between movements

        print("\nSequence complete!")


def main():
    rclpy.init()

    # Define TurtleBot movement sequence
    movements_before = [
        {"linear_vel": 0.0, "angular_vel": -0.5, "duration": 2.0},
        {"linear_vel": 0.2, "angular_vel": 0.0, "duration": 5.5},
        {"linear_vel": 0.0, "angular_vel": 0.5, "duration": 2.7},
        {"linear_vel": 0.2, "angular_vel": 0.0, "duration": 8.0},
        {"linear_vel": 0.0, "angular_vel": -0.5, "duration": 2.3},
        {"linear_vel": 0.2, "angular_vel": 0.0, "duration": 6.0},
        {"linear_vel": 0.0, "angular_vel": 0.5, "duration": 0.8},
    ]

    # Generate reverse sequence
    movements_reverse = [
        {"linear_vel": -move["linear_vel"], "angular_vel": -move["angular_vel"], "duration": move["duration"]}
        for move in reversed(movements_before)
    ]

    # Move the TurtleBot forward
    turtlebot = TurtleBotController()
    turtlebot.move_sequence(movements_before)

    # Initialize Kinova arm and gripper
    gripper = Gen3LiteGripper()
    arm = Gen3LiteArm()
    arm.go_vertical()

    # Define Kinova arm poses
    pre_pick_pose = set_pose(0.370, -0.092, 0.371, 0.817, 0.574, -0.036, 0.028)
    pick_pose = set_pose(0.370, -0.035, 0.130, 0.758, 0.652, 0.008, 0.017)
    pre_put_pose1 = set_pose(-0.293, 0.081, 0.228, 0.752, 0.658, 0.023, -0.027)
    pre_put_pose2 = set_pose(-0.174, -0.002, 0.191, 0.494, 0.867, -0.007, -0.066)
    put_pose1 = set_pose(-0.195, 0.034, -0.164, 0.494, 0.867, -0.008, -0.066)
    put_pose2 = set_pose(-0.173, 0.0, -0.342, 0.542, 0.838, -0.04, -0.042)
    put_pose3 = set_pose(-0.159, 0.085, -0.336, -0.018, 0.997, 0.026, -0.076)

    # Execute pick and place operations
    pick_and_place(arm, gripper, pre_pick_pose, pick_pose, pre_put_pose1, pre_put_pose2, put_pose1, put_pose2, put_pose3)

    # Move the TurtleBot back to the original location
    print("\nReturning TurtleBot to the original location...")
    turtlebot.move_sequence(movements_reverse)

    # Clean up and shut down
    turtlebot.destroy_node()
    rclpy.shutdown()

    gripper.shutdown()
    arm.shutdown()


if __name__ == '__main__':
    main()

