#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Pose, Point, Quaternion
from .gen3lite_pymoveit2 import Gen3LiteArm
from .gen3lite_pymoveit2 import Gen3LiteGripper

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        self.gripper = Gen3LiteGripper()

        # Declare ROS2 parameters for gripper control
        self.declare_parameter("command", "open")  # Default: Open
        self.declare_parameter("position", 0.5)  # Default: Midway (between open and close)

    def control_gripper(self):
        """Controls the gripper based on input parameters."""
        command = self.get_parameter("command").value
        position = self.get_parameter("position").value

        if command == "open":
            self.get_logger().info("🛠 Opening gripper...")
            self.gripper.open()
        elif command == "close":
            self.get_logger().info("🤖 Closing gripper...")
            self.gripper.close()
        elif command == "move":
            if 0.0 <= position <= 0.85:
                self.get_logger().info(f"🔄 Moving gripper to position: {position}")
                self.gripper.move_to_position(position)
            else:
                self.get_logger().error(f"❌ Invalid position: {position}. Must be between 0.0 and 0.85.")
        else:
            self.get_logger().error(f"❌ Unknown command: {command}. Use 'open', 'close', or 'move'.")

        self.get_logger().info("✅ Gripper operation completed.")

class KinovaPickAndPlace(Node):
    """Handles Kinova robotic arm movements for pick-and-place operations."""

    def __init__(self):
        super().__init__('kinova_controller_bonus')
        self.arm = Gen3LiteArm()
        self.gripper = GripperController()

    def move_to_pose(self, x, y, z, qx, qy, qz, qw, description=""):
        """Moves the arm to a specific pose and verifies movement success."""
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        self.get_logger().info(f"🔄 Moving to {description}: x={x}, y={y}, z={z}")
        self.arm.inverse_kinematic_movement(pose)
        time.sleep(1)

    def control_gripper(self, command):
        """Controls the gripper (open/close)."""
        if command == "open":
            self.get_logger().info("🛠 Opening gripper...")
            self.gripper.open()
        elif command == "close":
            self.get_logger().info("🤖 Closing gripper...")
            self.gripper.close()
        else:
            self.get_logger().error(f"❌ Invalid gripper command: {command}")

    def execute_pick_and_place(self):
        """Executes the full pick-and-place sequence."""

        # Define the movement sequence
        movements = [
            {"x": 0.380, "y": -0.012, "z": 0.3, "qx": 1.000, "qy": 0.002, "qz": 0.011, "qw": -0.012, "desc": "Pre-Pick Position"},
            {"x": 0.380, "y": -0.014, "z": 0.124, "qx": 1.000, "qy": 0.002, "qz": 0.011, "qw": -0.012, "desc": "Pick Position"},
            {"gripper": "open"},
            {"x": 0.380, "y": -0.012, "z": 0.23, "qx": 1.000, "qy": 0.002, "qz": 0.011, "qw": -0.012, "desc": "Lift Position"},
            {"x": -0.314, "y": 0.047, "z": 0.163, "qx": -0.542, "qy": 0.832, "qz": 0.071, "qw": -0.095, "desc": "Pre-Put Position"},
            {"x": -0.314, "y": 0.041, "z": -0.036, "qx": -0.548, "qy": 0.835, "qz": -0.006, "qw": -0.044, "desc": "Lower to Put"},
            {"x": -0.267, "y": -0.007, "z": -0.232, "qx": -0.552, "qy": 0.829, "qz": -0.084, "qw": 0.007, "desc": "Put Position 1"},
            {"x": -0.257, "y": -0.019, "z": -0.315, "qx": -0.553, "qy": 0.829, "qz": -0.085, "qw": 0.008, "desc": "Put Position 2"},
            {"gripper": "close"},
        ]

        # Execute the sequence
        for action in movements:
            if "gripper" in action:
                self.control_gripper(action["gripper"])
            else:
                self.move_to_pose(**action)

        self.get_logger().info("✅ Pick-and-place sequence completed!")


def main():
    """Runs the Kinova pick-and-place controller."""
    rclpy.init()
    kinova_controller = KinovaPickAndPlace()

    try:
        kinova_controller.execute_pick_and_place()
    except Exception as e:
        kinova_controller.get_logger().error(f"❌ Error during execution: {e}")
    finally:
        kinova_controller.get_logger().info("Shutting down node.")
        kinova_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
