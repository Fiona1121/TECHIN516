#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Pose, Twist
from .gen3lite_pymoveit2 import Gen3LiteArm
from .go_to_position import MoveToPosition  # Import MoveToPosition class
from .control_gripper import GripperController


class TurtleBot3Controller(Node):
    """Handles linear and angular movement with speed control and acceleration."""
    
    MAX_SPEED = 0.24  # Set max allowed speed (TurtleBot3 firmware limit)
    ACCELERATION_STEP = 0.02  # Gradual increase per step

    def __init__(self):
        super().__init__('turtlebot3_controller')

        # Declare parameters
        self.declare_parameter('linear_vel', 0.2)  # Default 0.2 m/s
        self.declare_parameter('angular_vel', 0.0)  # Default 0 rad/s
        self.declare_parameter('duration', 5.0)  # Default 5 seconds

        # Create publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Ensure node has initialized before sending commands
        time.sleep(1)

    def move(self):
        """Executes movement with acceleration control."""
        
        # Fetch parameters
        target_linear_velocity = self.get_parameter('linear_vel').value
        angular_velocity = self.get_parameter('angular_vel').value
        duration = self.get_parameter('duration').value

        # Enforce speed limit
        if target_linear_velocity > self.MAX_SPEED:
            self.get_logger().warn(f"⚠️ Speed {target_linear_velocity} m/s exceeds max limit! Setting to {self.MAX_SPEED} m/s.")
            target_linear_velocity = self.MAX_SPEED

        # Validate duration
        if duration <= 0:
            self.get_logger().warn("⚠️ Duration must be greater than 0! Setting to 1 second.")
            duration = 1.0

        # Log movement settings
        self.get_logger().info(f"🚀 Moving: Linear={target_linear_velocity} m/s, Angular={angular_velocity} rad/s, Duration={duration}s")

        msg = Twist()
        current_speed = 0.0  # Start from 0 speed
        start_time = time.time()

        # **Accelerate Gradually** to Target Speed
        while time.time() - start_time < duration:
            # Increase or decrease speed gradually
            if current_speed < target_linear_velocity:
                current_speed += self.ACCELERATION_STEP
                if current_speed > target_linear_velocity:
                    current_speed = target_linear_velocity
            elif current_speed > target_linear_velocity:
                current_speed -= self.ACCELERATION_STEP
                if current_speed < target_linear_velocity:
                    current_speed = target_linear_velocity

            msg.linear.x = current_speed
            msg.angular.z = angular_velocity
            self.publisher_.publish(msg)
            self.get_logger().info(f"🔄 Accelerating: Speed={current_speed:.2f} m/s")

            time.sleep(0.1)  # Ensure consistent updates

        # **Stop the Robot After Duration Ends**
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info("🛑 Stopping robot.")


class PickAndPlaceSequence:
    """Handles the entire pick-and-place sequence using MoveToPosition and GripperController."""

    def __init__(self):
        self.arm = Gen3LiteArm()

    def move_to_pose(self, x, y, z, qx, qy, qz, qw, description=""):
        """Moves the arm to a specific pose using MoveToPosition."""
        mover = MoveToPosition()

        # ✅ Set movement parameters correctly
        mover.set_parameters([
            Parameter("x", Parameter.Type.DOUBLE, x),
            Parameter("y", Parameter.Type.DOUBLE, y),
            Parameter("z", Parameter.Type.DOUBLE, z),
            Parameter("qx", Parameter.Type.DOUBLE, qx),
            Parameter("qy", Parameter.Type.DOUBLE, qy),
            Parameter("qz", Parameter.Type.DOUBLE, qz),
            Parameter("qw", Parameter.Type.DOUBLE, qw),
        ])

        mover.execute_movement()
        mover.destroy_node()
        print(f"✅ {description} reached.")

    def control_gripper(self, command, position=0.5):
        """Controls the gripper using GripperController."""
        gripper_node = GripperController()

        # ✅ Set gripper control parameters
        gripper_node.set_parameters([
            Parameter("command", Parameter.Type.STRING, command),
            Parameter("position", Parameter.Type.DOUBLE, position),
        ])

        gripper_node.control_gripper()
        gripper_node.destroy_node()

    def execute_sequence(self):
        """Executes the full pick-and-place sequence."""
        positions = [
            (0.380, -0.012, 0.4, 1.000, 0.002, 0.011, -0.012, "Pre-Pick Position"),
            (0.380, -0.012, 0.124, 1.000, 0.002, 0.011, -0.012, "Pick Position"),
            ("open",),  # Open gripper
            (0.380, -0.012, 0.23, 1.000, 0.002, 0.011, -0.012, "Lift Position"),
            (-0.314, 0.047, 0.163, -0.542, 0.832, 0.071, -0.095, "Pre-Put Position 1"),
            (-0.314, 0.041, -0.036, -0.548, 0.835, -0.006, -0.044, "Pre-Put Position 2"),
            (-0.267, -0.007, -0.232, -0.552, 0.829, -0.084, 0.007, "Put Position 1"),
            (-0.257, -0.019, -0.315, -0.553, 0.829, -0.085, 0.008, "Put Position 2"),
            ("close",),  # Close gripper
        ]

        for action in positions:
            if isinstance(action, tuple):
                if len(action) == 1:
                    self.control_gripper(action[0])  # Gripper operation
                else:
                    self.move_to_pose(*action)  # Move arm to pose

def execute_movement_sequence(movement_list):
    """Executes a sequence of TurtleBot3 movements."""
    for move in movement_list:
        controller = TurtleBot3Controller()
        controller.set_parameters([
            Parameter("linear_vel", Parameter.Type.DOUBLE, move["linear_vel"]),
            Parameter("angular_vel", Parameter.Type.DOUBLE, move["angular_vel"]),
            Parameter("duration", Parameter.Type.DOUBLE, move["duration"]),
        ])
        controller.move()
        controller.destroy_node()
        time.sleep(1)
    time.sleep(3)


def main():
    """Executes the full movement sequence and pick-and-place task."""
    rclpy.init()

    # 🔄 Movement before pick-and-place
    movements_before = [
        {"linear_vel": 0.0, "angular_vel": -0.5, "duration": 1.0},
        {"linear_vel": 0.2, "angular_vel": 0.0, "duration": 5.5},
        {"linear_vel": 0.0, "angular_vel": 0.5, "duration": 2.7},
        {"linear_vel": 0.2, "angular_vel": 0.0, "duration": 8.0},
        {"linear_vel": 0.0, "angular_vel": -0.5, "duration": 2.4},
        {"linear_vel": 0.2, "angular_vel": 0.0, "duration": 6.0},
        {"linear_vel": 0.0, "angular_vel": 0.5, "duration": 1.2},
    ]

    execute_movement_sequence(movements_before)

    # 🛠 Execute Pick-and-Place Sequence
    pick_and_place = PickAndPlaceSequence()
    pick_and_place.execute_sequence()

    # 🔄 Movement after pick-and-place
    movements_after = [
        {"linear_vel": 0.0, "angular_vel": -0.5, "duration": 1.0},
        {"linear_vel": -0.2, "angular_vel": 0.0, "duration": 6.0},
        {"linear_vel": 0.0, "angular_vel": 0.5, "duration": 2.35},
        {"linear_vel": -0.2, "angular_vel": 0.0, "duration": 7.5},
        {"linear_vel": 0.0, "angular_vel": -0.5, "duration": 2.5},
        {"linear_vel": -0.2, "angular_vel": 0.0, "duration": 5.5},
        {"linear_vel": 0.0, "angular_vel": 0.5, "duration": 1.1},
    ]

    execute_movement_sequence(movements_after)

    # 🔚 Shutdown everything
    rclpy.shutdown()
    print("\n✅ Task completed successfully!")


if __name__ == "__main__":
    main()

