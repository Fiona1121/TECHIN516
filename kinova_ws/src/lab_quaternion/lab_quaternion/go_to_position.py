import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Pose, Point, Quaternion
from .gen3lite_pymoveit2 import Gen3LiteArm
import sys


class MoveToPosition(Node):
    def __init__(self):
        super().__init__('move_to_position')
        self.arm = Gen3LiteArm()

        # Declare ROS2 parameters with default values
        self.declare_parameter("x", 0.4)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.6)
        self.declare_parameter("qx", 0.0)
        self.declare_parameter("qy", 0.0)
        self.declare_parameter("qz", 0.0)
        self.declare_parameter("qw", 1.0)

    def move_to_pose(self, target_pose, description=""):
        """Moves the arm to a specific pose with retries if needed."""
        max_attempts = 5
        attempt = 0

        while attempt < max_attempts:
            self.get_logger().info(f"Attempt {attempt + 1}/{max_attempts}: Moving to {description} - {target_pose.position}, {target_pose.orientation}")

            try:
                self.arm.inverse_kinematic_movement(target_pose)

                # Verify if movement was successful
                new_pose = self.arm.get_end_effector_pose()
                if self._poses_are_close(new_pose, target_pose):
                    self.get_logger().info(f"✅ Successfully moved to {description} on attempt {attempt + 1}.")
                    return True
            except Exception as e:
                self.get_logger().error(f"❌ Failed attempt {attempt + 1} to move to {description}. Error: {str(e)}")

            attempt += 1

        self.get_logger().error(f"🚨 Failed to move to {description} after {max_attempts} attempts. Shutting down.")
        rclpy.shutdown()
        return False

    def _poses_are_close(self, pose1, pose2, tolerance=0.01):
        """Check if two poses are close enough within a given tolerance."""
        return (abs(pose1.position.x - pose2.position.x) < tolerance and
                abs(pose1.position.y - pose2.position.y) < tolerance and
                abs(pose1.position.z - pose2.position.z) < tolerance)

    def execute_movement(self):
        """Main function to move the arm to a designated position."""
        # Retrieve parameters from command-line input
        x = self.get_parameter("x").value
        y = self.get_parameter("y").value
        z = self.get_parameter("z").value
        qx = self.get_parameter("qx").value
        qy = self.get_parameter("qy").value
        qz = self.get_parameter("qz").value
        qw = self.get_parameter("qw").value

        # **Define Target Position**
        target_pose = Pose()
        target_pose.position = Point(x=x, y=y, z=z)
        target_pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        self.get_logger().info(f"🛠 Moving to Designated Position: x={x}, y={y}, z={z}, qx={qx}, qy={qy}, qz={qz}, qw={qw}")

        if self.move_to_pose(target_pose, "Designated Position"):
            self.get_logger().info("🎯 Arm successfully reached the target position!")


def main(args=None):
    rclpy.init(args=args)
    mover = MoveToPosition()
    mover.execute_movement()
    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
