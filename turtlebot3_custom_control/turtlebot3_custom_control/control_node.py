#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBot3Controller(Node):
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

        # Ensure the node has initialized before sending commands
        time.sleep(1)

    def move(self):
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
            # Increase speed gradually
            if current_speed < target_linear_velocity:
                current_speed += self.ACCELERATION_STEP
                if current_speed > target_linear_velocity:
                    current_speed = target_linear_velocity  # Cap at target speed
            elif current_speed > target_linear_velocity:
                current_speed -= self.ACCELERATION_STEP
                if current_speed < target_linear_velocity:
                    current_speed = target_linear_velocity  # Cap at target speed


            msg.linear.x = current_speed
            msg.angular.z = angular_velocity
            self.publisher_.publish(msg)
            self.get_logger().info(f"🔄 Accelerating: Speed={current_speed:.2f} m/s")

            # time.sleep(0.025)  # Ensure consistent updates

        # **Stop the Robot After Duration Ends**
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info("🛑 Stopping robot.")

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBot3Controller()

    try:
        controller.move()
    except Exception as e:
        controller.get_logger().error(f"❌ Error: {e}")
    finally:
        controller.get_logger().info("Shutting down node.")
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
