#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class TurtleBot3DistanceController(Node):
    MAX_SPEED = 0.24  # Max linear speed for TurtleBot3
    MAX_ANGULAR_SPEED = 1.82  # Max angular speed
    ACCELERATION_STEP = 0.02  # Linear acceleration step
    ANGULAR_STEP = 0.1  # Angular acceleration step

    def __init__(self):
        super().__init__('turtlebot3_distance_controller')

        # Declare distance and rotation parameters
        self.declare_parameter('distance', 1.0)  # Default move 1 meter
        self.declare_parameter('angle', 0.0)  # Default no rotation

        # Get parameter values
        self.target_distance = self.get_parameter('distance').value
        self.target_angle = math.radians(self.get_parameter('angle').value)  # Convert degrees to radians

        # Initialize position variables
        self.current_x = None
        self.current_y = None
        self.current_theta = None
        self.start_x = None
        self.start_y = None
        self.start_theta = None
        self.odom_received = False

        # Create velocity publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to odometry for position feedback
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info("✅ Initialized. Waiting for /odom...")

    def odom_callback(self, msg):
        """ Updates the robot's current position from odometry. """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to yaw angle
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

        # Store starting position when odometry is first received
        if not self.odom_received:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_theta = self.current_theta
            self.odom_received = True
            self.get_logger().info("✅ Odometry received. Ready to move.")

    def wait_for_odometry(self, timeout=5):
        """ Wait for odometry before moving, with a timeout. """
        start_time = time.time()
        while not self.odom_received and (time.time() - start_time) < timeout and rclpy.ok():
            self.get_logger().warn("⚠️ Waiting for /odom...")
            time.sleep(1)

        if not self.odom_received:
            self.get_logger().error("❌ No /odom received! Ensure TurtleBot3 is running and publishing odometry.")
            return False  # Return False if odometry is not received
        return True  # Return True when odometry is available

    def move(self):
        """ Moves the TurtleBot3 based on distance and angle parameters. """
        if not self.wait_for_odometry():
            return  # Exit if odometry is not available

        if abs(self.target_distance) > 0:
            self.move_straight(self.target_distance)

        if abs(self.target_angle) > 0:
            self.rotate(self.target_angle)

    def move_straight(self, target_distance):
        """ Moves the robot straight using odometry feedback. """
        if self.start_x is None or self.start_y is None:
            self.get_logger().error("❌ Cannot move: Missing start position from odometry.")
            return

        self.get_logger().info(f"🚀 Moving {target_distance:.2f} meters")
        msg = Twist()
        current_speed = 0.0

        while rclpy.ok():
            # Ensure odometry is valid
            if self.current_x is None or self.current_y is None:
                self.get_logger().warn("⚠️ Odometry lost, stopping movement.")
                break

            # Calculate traveled distance
            traveled_distance = math.sqrt((self.current_x - self.start_x) ** 2 + (self.current_y - self.start_y) ** 2)

            if traveled_distance >= abs(target_distance):
                break

            # Accelerate smoothly
            if current_speed < self.MAX_SPEED:
                current_speed += self.ACCELERATION_STEP
                current_speed = min(current_speed, self.MAX_SPEED)

            # Set movement direction
            msg.linear.x = current_speed if target_distance > 0 else -current_speed
            self.publisher_.publish(msg)

            self.get_logger().info(f"🔄 Moving: {traveled_distance:.2f}m / {target_distance:.2f}m")
            time.sleep(0.1)

        # Decelerate before stopping
        self.decelerate(msg)
        self.get_logger().info("🛑 Stopped moving.")

    def rotate(self, target_angle):
        """ Rotates the robot by a given angle using odometry feedback. """
        if self.start_theta is None:
            self.get_logger().error("❌ Cannot rotate: Missing start orientation from odometry.")
            return

        self.get_logger().info(f"🔄 Rotating {math.degrees(target_angle):.2f} degrees")
        msg = Twist()
        current_angular_speed = 0.0

        while rclpy.ok():
            # Ensure odometry is valid
            if self.current_theta is None:
                self.get_logger().warn("⚠️ Odometry lost, stopping rotation.")
                break

            # Calculate rotation progress
            angle_traveled = self.current_theta - self.start_theta

            if abs(angle_traveled) >= abs(target_angle):
                break

            # Accelerate rotation smoothly
            if current_angular_speed < self.MAX_ANGULAR_SPEED:
                current_angular_speed += self.ANGULAR_STEP
                current_angular_speed = min(current_angular_speed, self.MAX_ANGULAR_SPEED)

            # Set rotation direction
            msg.angular.z = current_angular_speed if target_angle > 0 else -current_angular_speed
            self.publisher_.publish(msg)

            self.get_logger().info(f"🔄 Rotating: {math.degrees(angle_traveled):.2f}° / {math.degrees(target_angle):.2f}°")
            time.sleep(0.1)

        # Stop rotation
        self.decelerate(msg, angular=True)
        self.get_logger().info("🛑 Rotation complete.")

    def decelerate(self, msg, angular=False):
        """ Gradually slows down before stopping for smooth control. """
        step = self.ANGULAR_STEP if angular else self.ACCELERATION_STEP
        speed_attr = 'angular.z' if angular else 'linear.x'

        while getattr(msg, speed_attr) > 0:
            setattr(msg, speed_attr, max(getattr(msg, speed_attr) - step, 0))
            self.publisher_.publish(msg)
            time.sleep(0.1)

        # Stop completely
        setattr(msg, speed_attr, 0.0)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBot3DistanceController()

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
