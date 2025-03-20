#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class TurtleBotController(Node):
    def __init__(self, speed=0.1, distance=1.0):
        super().__init__('turtlebot_controller')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = speed  # Set the speed (positive for moving forward)
        self.target_distance = distance  # Set the target distance to move
        self.movement_complete = False

    # Function to move the TurtleBot and display progress in meters
    def move(self):
        print("Starting to move forward...")

        twist = Twist()
        twist.linear.x = self.speed  # Set the forward speed

        # Calculate total duration and setup for progress display
        duration = self.target_distance / abs(self.speed)
        start_time = time.time()

        # Initialize distance tracking
        traveled_distance = 0.0

        # Publish speed command and display progress
        while not self.movement_complete and (time.time() - start_time) < duration:
            self.cmd_vel_publisher.publish(twist)
            elapsed_time = time.time() - start_time
            traveled_distance = abs(self.speed) * elapsed_time

            # Print progress in meters
            print(f"Moved: {traveled_distance:.2f} meters", end='\r')

            # Add a small delay to avoid overloading the command buffer
            time.sleep(0.1)

        # Stop the TurtleBot
        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.movement_complete = True

        print(f"\nFinished! Moved a total of {traveled_distance:.2f} meters.")


def main():
    rclpy.init()

    # Move forward by 3.3 meters
    node = TurtleBotController(speed=0.1, distance=3.3)
    node.move()

    # Move backward by 3.2 meters
    node2 = TurtleBotController(speed=-0.1, distance=3.2)
    node2.move()

    # Clean up and shut down
    node.destroy_node()
    node2.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

