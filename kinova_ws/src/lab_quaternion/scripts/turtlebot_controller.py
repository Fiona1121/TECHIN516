#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def move_sequence(self, movements):
        for move in movements:
            twist = Twist()
            twist.linear.x = move["linear_vel"]
            twist.angular.z = move["angular_vel"]
            self.get_logger().info(f"Moving: linear={move['linear_vel']}, angular={move['angular_vel']} for {move['duration']}s")

            start_time = time.time()
            while (time.time() - start_time) < move["duration"]:
                self.cmd_vel_publisher.publish(twist)
                time.sleep(0.1)

            # Stop after each movement step
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.5)

        self.get_logger().info("✅ TurtleBot movement completed!")


def main():
    rclpy.init()
    turtlebot_node = TurtleBotController()
    movements = [
        {"linear_vel": 0.2, "angular_vel": 0.0, "duration": 16.3},
        # Add more movements if needed
    ]
    turtlebot_node.move_sequence(movements)
    time.sleep(2.5*60)
    movements_reverse = [
        {"linear_vel": -0.2, "angular_vel": 0.0, "duration": 16.3},
    ]
    turtlebot_node.move_sequence(movements_reverse)
    turtlebot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()