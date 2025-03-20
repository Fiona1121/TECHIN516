import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        self.client = ActionClient(self, GripperCommand, '/gen3_lite_2f_gripper_controller/gripper_cmd')

    def control_gripper(self, command):
        """Controls the gripper by sending an action goal."""
        goal_msg = GripperCommand.Goal()
        if command == "open":
            goal_msg.command.position = 0.85  # Fully open
        elif command == "close":
            goal_msg.command.position = 0.0  # Fully closed
        else:
            self.get_logger().error(f"Invalid command: {command}")
            return

        goal_msg.command.max_effort = 5.0
        self.client.wait_for_server()
        self.get_logger().info(f"Sending gripper command: {command}")
        self.client.send_goal(goal_msg)

def main():
    rclpy.init()
    gripper = GripperController()
    gripper.control_gripper("open")  # Change to "close" for closing
    rclpy.shutdown()

if __name__ == "__main__":
    main()
