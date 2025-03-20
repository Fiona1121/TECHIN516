#!/usr/bin/env python3

# import os
# import rclpy
# import time
# from rclpy.node import Node
# from rclpy.parameter import Parameter
# from geometry_msgs.msg import Pose, Twist
# from .gen3lite_pymoveit2 import Gen3LiteArm
# from .go_to_position import MoveToPosition  # Import MoveToPosition class
# from .control_gripper import GripperController


# class TurtleBot3Controller(Node):
#     """Handles linear and angular movement with speed control and acceleration."""
    
#     MAX_SPEED = 0.24  # Set max allowed speed (TurtleBot3 firmware limit)
#     ACCELERATION_STEP = 0.02  # Gradual increase per step

#     def __init__(self):
#         super().__init__('turtlebot3_controller')

#         # Declare parameters
#         self.declare_parameter('linear_vel', 0.2)  # Default 0.2 m/s
#         self.declare_parameter('angular_vel', 0.0)  # Default 0 rad/s
#         self.declare_parameter('duration', 5.0)  # Default 5 seconds

#         # Create publisher for velocity commands
#         self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

#         # Ensure node has initialized before sending commands
#         time.sleep(1)

#     def move(self):
#         """Executes movement with acceleration control."""
        
#         # Fetch parameters
#         target_linear_velocity = self.get_parameter('linear_vel').value
#         angular_velocity = self.get_parameter('angular_vel').value
#         duration = self.get_parameter('duration').value

#         # Enforce speed limit
#         if target_linear_velocity > self.MAX_SPEED:
#             self.get_logger().warn(f"⚠️ Speed {target_linear_velocity} m/s exceeds max limit! Setting to {self.MAX_SPEED} m/s.")
#             target_linear_velocity = self.MAX_SPEED

#         # Validate duration
#         if duration <= 0:
#             self.get_logger().warn("⚠️ Duration must be greater than 0! Setting to 1 second.")
#             duration = 1.0

#         # Log movement settings
#         self.get_logger().info(f"🚀 Moving: Linear={target_linear_velocity} m/s, Angular={angular_velocity} rad/s, Duration={duration}s")

#         msg = Twist()
#         current_speed = 0.0  # Start from 0 speed
#         start_time = time.time()

#         # **Accelerate Gradually** to Target Speed
#         while time.time() - start_time < duration:
#             # Increase or decrease speed gradually
#             if current_speed < target_linear_velocity:
#                 current_speed += self.ACCELERATION_STEP
#                 if current_speed > target_linear_velocity:
#                     current_speed = target_linear_velocity
#             elif current_speed > target_linear_velocity:
#                 current_speed -= self.ACCELERATION_STEP
#                 if current_speed < target_linear_velocity:
#                     current_speed = target_linear_velocity

#             msg.linear.x = current_speed
#             msg.angular.z = angular_velocity
#             self.publisher_.publish(msg)
#             self.get_logger().info(f"🔄 Accelerating: Speed={current_speed:.2f} m/s")

#             time.sleep(0.1)  # Ensure consistent updates

#         # **Stop the Robot After Duration Ends**
#         msg.linear.x = 0.0
#         msg.angular.z = 0.0
#         self.publisher_.publish(msg)
#         self.get_logger().info("🛑 Stopping robot.")


# class PickAndPlaceSequence:
#     """Handles the entire pick-and-place sequence using MoveToPosition and GripperController."""

#     def __init__(self):
#         self.arm = Gen3LiteArm()

#     def move_to_pose(self, x, y, z, qx, qy, qz, qw, description=""):
#         """Moves the arm to a specific pose using MoveToPosition."""
#         mover = MoveToPosition()

#         # ✅ Set movement parameters correctly
#         mover.set_parameters([
#             Parameter("x", Parameter.Type.DOUBLE, x),
#             Parameter("y", Parameter.Type.DOUBLE, y),
#             Parameter("z", Parameter.Type.DOUBLE, z),
#             Parameter("qx", Parameter.Type.DOUBLE, qx),
#             Parameter("qy", Parameter.Type.DOUBLE, qy),
#             Parameter("qz", Parameter.Type.DOUBLE, qz),
#             Parameter("qw", Parameter.Type.DOUBLE, qw),
#         ])

#         mover.execute_movement()
#         mover.destroy_node()
#         print(f"✅ {description} reached.")

#     def control_gripper(self, command, position=0.5):
#         """Controls the gripper using GripperController."""
#         gripper_node = GripperController()

#         # ✅ Set gripper control parameters
#         gripper_node.set_parameters([
#             Parameter("command", Parameter.Type.STRING, command),
#             Parameter("position", Parameter.Type.DOUBLE, position),
#         ])

#         gripper_node.control_gripper()
#         gripper_node.destroy_node()

#     def execute_sequence(self):
#         """Executes the full pick-and-place sequence."""
#         positions = [
#             ("close",), 
#             # (0.365, -0.012, 0.39, 1.000, 0.002, 0.011, -0.012, "Pre-Pick Position"),
#             (0.390, 0.071, 0.120, 1.000, 0.002, 0.011, -0.012, "Pick Position"),
#             ("open",),  # Open gripper
#             (0.390, 0.071, 0.39, 1.000, 0.002, 0.011, -0.012, "Lift Position"),
#             (-0.314, 0.047, 0.163, -0.542, 0.832, 0.071, -0.095, "Pre-Put Position 1"),
#             (-0.314, 0.041, -0.036, -0.548, 0.835, -0.006, -0.044, "Pre-Put Position 2"),
#             (-0.267, -0.007, -0.232, -0.552, 0.829, -0.084, 0.007, "Put Position 1"),
#             # (-0.257, -0.019, -0.315, -0.553, 0.829, -0.085, 0.008, "Put Position 2"),
#             ("close",),  # Close gripper
#         ]

#         self.arm.go_vertical()

#         for action in positions:
#             if isinstance(action, tuple):
#                 if len(action) == 1:
#                     self.control_gripper(action[0])  # Gripper operation
#                 else:
#                     self.move_to_pose(*action)  # Move arm to pose

# def execute_movement_sequence(movement_list):
#     """Executes a sequence of TurtleBot3 movements."""
#     for move in movement_list:
#         controller = TurtleBot3Controller()
#         controller.set_parameters([
#             Parameter("linear_vel", Parameter.Type.DOUBLE, move["linear_vel"]),
#             Parameter("angular_vel", Parameter.Type.DOUBLE, move["angular_vel"]),
#             Parameter("duration", Parameter.Type.DOUBLE, move["duration"]),
#         ])
#         controller.move()
#         controller.destroy_node()
#         time.sleep(1)
#     time.sleep(3)


# def main():
#     """Executes the full movement sequence and pick-and-place task."""
#     rclpy.init()

#     # 🛠 Set new value
#     # new_server = "10.19.159.77:11811"
#     # os.environ["ROS_DISCOVERY_SERVER"] = new_server
#     # time.sleep(3)

#     # # 🔄 Movement before pick-and-place
#     # movements_before = [
#     #     # {"linear_vel": 0.0, "angular_vel": -0.5, "duration": 1.0},
#     #     # {"linear_vel": 0.2, "angular_vel": 0.0, "duration": 5.5},
#     #     # {"linear_vel": 0.0, "angular_vel": 0.5, "duration": 2.7},
#     #     # {"linear_vel": 0.2, "angular_vel": 0.0, "duration": 8.0},
#     #     # {"linear_vel": 0.0, "angular_vel": -0.5, "duration": 2.3},
#     #     # {"linear_vel": 0.2, "angular_vel": 0.0, "duration": 6.3},
#     #     # {"linear_vel": 0.0, "angular_vel": 0.5, "duration": 0.8},
#     #     {"linear_vel": 0.2, "angular_vel": 0.0, "duration": 16.0},
#     # ]

#     # execute_movement_sequence(movements_before)




#     # 🔍 Print current value before unsetting
#     # print(f"🔍 Before unsetting: {os.environ.get('ROS_DISCOVERY_SERVER')}")

#     # ❌ Unset the variable
#     # os.environ.pop("ROS_DISCOVERY_SERVER", None)
#     # time.sleep(3)

#     # 🔍 Print after unsetting
#     # print(f"🔍 After unsetting: {os.environ.get('ROS_DISCOVERY_SERVER')}")

#     # 🛠 Execute Pick-and-Place Sequence
#     pick_and_place = PickAndPlaceSequence()
#     pick_and_place.execute_sequence()

    

#     # 🛠 Set new value
#     # os.environ["ROS_DISCOVERY_SERVER"] = new_server
#     # time.sleep(3)

#     # print(f"🔍 After setting: {os.environ.get('ROS_DISCOVERY_SERVER')}")

#     # 🔄 Movement after pick-and-place
#     # movements_after = [
#     #     # {"linear_vel": 0.0, "angular_vel": -0.5, "duration": 0.8},
#     #     # {"linear_vel": -0.2, "angular_vel": 0.0, "duration": 6.3},
#     #     # {"linear_vel": 0.0, "angular_vel": 0.5, "duration": 2.3},
#     #     # {"linear_vel": -0.2, "angular_vel": 0.0, "duration": 8.0},
#     #     # {"linear_vel": 0.0, "angular_vel": -0.5, "duration": 2.7},
#     #     # {"linear_vel": -0.2, "angular_vel": 0.0, "duration": 5.5},
#     #     # {"linear_vel": 0.0, "angular_vel": 0.5, "duration": 1.0},
#     #     {"linear_vel": -0.2, "angular_vel": 0.0, "duration": 16.0},
#     # ]

#     # execute_movement_sequence(movements_after)

#     # 🔚 Shutdown everything
#     rclpy.shutdown()
#     print("\n✅ Task completed successfully!")


# if __name__ == "__main__":
#     main()


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
    # movements_before = [
    #     # {"linear_vel": 0.0, "angular_vel": -0.5, "duration": 2.0},
    #     {"linear_vel": 0.2, "angular_vel": 0.0, "duration": 16.5},
    #     # {"linear_vel": 0.0, "angular_vel": 0.5, "duration": 2.7},
    #     # {"linear_vel": 0.2, "angular_vel": 0.0, "duration": 8.0},
    #     # {"linear_vel": 0.0, "angular_vel": -0.5, "duration": 2.3},
    #     # {"linear_vel": 0.2, "angular_vel": 0.0, "duration": 6.0},
    #     # {"linear_vel": 0.0, "angular_vel": 0.5, "duration": 0.8},
    # ]

    # # Generate reverse sequence
    # movements_reverse = [
    #     {"linear_vel": -move["linear_vel"], "angular_vel": -move["angular_vel"], "duration": move["duration"]}
    #     for move in reversed(movements_before)
    # ]

    # # Move the TurtleBot forward
    # turtlebot = TurtleBotController()
    # turtlebot.move_sequence(movements_before)

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
    # turtlebot.move_sequence(movements_reverse)

    # Clean up and shut down
    # turtlebot.destroy_node()
    rclpy.shutdown()

    gripper.shutdown()
    arm.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# import rclpy
# import threading
# from rclpy.node import Node
# from rclpy.executors import MultiThreadedExecutor
# from geometry_msgs.msg import Twist, Pose
# import time
# from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper


# class TurtleBotController(Node):
#     """Controls the TurtleBot's movement."""
#     def __init__(self):
#         super().__init__('turtlebot_controller')
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

#     def move_sequence(self, movements):
#         """Executes a sequence of TurtleBot movements asynchronously."""
#         for move in movements:
#             twist = Twist()
#             twist.linear.x = move["linear_vel"]
#             twist.angular.z = move["angular_vel"]

#             self.get_logger().info(f"Moving: linear={move['linear_vel']} m/s, angular={move['angular_vel']} rad/s for {move['duration']} sec")

#             start_time = time.time()
#             while (time.time() - start_time) < move["duration"]:
#                 self.cmd_vel_publisher.publish(twist)
#                 time.sleep(0.1)  # Small delay to ensure updates

#             # Stop after each step
#             twist.linear.x = 0.0
#             twist.angular.z = 0.0
#             self.cmd_vel_publisher.publish(twist)
#             time.sleep(0.5)  # Pause between movements

#         self.get_logger().info("✅ TurtleBot movement sequence complete!")


# class KinovaArmController(Node):
#     """Handles the pick-and-place sequence using the Kinova arm."""
#     def __init__(self):
#         super().__init__('kinova_arm_controller')
#         self.arm = Gen3LiteArm()
#         self.gripper = Gen3LiteGripper()

#     def set_pose(self, x, y, z, qx, qy, qz, qw):
#         """Creates and returns a Pose object."""
#         pose = Pose()
#         pose.position.x = x
#         pose.position.y = y
#         pose.position.z = z
#         pose.orientation.x = qx
#         pose.orientation.y = qy
#         pose.orientation.z = qz
#         pose.orientation.w = qw
#         return pose

#     def execute_pick_and_place(self):
#         """Executes the pick and place sequence."""
#         self.arm.go_vertical()

#         # Define Kinova arm poses
#         pre_pick_pose = self.set_pose(0.370, -0.092, 0.371, 0.817, 0.574, -0.036, 0.028)
#         pick_pose = self.set_pose(0.370, -0.035, 0.130, 0.758, 0.652, 0.008, 0.017)
#         pre_put_pose1 = self.set_pose(-0.293, 0.081, 0.228, 0.752, 0.658, 0.023, -0.027)
#         pre_put_pose2 = self.set_pose(-0.174, -0.002, 0.191, 0.494, 0.867, -0.007, -0.066)
#         put_pose1 = self.set_pose(-0.195, 0.034, -0.164, 0.494, 0.867, -0.008, -0.066)
#         put_pose2 = self.set_pose(-0.173, 0.0, -0.342, 0.542, 0.838, -0.04, -0.042)
#         put_pose3 = self.set_pose(-0.159, 0.085, -0.336, -0.018, 0.997, 0.026, -0.076)

#         # Pick and Place Sequence
#         self.arm.inverse_kinematic_movement(pre_pick_pose)
#         time.sleep(0.5)
#         self.gripper.open()

#         self.arm.inverse_kinematic_movement(pick_pose)
#         self.gripper.close()
#         self.get_logger().info("✅ Picked up the cube!")

#         self.arm.inverse_kinematic_movement(pre_put_pose1)
#         time.sleep(0.5)
#         self.arm.inverse_kinematic_movement(pre_put_pose2)
#         time.sleep(0.5)
#         self.arm.inverse_kinematic_movement(put_pose1)
#         time.sleep(0.5)
#         self.arm.inverse_kinematic_movement(put_pose2)
#         time.sleep(0.5)

#         self.gripper.open()
#         self.get_logger().info("✅ Finished placing!")

#     def shutdown(self):
#         """Shuts down Kinova hardware safely."""
#         self.gripper.shutdown()
#         self.arm.shutdown()


# def main():
#     rclpy.init()

#     # Define TurtleBot movement sequence
#     movements_before = [
#         {"linear_vel": 0.2, "angular_vel": 0.0, "duration": 16.5},
#     ]

#     # Generate reverse movement sequence
#     movements_reverse = [
#         {"linear_vel": -move["linear_vel"], "angular_vel": -move["angular_vel"], "duration": move["duration"]}
#         for move in reversed(movements_before)
#     ]

#     # Create TurtleBot and Kinova Nodes
#     turtlebot_node = TurtleBotController()
#     kinova_arm_node = KinovaArmController()

#     # Create MultiThreadedExecutor
#     executor = MultiThreadedExecutor()
#     executor.add_node(turtlebot_node)
#     executor.add_node(kinova_arm_node)

#     # Start executor in a separate thread
#     executor_thread = threading.Thread(target=executor.spin, daemon=True)
#     executor_thread.start()

#     # Move the TurtleBot forward
#     turtlebot_thread = threading.Thread(target=turtlebot_node.move_sequence, args=(movements_before,))
#     kinova_thread = threading.Thread(target=kinova_arm_node.execute_pick_and_place)

#     turtlebot_thread.start()
#     kinova_thread.start()

#     turtlebot_thread.join()
#     kinova_thread.join()

#     # Move the TurtleBot back
#     turtlebot_node.move_sequence(movements_reverse)

#     # Clean up and shut down
#     turtlebot_node.destroy_node()
#     kinova_arm_node.shutdown()
#     kinova_arm_node.destroy_node()
#     executor.shutdown()
#     rclpy.shutdown()

#     print("\n✅ Task completed successfully!")


# if __name__ == '__main__':
#     main()
