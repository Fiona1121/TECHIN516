#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from threading import Thread
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy.exceptions
import rclpy.executors
from pymoveit2 import GripperInterface, MoveIt2, MoveIt2State
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState

class Gen3LiteGripper():
    def __init__(self):
        self.node = Node("gen3_lite_gripper")
        self.callback_group = ReentrantCallbackGroup()
        self.min_close = 0.0
        self.max_open = 0.85
        self.gripper = GripperInterface(
            node=self.node,
            gripper_joint_names=['right_finger_bottom_joint',],
            open_gripper_joint_positions=[self.max_open,],
            closed_gripper_joint_positions=[self.min_close,],
            gripper_group_name='gripper',
            callback_group=self.callback_group,
            gripper_command_action_name='gripper_action_controller/gripper_cmd',
        )
        # spin node in background and wait for ini
        self.executor = rclpy.executors.MultiThreadedExecutor(2)
        self.executor.add_node(self.node)
        self.executor_thread = Thread(target=self.executor.spin, daemon=True, args=())
        self.executor_thread.start()
        self.node.create_rate(1.0).sleep()

    def open(self):
        """open gripper"""
        self.node.get_logger().info('open gripper')
        self.gripper.open()
        self.gripper.wait_until_executed()
        time.sleep(1)
    
    def close(self):
        """close gripper"""
        self.node.get_logger().info('close gripper')
        self.gripper.close()
        self.gripper.wait_until_executed()
        time.sleep(1)
    
    def move_to_position(self, position):
        """
        move gripper to desired position
        position: float between 0.0 and 0.85
        returns: None
        """
        self.node.get_logger().info(f'moving gripper to position: {position}')
        if position > self.max_open or position < self.min_close:
            self.node.get_logger().warn('specified gripper position is outside of gripper range')
            return
        self.gripper.move_to_position(position)
        self.gripper.wait_until_executed()
        time.sleep(1)
    
    def shutdown(self):
        """destroy node"""
        self.executor_thread.join()
        exit(0)


class Gen3LiteArm():
    def __init__(self):
        """initialize MoveIt2 controls for a Kinova Gen 3 Lite arm"""
        self.node = Node("gen3_lite_arm")
        self.callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self.node,
            joint_names=['joint_1',
                         'joint_2',
                         'joint_3',
                         'joint_4',
                         'joint_5',
                         'joint_6',
                         'end_effector_link'],
            base_link_name='base_link',
            end_effector_name='end_effector_link',
            group_name='arm',
            callback_group=self.callback_group,
        )
        # subscriber to read current joint angles
        self.node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        # spin node in background thread and wait for init
        self.executor = rclpy.executors.MultiThreadedExecutor(2)
        self.executor.add_node(self.node)
        self.executor_thread = Thread(target=self.executor.spin, daemon=True, args=())
        self.executor_thread.start()
        self.node.create_rate(1.0).sleep()
        # set moveit planning parameters
        self.moveit2.pipeline_id = 'ompl'
        self.moveit2.planner_id = 'RRTConnectkConfigDefault'
        self.moveit2.allowed_planning_time = 5.0
        self.moveit2.num_planning_attempts = 10
        self.moveit2.max_velocity = 0.1
        self.moveit2.max_acceleration = 0.1
        self.moveit2.cartesian_jump_threshold = 0.0
        # variable for joint angles
        self.joint_angles = None
   
    def joint_state_callback(self, msg):
        """callback function for updating joint angles"""
        self.joint_state = list(msg.position)
        self.joint_state_timestamp = time.time()
        time.sleep(1)

    def inverse_kinematic_movement(self, target_pose):
        """
        plan and move to a given end effector pose
        target_pose: geometry_msgs Pose
        returns: None
        """
        self.node.get_logger().info(f'moving to pose: {target_pose.position} {target_pose.orientation}')
        self.moveit2.move_to_pose(
            pose=target_pose,
            cartesian=False,
            cartesian_max_step=0.0025,
            cartesian_fraction_threshold=0.0,
        )
        rate = self.node.create_rate(10)
        while self.moveit2.query_state() != MoveIt2State.EXECUTING:
            rate.sleep()
        future = self.moveit2.get_execution_future()
        while not future.done():
            rate.sleep()
    
    def forward_kinematic_movement(self, target_angles):
        """
        plan and move to a set of joint angles
        target_angles: list of floats (joints 1-6) of goal angles in radians
        returns: None
        """
        self.node.get_logger().info(f'moving to angles: {target_angles}')
        self.moveit2.move_to_configuration(joint_positions=target_angles)
        rate = self.node.create_rate(10)
        while self.moveit2.query_state() != MoveIt2State.EXECUTING:
            rate.sleep()
        future = self.moveit2.get_execution_future()
        while not future.done():
            rate.sleep()
    
    def get_end_effector_pose(self):
        """
        get current pose of the end effector link
        returns: geometry_msgs Pose
        """
        curr_pose_stamped = self.moveit2.compute_fk()
        pose = curr_pose_stamped.pose
        return pose
    
    def get_joint_angles(self):
        """
        get current joint angles in radians
        returns list or None
        """
        if self.joint_state is not None and (time.time() - self.joint_state_timestamp) < 1.0:
            return self.joint_state
    
    def go_home(self):
        """move to home position"""
        self.node.get_logger().info(f'moving to home position')
        self.forward_kinematic_movement([0.0, 0.0, 2.618, 0.0, 0.0, 0.0])
    
    def go_vertical(self):
        """move to vertical position"""
        self.node.get_logger().info(f'moving to vertical position')
        self.forward_kinematic_movement([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def shutdown(self):
        """destroy node"""
        self.executor_thread.join()
        exit(0)

class KinovaArmController(Node):
    def __init__(self):
        super().__init__('kinova_arm_controller')
        self.arm = Gen3LiteArm()
        self.gripper = Gen3LiteGripper()
        self.get_logger().info("✅ Kinova Arm Controller Node Initialized.")

    def set_pose(self, x, y, z, qx, qy, qz, qw):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def execute_pick_and_place(self):
        self.get_logger().info("🚀 Starting pick-and-place sequence...")
        
        self.get_logger().info("🔄 Moving arm to vertical home position...")
        self.arm.go_vertical()
        time.sleep(1.0)

        # Define poses
        pre_pick_pose = self.set_pose(0.370, -0.092, 0.371, 0.817, 0.574, -0.036, 0.028)
        pick_pose = self.set_pose(0.370, -0.035, 0.130, 0.758, 0.652, 0.008, 0.017)
        pre_put_pose1 = self.set_pose(-0.293, 0.081, 0.228, 0.752, 0.658, 0.023, -0.027)
        pre_put_pose2 = self.set_pose(-0.174, -0.002, 0.191, 0.494, 0.867, -0.007, -0.066)
        put_pose1 = self.set_pose(-0.195, 0.034, -0.164, 0.494, 0.867, -0.008, -0.066)
        put_pose2 = self.set_pose(-0.173, 0.0, -0.342, 0.542, 0.838, -0.04, -0.042)

        self.get_logger().info("🤖 Moving to pre-pick position...")
        self.arm.inverse_kinematic_movement(pre_pick_pose)
        time.sleep(0.5)

        self.get_logger().info("🛠 Closing gripper before pick...")
        self.gripper.move_to_position(0.0)

        self.get_logger().info("🤖 Moving to pick position...")
        self.arm.inverse_kinematic_movement(pick_pose)

        self.get_logger().info("🛠 Opening gripper to grasp object...")
        self.gripper.move_to_position(0.8)
        self.get_logger().info("✅ Object picked up!")

        self.get_logger().info("📦 Moving to pre-put position 1...")
        self.arm.inverse_kinematic_movement(pre_put_pose1)
        time.sleep(0.5)

        self.get_logger().info("📦 Moving to pre-put position 2...")
        self.arm.inverse_kinematic_movement(pre_put_pose2)
        time.sleep(0.5)

        self.get_logger().info("📦 Moving to put-down position 1...")
        self.arm.inverse_kinematic_movement(put_pose1)
        time.sleep(0.5)

        self.get_logger().info("📦 Moving to final put-down position...")
        self.arm.inverse_kinematic_movement(put_pose2)
        time.sleep(0.5)

        self.get_logger().info("🛠 Releasing object (closing gripper)...")
        self.gripper.move_to_position(0.0)
        self.get_logger().info("✅ Object placed successfully!")

        self.get_logger().info("🔄 Returning to vertical home position...")
        self.arm.go_vertical()
        time.sleep(1.0)

        self.get_logger().info("✅ Pick-and-place sequence completed!")

    def shutdown(self):
        self.get_logger().info("🛑 Shutting down gripper and arm...")
        self.gripper.shutdown()
        self.arm.shutdown()
        self.get_logger().info("✅ Kinova Arm Controller shut down cleanly.")


def main():
    rclpy.init()
    kinova_node = KinovaArmController()
    kinova_node.get_logger().info("✨ Ready to execute pick-and-place sequence.")
    kinova_node.execute_pick_and_place()
    kinova_node.shutdown()
    kinova_node.destroy_node()
    rclpy.shutdown()
    print("\n✅ Kinova pick-and-place script finished execution.")


if __name__ == '__main__':
    main()
