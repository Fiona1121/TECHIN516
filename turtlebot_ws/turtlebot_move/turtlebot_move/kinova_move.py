#!/usr/bin/env python3
import time
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from pyquaternion import Quaternion as PyQuaternion
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper  
import numpy as np

def execute_robotic_transfer(robot_arm, gripper_control, approach_position, grasp_position, 
                           transit_position1, transit_position2, release_position1, 
                           release_position2, release_position3):
    # Move to position above the object
    robot_arm.inverse_kinematic_movement(approach_position)
    time.sleep(0.5)
    
    # Open gripper
    gripper_control.move_to_position(0.0)
    
    # Move to object and grasp it
    robot_arm.inverse_kinematic_movement(grasp_position)
    gripper_control.move_to_position(0.8)
    print("got the cube")
    time.sleep(0.5)
    
    # Move through transit positions to destination
    robot_arm.inverse_kinematic_movement(transit_position1)
    time.sleep(0.5)
    robot_arm.inverse_kinematic_movement(transit_position2)
    time.sleep(0.5)
    
    # Position for release
    robot_arm.inverse_kinematic_movement(release_position1)
    time.sleep(0.5)
    robot_arm.inverse_kinematic_movement(release_position2)
    time.sleep(0.5)
    robot_arm.inverse_kinematic_movement(release_position3)
    time.sleep(0.5)
    
    # Release object
    gripper_control.move_to_position(0.0)
    print("finished!")

def create_robot_pose(position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w):
    new_pose = Pose()
    
    # Set position components
    new_pose.position.x = position_x
    new_pose.position.y = position_y
    new_pose.position.z = position_z
    
    # Set orientation components
    new_pose.orientation.x = orientation_x
    new_pose.orientation.y = orientation_y
    new_pose.orientation.z = orientation_z
    new_pose.orientation.w = orientation_w
    
    return new_pose
    
def main():   
    # Initialize ROS2
    rclpy.init()
    
    # Setup gripper and arm controllers
    gripper = Gen3LiteGripper()
    arm = Gen3LiteArm()
    arm.go_vertical()
    
    # Define waypoint positions
    approach_position = create_robot_pose(0.370, -0.092, 0.371, 0.817, 0.574, -0.036, 0.028)
    # Commented out alternative position
    # alt_approach_position = create_robot_pose(0.380, -0.030, 0.371, 0.817, 0.574, -0.036, 0.028)
    
    grasp_position = create_robot_pose(0.370, -0.092, 0.130, 0.758, 0.652, 0.008, 0.017)
    # Commented out alternative position
    # alt_grasp_position = create_robot_pose(0.394, -0.031, 0.161, 0.751, 0.659, -0.025, 0.017)
    
    transit_position1 = create_robot_pose(-.293, 0.081, 0.228, 0.752, 0.658, 0.023, -0.027)
    transit_position2 = create_robot_pose(-0.174, -0.002, 0.191, 0.494, 0.867, -0.007, -0.066)
    
    release_position1 = create_robot_pose(-0.195, 0.034, -0.164, 0.494, 0.867, -0.008, -0.066)
    release_position2 = create_robot_pose(-0.173, 0.0, -0.342, 0.542, 0.838, -0.04, -0.042)
    release_position3 = create_robot_pose(-0.159, 0.085, -0.336, -0.018, 0.997, 0.026, -0.076)
    
    # Note: standard movement takes 2min 5s
    
    # Execute the pick and place operation
    execute_robotic_transfer(arm, gripper, approach_position, grasp_position, 
                           transit_position1, transit_position2, release_position1, 
                           release_position2, release_position3)
    
    # Clean up resources
    rclpy.shutdown()
    gripper.shutdown()
    arm.shutdown()
    
if __name__ == '__main__':
    main()
