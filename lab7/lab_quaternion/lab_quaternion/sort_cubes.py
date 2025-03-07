import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper


class CubeSorter(Node):
    def __init__(self):
        super().__init__('cube_sorter')
        self.arm = Gen3LiteArm()
        self.gripper = Gen3LiteGripper()
        self.arm.go_vertical()

    def move_to_pose(self, target_pose, description=""):
        """Moves the arm to a specific pose while checking for errors."""
        self.get_logger().info(f"Attempting to move to {description}: {target_pose.position}, {target_pose.orientation}")

        # Get current pose for reference
        current_pose = self.arm.get_end_effector_pose()
        self.get_logger().info(f"Current pose before move: {current_pose.position}, {current_pose.orientation}")

        # Move position first, keeping orientation the same
        intermediate_pose = Pose()
        intermediate_pose.position = target_pose.position
        intermediate_pose.orientation = current_pose.orientation  # Keep existing orientation
        self.arm.inverse_kinematic_movement(intermediate_pose)

        # Adjust orientation separately
        final_pose = Pose()
        final_pose.position = target_pose.position  # Keep position
        final_pose.orientation = target_pose.orientation
        self.arm.inverse_kinematic_movement(final_pose)

        # Confirm movement
        new_pose = self.arm.get_end_effector_pose()
        self.get_logger().info(f"New pose after move: {new_pose.position}, {new_pose.orientation}")

    def pick_and_place(self, pick_pose, place_pose):
        """Handles picking up and placing a cube while moving position and orientation separately."""
        approach_height = 0.70  # Safe height before descending
        transit_height = 0.80  # High enough to avoid obstacles
        final_offset = 0.02  # Height buffer after placement

        # **Step 1: Move up to a safe approach height**
        above_pick = Pose()
        above_pick.position = Point(x=pick_pose.position.x, y=pick_pose.position.y, z=approach_height)
        above_pick.orientation = pick_pose.orientation
        self.move_to_pose(above_pick, "Lifting to Safe Pick Height")

        # **Step 2: Move in X-Y plane to above the pick position (no orientation change)**
        xy_pick = Pose()
        xy_pick.position = Point(x=pick_pose.position.x, y=pick_pose.position.y, z=approach_height)
        xy_pick.orientation = pick_pose.orientation
        self.move_to_pose(xy_pick, "Moving Over Pick Position")

        # **Step 3: Lower to pick cube**
        self.move_to_pose(pick_pose, "Lowering to Pick Cube")

        # **Step 4: Close gripper**
        self.gripper.close()

        # **Step 5: Lift cube up before moving sideways**
        lifted_pose = Pose()
        lifted_pose.position = Point(x=pick_pose.position.x, y=pick_pose.position.y, z=transit_height)
        lifted_pose.orientation = pick_pose.orientation
        self.move_to_pose(lifted_pose, "Lifting Cube to Transit Height")

        # **Step 6: Move in X-Y plane to the place position (keeping transit height)**
        xy_place = Pose()
        xy_place.position = Point(x=place_pose.position.x, y=place_pose.position.y, z=transit_height)
        xy_place.orientation = pick_pose.orientation  # Keep same orientation for now
        self.move_to_pose(xy_place, "Moving Over Place Position")

        # **Step 7: Adjust orientation separately at transit height**
        final_oriented_place = Pose()
        final_oriented_place.position = xy_place.position
        final_oriented_place.orientation = place_pose.orientation
        self.move_to_pose(final_oriented_place, "Adjusting Orientation Before Placement")

        # **Step 8: Lower to place object with a slight offset**
        place_pose.position.z += final_offset
        self.move_to_pose(place_pose, "Lowering to Place Cube")

        # **Step 9: Open gripper to release cube**
        self.gripper.open()

        # **Step 10: Move slightly higher after placement**
        place_pose.position.z += final_offset
        self.move_to_pose(place_pose, "Raising After Placement")

        # **Step 11: Return to home position**
        self.arm.go_home()

    def sort_cubes(self):
        """Sorts cubes into correct bins with separate position and orientation steps."""
        self.get_logger().info("Starting Sorting Task")

        # Define block positions (from sort_world.sdf)
        green_block = Pose()
        green_block.position = Point(x=0.4, y=0.3, z=0.53)
        green_block.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Move green block to table2 (offset in Y)
        green_bin = Pose()
        green_bin.position = Point(x=0.4, y=-0.4, z=0.53)
        green_bin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        red_block = Pose()
        red_block.position = Point(x=0.4, y=-0.3, z=0.53)
        red_block.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Move red block to table1 (offset in Y)
        red_bin = Pose()
        red_bin.position = Point(x=0.4, y=0.4, z=0.53)
        red_bin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Move cubes to correct tables
        self.get_logger().info("Sorting Green Cube")
        self.pick_and_place(green_block, green_bin)

        self.get_logger().info("Sorting Red Cube")
        self.pick_and_place(red_block, red_bin)

        self.get_logger().info("Sorting Task Completed!")


def main():
    rclpy.init()
    sorter = CubeSorter()
    sorter.sort_cubes()
    sorter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
