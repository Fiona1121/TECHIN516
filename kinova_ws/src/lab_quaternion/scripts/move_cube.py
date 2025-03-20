import os
import time

def run_ros2_command(command):
    """Executes a ROS2 command and waits for completion."""
    print(f"\n🚀 Executing: {command}")
    exit_code = os.system(command)

    if exit_code != 0:
        print(f"⚠️ Command failed: {command}")
    else:
        print(f"✅ Completed: {command}")

    time.sleep(2)  # Small delay between commands to ensure execution

def main():
    """Executes a sequence of ROS2 commands for motion and gripper control."""

    commands = [
 
        # Close Gripper
        "ros2 run lab_quaternion control_gripper --ros-args -p command:=open",

        # Step 3 (Grip closed)
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.257 -p y:=-0.094 -p z:=0.103 -p qx:=0.076 -p qy:=0.977 -p qz:=-0.042 -p qw:=0.194",

        # Step 4
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.241 -p y:=-0.082 -p z:=0.271 -p qx:=0.076 -p qy:=0.978 -p qz:=-0.041 -p qw:=0.189",

        # Step 5
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.263 -p y:=-0.081 -p z:=0.361 -p qx:=0.083 -p qy:=0.996 -p qz:=-0.025 -p qw:=-0.005",

        # Step 6
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.239 -p y:=-0.226 -p z:=0.369 -p qx:=0.083 -p qy:=0.996 -p qz:=-0.025 -p qw:=-0.006",

        # Step 7
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.278 -p y:=-0.278 -p z:=0.306 -p qx:=0.077 -p qy:=0.978 -p qz:=-0.152 -p qw:=0.119",

        # Step 8
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.289 -p y:=-0.332 -p z:=0.245 -p qx:=0.062 -p qy:=0.988 -p qz:=-0.041 -p qw:=0.132",

        # Step 10
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.346 -p y:=-0.367 -p z:=0.206 -p qx:=0.062 -p qy:=0.988 -p qz:=-0.041 -p qw:=0.132",

        # Step 11
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.374 -p y:=-0.378 -p z:=0.102 -p qx:=0.063 -p qy:=0.988 -p qz:=-0.041 -p qw:=0.134",

        # Open Gripper
        "ros2 run lab_quaternion control_gripper --ros-args -p command:=close",
    ]

    for command in commands:
        run_ros2_command(command)


if __name__ == "__main__":
    main()
