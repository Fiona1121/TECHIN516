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

    time.sleep(2)  # Small delay between commands

def main():
    """Executes a sequence of ROS2 commands for movement and gripper control."""

    commands = [
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.1 -p y:=0.1 -p z:=0.7 -p qx:=0.0 -p qy:=0.0 -p qz:=0.0 -p qw:=1.0",
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.2 -p y:=0.2 -p z:=0.6 -p qx:=0.0 -p qy:=0.0 -p qz:=0.0 -p qw:=1.0",
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.2 -p y:=0.2 -p z:=0.6 -p qx:=0.3 -p qy:=0.0 -p qz:=0.0 -p qw:=0.8",
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.2 -p y:=0.2 -p z:=0.6 -p qx:=0.7 -p qy:=0.0 -p qz:=0.0 -p qw:=0.7",
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.3 -p y:=0.3 -p z:=0.5 -p qx:=0.7 -p qy:=0.0 -p qz:=0.0 -p qw:=0.7",
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.3 -p y:=0.3 -p z:=0.5 -p qx:=-0.023 -p qy:=0.622 -p qz:=0.050 -p qw:=0.781",
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.4 -p y:=0.3 -p z:=0.5 -p qx:=-0.023 -p qy:=0.622 -p qz:=0.050 -p qw:=0.781",
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.4 -p y:=0.3 -p z:=0.5 -p qx:=0.012 -p qy:=0.720 -p qz:=0.003 -p qw:=0.694",
        "ros2 run lab_quaternion control_gripper --ros-args -p command:=open",
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.3 -p y:=0.3 -p z:=0.5 -p qx:=0.012 -p qy:=0.720 -p qz:=0.003 -p qw:=0.694",
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.3 -p y:=0.3 -p z:=0.239 -p qx:=0.012 -p qy:=0.720 -p qz:=0.003 -p qw:=0.694",
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.3 -p y:=0.3 -p z:=0.239 -p qx:=0.012 -p qy:=0.726 -p qz:=0.003 -p qw:=0.687",
        "ros2 run lab_quaternion control_gripper --ros-args -p command:=close",
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.3 -p y:=0.3 -p z:=0.4 -p qx:=0.012 -p qy:=0.726 -p qz:=0.003 -p qw:=0.687",
        "ros2 run lab_quaternion go_to_position --ros-args -p x:=0.3 -p y:=-0.2 -p z:=0.4 -p qx:=0.012 -p qy:=0.726 -p qz:=0.003 -p qw:=0.687",
    ]

    for command in commands:
        run_ros2_command(command)


if __name__ == "__main__":
    main()
