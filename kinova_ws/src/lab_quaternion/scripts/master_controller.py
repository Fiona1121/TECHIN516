#!/usr/bin/env python3
import subprocess
import time

def run_kinova():
    return subprocess.Popen(["bash", "-c", "export ROS_DOMAIN_ID=1 && python3 kinova_controller.py"])

def run_turtlebot():
    return subprocess.Popen(["bash", "-c", "export ROS_DOMAIN_ID=2 && python3 turtlebot_controller.py"])

def main():
    print("🚀 Launching TurtleBot and Kinova sequences in parallel...")

    turtlebot_process = run_turtlebot()
    kinova_process = run_kinova()

    # Optionally wait for both to finish
    turtlebot_process.wait()
    kinova_process.wait()

    print("✅ Both sequences completed.")

if __name__ == '__main__':
    main()
