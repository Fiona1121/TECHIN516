# 🚀 Kinova + TurtleBot3 Pick-and-Place Demo

This project demonstrates a **ROS2-based pick-and-place operation** using:

- ✅ **Kinova Gen3 Lite Arm** for precise object manipulation.
- ✅ **TurtleBot3** for autonomous navigation.
- ✅ **Master Controller** to manage high-level execution of movements.

## 🎯 **System Overview**

### **1️⃣ Master Controller (`master_controller.py`)**

The **`master_controller.py`** is the central script that orchestrates:

- **TurtleBot3 movement** before and after the pick-and-place sequence.
- **Kinova Gen3 Lite arm control** for accurate pick-and-place operations.
- **Gripper control** to interact with objects.
- **Environment variable management** for ROS2 communication.

### **2️⃣ TurtleBot3 Navigation**

- Moves to the pick location **before** the Kinova arm picks up the object.
- After placing the object, it **returns to the starting position**.
- Ensures **smooth acceleration** and respects **speed limits**.

### **3️⃣ Kinova Gen3 Lite Arm**

- Moves to predefined positions using **inverse kinematics**.
- Picks up and places objects using the **Kinova gripper**.
- Uses **MoveIt!** for trajectory planning.

## 🚀 **Launch Process**

### **1️⃣ Start the Fast DDS Discovery Server**

```bash
fastdds discovery --server-id 0
```

### **2️⃣ Start Kinova Gen3 Lite**

#### **Launch Kortex Bringup**

```bash
export ROS_DOMAIN_ID=1 && ros2 launch kortex_bringup gen3_lite.launch.py robot_ip:=10.18.2.240 launch_rviz:=false
```

#### **Launch MoveIt! for Kinova**

```bash
export ROS_DOMAIN_ID=1 && ros2 launch kinova_gen3_lite_moveit_config robot.launch.py robot_ip:=10.18.2.240
```

### **3️⃣ Start TurtleBot3**

#### **SSH into the TurtleBot3**

```bash
ssh ubuntu@10.18.3.90
```

#### **Launch TurtleBot3 Bringup**

```bash
export ROS_DOMAIN_ID=2
ros2 launch turtlebot3_bringup robot.launch.py
```

### **4️⃣ Start the Master Controller**

On the **main control machine**, run:

```bash
python3 master_controller.py
```

## 📂 **Project Structure**

```
kinova_ws/
│── lab_quaternion/
│   ├── scripts/
│   │   ├── master_controller.py  # Main high-level controller
│   │   ├── kinova_controller.py  # Kinova arm control
│   │   ├── turtlebot_controller.py  # TurtleBot3 movement
│   │   ├── go_to_position.py  # Arm positioning
│   │   ├── control_gripper.py  # Gripper controls
│── README.md  # This documentation
```

## 🎯 **Key Features**

- **Master Controller** ensures synchronized execution.
- **TurtleBot3 navigation** with **speed control**.
- **Kinova arm control** for precision movement.
- **MoveIt! integration** for trajectory planning.
- **ROS2-based communication** with **Fast DDS discovery**.
