# WAM Teleoperation Package

**Note:**  
This version of the code corresponds to the **horizontal configuration** of the WAM teleoperation setup used in the experiments reported in the associated Frontiers manuscript.

---

# Overview

This package implements a **bilateral teleoperation system between two Barrett WAM manipulators**:

- **Leader:** 4-DOF WAM equipped with a haptic wrist  
- **Follower:** 7-DOF WAM manipulator

Although the software is structured as a **ROS package**, ROS is **not used for control communication** between the robots.

Instead:

- **UDP communication** is used for real-time signal exchange between the leader and follower.
- **ROS topics** are used only to publish robot states for **data logging and analysis**.

This design keeps the teleoperation loop independent from ROS timing while still allowing convenient experimental data collection.

---

# Controllers Implemented

This repository includes several control architectures used to evaluate transparency and interaction performance in the WAM teleoperation system. All controllers operate in **joint space**, and interaction forces are estimated using **inverse-dynamics-based external torque estimation**.

The following controllers are implemented:

- **Gravity Compensation (GC)**  
  Baseline two-channel position–position teleoperation with gravity compensation only.

- **Gravity Compensation with Force Feedforward (GC-FF)**  
  The estimated external torque at the leader is fed forward to the leader to improve interaction feedback.

- **Gravity Compensation with Local Force Feedback (GC-LFB)**  
  Local force feedback is applied on the leader using the estimated external torque.

- **Dynamic Compensation (DC)**  
  Full robot dynamics are compensated using the identified inverse dynamics model, reducing the apparent inertia of the robots and improving transparency.

These controllers correspond to the methods evaluated in the experiments reported in the associated manuscript.

---

# Build Instructions

Place the package in your ROS workspace:

```
<catkin_ws>/src/wam_teleop
```

Then build the workspace:

```bash
cd <catkin_ws>
catkin_make
```

## Building Only the Follower

If only the follower node is required:

```bash
catkin_make --cmake-args -DBUILD_LEADER=OFF
```

Alternatively modify the option in `CMakeLists.txt`:

```cmake
option(BUILD_LEADER "Build leader executable" OFF)
```

## Leader Dependency

The leader node requires the **haptic wrist library**.

Build and install it following the instructions here:

https://github.com/dmiller12/libhaptic_wrist

---

# Configuration

The `config/` directory contains the Barrett configuration files for the **leader** and **follower** robots.

To load the appropriate configuration file:

```bash
source scripts/setup_leader.sh
source scripts/setup_follower.sh
```

These scripts set the environment variable

```
BARRETT_CONFIG_FILE
```

which specifies the configuration file used by the Barrett WAM driver.

Note that these variables persist only for the **current terminal session**.

The CAN interface may need to be adjusted in:

```
config/leader.conf
config/follower.conf
```

depending on your hardware setup.

---

# Running the Teleoperation System

Each node supports the following interface:

```bash
rosrun wam_teleop leader [remoteHost] [recPort] [sendPort]
rosrun wam_teleop follower [remoteHost] [recPort] [sendPort]
```

Use:

```bash
--help
```

to see the available options.

---

# Example Execution

## 1. Source the workspace

In each terminal:

```bash
cd ~/amir/catkin_ws
source devel/setup.bash
```

---

## 2. Initialize CAN Interface

Navigate to the package directory:

```bash
cd ~/amir/catkin_ws/src/wam_teleop
```

For **PCI CAN interface**:

```bash
source scripts/pci_can_init.sh
```

For **USB CAN interface**:

```bash
source scripts/usb_can_init.sh
```

You may need to adjust the `can#` interface depending on the order in which the devices were connected.

---

## 3. Start ROS

```bash
roscore
```

---

## 4. Start the Leader

```bash
source scripts/setup_leader.sh
rosrun wam_teleop leader_nowrist 127.0.0.1 5555 5554
```

---

## 5. Start the Follower

```bash
source scripts/setup_follower.sh
rosrun wam_teleop follower 127.0.0.1 5554 5555
```

The receive and send ports must match between leader and follower:

| Robot | Receive Port | Send Port |
|------|------|------|
| Leader | 5555 | 5554 |
| Follower | 5554 | 5555 |

---

# Teleoperation Procedure

Once both nodes are running:

1. On the **leader**, press `l` to move to the synchronization position.
2. On the **follower**, press `l` to move to the synchronization position.
3. Ensure both arms have reached the synchronization configuration.
4. Press **Enter** on the leader to establish the teleoperation link.
5. Press **Enter** on the follower to complete the connection.

The system is now ready for teleoperation.

---

# Shutdown Procedure

To ensure proper cleanup of threads and sockets:

1. Return both WAMs to the **home position**
2. On the **leader**, press `x` to exit the control loop
3. **Shift-idle** the leader
4. Repeat the same procedure for the follower
