# ORBIT Setup, Installation & Operation Guide

This guide provides step-by-step instructions for configuring the development environment, setting up the onboard NVIDIA Jetson Orin compute platform, building the ROS 2 packages, and executing SLAM mapping and autonomous navigation runs.

---

## 1. Prerequisites and System Requirements

### Target Platform (NVIDIA Jetson Orin)
- **Operating System:** Ubuntu 22.04 LTS (Jammy Jellyfish, 64-bit ARM / `aarch64`)
- **ROS 2 Distribution:** ROS 2 Humble Hawksbill (`ros-humble-desktop` or `ros-humble-ros-base`)
- **Python Version:** Python 3.10+
- **Build System:** `colcon` with `ament_python`

### Host Workstation (Remote Control & Monitoring)
- Any Linux, macOS, or Windows machine with OpenSSH client.
- For remote graphical visualization, either:
  1. An Ubuntu 22.04 workstation with ROS 2 Humble installed (recommended for native RViz2 DDS streaming), or
  2. SSH client with X11 forwarding enabled (`ssh -X` or `ssh -Y`).

---

## 2. Remote SSH Access and Network Setup

Connect the Jetson Orin and your workstation to the same local Wi-Fi network (or assign a static IP to the robot's Ethernet/WLAN interface).

### Connect via SSH
```bash
ssh <username>@<jetson-ip-address>
```
*Example:*
```bash
ssh orbit@192.168.1.120
```

### Serial Port Permissions
Grant the user account access to the serial USB communication devices (`/dev/ttyUSB0` for RPLidar and `/dev/ttyACM0` for ZLAC8015D driver):
```bash
sudo usermod -a -G dialout $USER
```
*(Log out and log back in for permissions to take effect).*

Confirm devices are detected:
```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

---

## 3. Installing Dependencies

Install the required ROS 2 Humble perception, navigation, and SLAM packages:

```bash
sudo apt update && sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-laser-filters \
    ros-humble-rplidar-ros \
    ros-humble-teleop-twist-keyboard \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-rviz2 \
    python3-colcon-common-extensions
```

---

## 4. Workspace Configuration and Building

Create a ROS 2 workspace (or navigate to your existing colcon workspace):

```bash
mkdir -p ~/orbit_ws/src
cd ~/orbit_ws/src

# Clone the repository
git clone https://github.com/Kanishkar843/ORBIT.git

# Build the path planning package
cd ~/orbit_ws
colcon build --symlink-install --packages-select path_planning_pkg

# Source the workspace setup
source /opt/ros/humble/setup.bash
source ~/orbit_ws/install/setup.bash
```

> **Pro Tip:** Add the source commands to your `~/.bashrc` file on the Jetson Orin:
> ```bash
> echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
> echo "source ~/orbit_ws/install/setup.bash" >> ~/.bashrc
> ```

---

## 5. Operational Workflow

Operating the ORBIT robot involves a structured two-phase procedure:
1. **Phase 1: Environment Mapping (SLAM)** — Drive the robot manually to scan and build an occupancy grid map.
2. **Phase 2: Autonomous Service Navigation** — Load the verified map, localize with AMCL, and dispatch autonomous delivery tasks.

```
+-------------------------------------------------------+
|  PHASE 1: SLAM MAPPING                                |
|  ros2 launch path_planning_pkg mapping_real.launch.py |
|  - Drive via teleop keys                              |
|  - slam_toolbox generates 2D occupancy grid           |
|  - Save map: ros2 run nav2_map_server map_saver_cli   |
+-------------------------------------------------------+
                           |
                           v
+-------------------------------------------------------+
|  PHASE 2: AUTONOMOUS NAVIGATION                       |
|  ros2 launch path_planning_pkg navigation_real...     |
|  - AMCL probabilistic localization                    |
|  - A* global path planning + DWB local avoidance      |
|  - ros2 run path_planning_pkg navigator (Waypoints)   |
+-------------------------------------------------------+
```

---

### Step 5.1: SLAM Mapping (Real Hardware)

Launch the complete SLAM stack including hardware bringup, RPLidar C1 driver, laser filters, and `slam_toolbox`:

```bash
ros2 launch path_planning_pkg mapping_real.launch.py
```

*Custom Serial Port Arguments:*
```bash
ros2 launch path_planning_pkg mapping_real.launch.py serial_port:=/dev/ttyACM0
```

**What happens during launch:**
- `t = 0s`: `hardware_bringup`, RPLidar driver, static TF publishers, and `laser_filters` initialize.
- `t = 2s`: `teleop_twist_keyboard` opens in an interactive terminal window.
- `t = 5s`: `slam_toolbox` begins map generation from `/scan_filtered`.
- `t = 7s`: `rviz2` opens to display the real-time mapping output.

Use the arrow keys in the teleop window to drive the robot smoothly through the dining/cafeteria area.

---

### Step 5.2: Saving the Map

Once the environment has been fully explored, save the occupancy grid to disk from a new terminal:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_dining_map
```

This command generates two files:
- `my_dining_map.yaml` (Map metadata, origin coordinates, resolution)
- `my_dining_map.pgm` (2D grayscale occupancy grid bitmap)

Copy or link these files into `path_planning_pkg/maps/` for production navigation.

---

### Step 5.3: Autonomous Navigation

Launch the Nav2 navigation stack configured with the saved map:

```bash
ros2 launch path_planning_pkg navigation_real.launch.py map:=/path/to/my_dining_map.yaml
```

If the `map` argument is omitted, the launch script automatically defaults to the pre-packaged verified map:
`path_planning_pkg/maps/map_1777558867.yaml`.

**Initial Pose Estimation:**
1. In the RViz2 window, click the **2D Pose Estimate** button on the top toolbar.
2. Click and drag the green arrow on the map at the robot's physical starting position and orientation.
3. Observe the AMCL particle cloud converging around the robot footprint.

---

### Step 5.4: Food-Serving Multi-Goal Navigation

Dispatch the robot on an automated sequential delivery route using the `navigator` action client:

```bash
ros2 run path_planning_pkg navigator
```

*Custom Table Waypoints via Parameter:*
Pass semicolon-separated coordinates in the format `x,y,yaw_degrees`:
```bash
ros2 run path_planning_pkg navigator --ros-args -p goals:="1.5,0.0,0.0; 1.5,2.0,90.0; 0.0,2.0,180.0; 0.0,0.0,-90.0"
```

---

### Step 5.5: Single Goal Injection (CLI)

For rapid point-to-point testing or table delivery from the command line:

```bash
ros2 run path_planning_pkg goal_publisher --ros-args \
    -p x:=1.2 -p y:=0.8 -p yaw:=45.0
```

---

## 6. Troubleshooting and Edge Diagnostics

### Issue 1: "Extrapolation into the future" TF Errors
- **Cause:** Duplicate `robot_state_publisher` nodes running simultaneously, broadcasting conflicting timestamps.
- **Solution:** Verify that only `hardware_bringup.launch.py` launches `robot_state_publisher`. Do not start a second instance.

### Issue 2: Nav2 Lifecycle Nodes Time Out During Startup
- **Cause:** CPU/Memory contention when initializing all Nav2 nodes simultaneously on edge hardware.
- **Solution:** The launch scripts use `TimerAction` stages (staggered over 10 seconds). Do not reduce these delays unless running on high-power x86 workstations.

### Issue 3: Serial Port Inaccessible (`Permission Denied`)
- **Cause:** User does not belong to the Linux `dialout` group.
- **Solution:** Run `sudo usermod -a -G dialout $USER` and reboot or re-login.

### Issue 4: LiDAR Reads False Obstacles on Chassis Rear
- **Cause:** Rear laser beam reflects off internal chassis or service trays.
- **Solution:** Verify that `laser_filters` node is running and downstream nodes subscribe to `/scan_filtered` rather than raw `/scan`.
