<div align="center">

# ORBIT
### LiDAR-Based Autonomous Food-Serving Robot

**A real-hardware autonomous mobile robot platform integrating 2D LiDAR SLAM, AMCL localization, A* path planning, and DWB obstacle avoidance deployed on NVIDIA Jetson Orin edge computing.**

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20Hawksbill-3498DB?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/)
[![Compute](https://img.shields.io/badge/Compute-NVIDIA%20Jetson%20Orin-76B900?style=flat-square&logo=nvidia)](https://developer.nvidia.com/embedded/jetson-modules)
[![Perception](https://img.shields.io/badge/LiDAR-SLAMTEC%20RPLidar%20C1-E74C3C?style=flat-square)](https://www.slamtec.com/)
[![Actuation](https://img.shields.io/badge/Actuation-ZLAC8015D%20BLDC%20Driver-F39C12?style=flat-square)](#hardware-architecture)
[![Navigation](https://img.shields.io/badge/Navigation-Nav2%20Stack-8E44AD?style=flat-square)](https://navigation.ros.org/)
[![SLAM](https://img.shields.io/badge/SLAM-Ceres%20Online%20Async-16A085?style=flat-square)](https://github.com/SteveMacenski/slam_toolbox)
[![OS](https://img.shields.io/badge/OS-Ubuntu%2022.04%20LTS%20(ARM64)-E95420?style=flat-square&logo=ubuntu)](https://ubuntu.com/)
[![Workflow](https://img.shields.io/badge/Deployment-Headless%20Remote%20SSH-2C3E50?style=flat-square)](https://www.openssh.com/)

[Overview](#project-overview) · [Capabilities](#key-capabilities) · [Architecture](#system-architecture) · [Hardware](#hardware-architecture) · [Tech Stack](#technology-stack) · [How It Works](#how-orbit-works) · [Installation](#installation-and-setup) · [Project Structure](#project-structure) · [Technical Challenges](#technical-challenges--engineering-solutions) · [Evaluation](#testing-and-evaluation) · [Research Relevance](#research-relevance--ccu-internship-alignment) · [Maintainer](#maintainer)

</div>

---

## Project Overview

**ORBIT** is a physical, LiDAR-driven autonomous food-serving mobile robot engineered for reliable point-to-point service delivery in indoor dining environments, institutional cafeterias, and hospitality facilities. Developed natively on an **NVIDIA Jetson Orin** edge computer running **ROS 2 Humble**, the platform addresses the challenge of navigating cluttered, dynamic indoor dining halls through deterministic range perception, robust probabilistic state estimation, and spill-safe trajectory control.

Rather than relying purely on software simulations, ORBIT is built, verified, and tuned on **physical differential-drive robotics hardware**:
- **Real-Time 2D LiDAR Perception:** Ingests laser scans from a **SLAMTEC RPLidar C1** (12 m radius) with custom angular bounds filtering to eliminate chassis self-obstruction.
- **Online Graph-Based SLAM:** Generates high-fidelity 2D occupancy grid maps using **Slam Toolbox** powered by the **Ceres nonlinear optimization solver** (Sparse Normal Cholesky linear solver).
- **Global Path Optimization:** Computes minimum-distance, collision-free global routes using **A\* graph search** configured inside Nav2 `NavfnPlanner`.
- **Dynamic Obstacle Avoidance:** Evaluates forward trajectory rollouts at **20 Hz** via the **DWB Local Planner**, tuned with kinematic velocity limits ($v_{\max} = 0.22\text{ m/s}$) to ensure smooth dish transport without spillage.
- **Edge Deployment Architecture:** Employs a staggered lifecycle management pipeline to eliminate memory contention and startup race conditions on edge ARM64 compute boards.

> **Research Portfolio Context:** This repository serves as primary technical documentation for international graduate research internship evaluation at **National Chung Cheng University (CCU), Taiwan — College of Engineering (2027)**, specifically targeting projects in **LiDAR & Camera Sensor Fusion (P6)**, **Autonomous LLM Robotic Systems (P7)**, and **Deep Learning Edge Computer Vision (P8)**.

---

## Key Capabilities

| Capability | Technical Implementation | Verification Status in Code |
| :--- | :--- | :--- |
| **Laser Range Sensing** | SLAMTEC RPLidar C1 2D range scanner via `rplidar_ros` on `/dev/ttyUSB0`. | **Implemented & Verified** |
| **Chassis Occlusion Masking**| `laser_filters` angular bounds filter suppressing rear 120° blind spot, yielding 240° active FOV (`/scan_filtered`). | **Implemented & Verified** |
| **Asynchronous 2D SLAM** | `slam_toolbox` online async mapping using Ceres Levenberg-Marquardt solver. | **Implemented & Verified** |
| **Occupancy Map Generation** | Pre-mapped indoor dining facility maps saved at $0.05\text{ m/cell}$ (`map_1777558867.pgm`, `my_map.pgm`). | **Verified Real Artifacts** |
| **Adaptive Localization** | `nav2_amcl` particle filter (500–2000 particles) using Likelihood Field model and differential kinematics. | **Implemented & Verified** |
| **A\* Global Path Planning** | Nav2 `NavfnPlanner` with explicit A\* heuristic search prunes node expansions across global costmaps. | **Implemented & Verified** |
| **Local Obstacle Avoidance** | DWB Local Planner (`dwb_core::DWBLocalPlanner`) running at 20 Hz with 2.5 s forward trajectory rollouts. | **Implemented & Verified** |
| **Multi-Goal Waypoint Sequencing**| Custom Python action client node (`navigator.py`) dispatching sequential dining table delivery poses. | **Implemented & Verified** |
| **CLI Goal Injection** | Custom ROS 2 node (`goal_publisher.py`) publishing discrete targets to `/goal_pose` via command-line parameters. | **Implemented & Verified** |
| **Staged Lifecycle Management** | Timed startup sequence ($t=0\text{s}$ to $10\text{s}$) preventing CPU throttling and lifecycle bond timeouts on Jetson. | **Implemented & Verified** |
| **Dual BLDC Actuation** | Closed-loop wheel velocity control via ZLAC8015D driver over serial bus `/dev/ttyACM0`. | **Integrated via Sibling Driver** |
| **Extrinsic Camera Transform** | Static coordinate transformation frame (`camera`) pre-configured in TF tree for visual perception. | **Configured in Launch Tree** |

---

## System Architecture

ORBIT organizes perception, spatial estimation, global path planning, local reactive control, and hardware actuation into an asynchronous ROS 2 Humble node graph:

```mermaid
flowchart TD
    subgraph PERCEPTION["Sensory Perception & Filtering"]
        LIDAR["RPLidar C1 (2D LiDAR)\nSerial: /dev/ttyUSB0"] -->|Raw Scan: /scan| FILTER["laser_filters\nLaserScanAngularBoundsFilter\nMasks [-120°, +120°]"]
        FILTER -->|Filtered Scan: /scan_filtered| S_OUT["Clean 240° Active Scan"]
    end

    subgraph ESTIMATION["Spatial Estimation & Localization"]
        S_OUT --> SLAM["slam_toolbox\nCeres Solver (Levenberg-Marquardt)"]
        SLAM -.->|Phase 1: Build Map| MAP_FILE["Occupancy Grid Map\n(0.05m/cell PGM/YAML)"]
        MAP_FILE --> MAP_SERVER["nav2_map_server"]
        MAP_SERVER --> AMCL["nav2_amcl\nAdaptive Monte Carlo Localization"]
        S_OUT --> AMCL
        AMCL -->|TF: map → odom| TF_TREE["Coordinate TF Tree"]
    end

    subgraph PLANNING["Nav2 Autonomy & Path Planning"]
        NAV_CLIENT["navigator.py / goal_publisher.py\nService Waypoint Dispatcher"] -->|NavigateToPose Action| BT["nav2_bt_navigator\nBehavior Tree Engine"]
        BT --> GLOBAL_PLAN["nav2_planner (NavfnPlanner)\nA* Global Graph Search"]
        GLOBAL_PLAN --> G_COST["Global Costmap\nStatic + Obstacle + Inflation (0.65m)"]
        BT --> LOCAL_PLAN["nav2_controller (DWBLocalPlanner)\nTrajectory Rollouts @ 20 Hz"]
        LOCAL_PLAN --> L_COST["Local Costmap\nRolling Obstacle Layer"]
        BT --> RECOVERIES["nav2_behaviors\nSpin | BackUp | Wait"]
    end

    subgraph HARDWARE["Actuation & Kinematics"]
        LOCAL_PLAN -->|Velocity: /cmd_vel| MOTOR_DRIVER["ZLAC8015D Motor Driver\nSerial: /dev/ttyACM0"]
        MOTOR_DRIVER --> MOTORS["Dual BLDC In-Wheel Motors\nDifferential Kinematics"]
        MOTORS --> ODOM["Encoder Feedback\nTF: odom → base_footprint"]
        ODOM --> TF_TREE
    end
```

### Coordinate Transformation (TF) Hierarchy

The physical robot frames comply strictly with **ROS REP-105**:
$$\mathbf{map} \xrightarrow{\text{AMCL / SLAM}} \mathbf{odom} \xrightarrow{\text{Driver}} \mathbf{base\_footprint} \xrightarrow{\text{URDF}} \mathbf{base\_link} \xrightarrow{\text{URDF}} \mathbf{lidar}$$

```
map (Global World Origin)
 └── odom (Odometric Reference Frame)
      └── base_footprint (2D Ground Projection)
           └── base_link (Robot Centroid, z = 0.10 m)
                ├── left_wheel (x = 0.0 m, y = +0.15 m)
                ├── right_wheel (x = 0.0 m, y = -0.15 m)
                ├── caster_wheel (x = -0.15 m, y = 0.0 m, z = -0.05 m)
                ├── lidar (Optical Center, x = 0.0 m, y = 0.0 m, z = +0.18 m)
                └── camera [Configured] (x = +0.10 m, y = 0.0 m, z = +0.15 m)
```

---

## Hardware Architecture

```
                          Top View
             +-------------------------------+
             |                               |
             |       [ RPLIDAR C1 ]          |  x = 0.00 m, z = 0.18 m
             |       [ CAMERA MOUNT ]        |  x = 0.10 m, z = 0.15 m
             |                               |
    +--------+-------------------------------+--------+
    | [LEFT] |                               | [RIGHT]|
    | DRIVE  |      NVIDIA JETSON ORIN       | DRIVE  |
    | WHEEL  |      ZLAC8015D BLDC DRIVER    | WHEEL  |
    | (0.1m) |                               | (0.1m) |
    +--------+-------------------------------+--------+
             |                               |
             |                               |  Track Width: 0.30 m
             |       [ PASSIVE CASTER ]      |  Length: 0.50 m, Width: 0.30 m
             +-------------------------------+  x = -0.15 m, z = -0.05 m
```

### Detailed Component Specifications

| Subsystem | Specification / Model | Operating Parameters | Documentation & Evidence |
| :--- | :--- | :--- | :--- |
| **Compute Board** | NVIDIA Jetson Orin Series | Ubuntu 22.04 LTS (ARM64), ROS 2 Humble | Verified deployment target; staged lifecycle configs |
| **LiDAR Sensor** | SLAMTEC RPLidar C1 | 12.0 m max range, 10 Hz, 360° DFOV | Configured in `mapping_real.launch.py` |
| **Motor Controller** | ZLAC8015D Dual BLDC Controller | Dual-channel closed-loop servo, RS485/Serial | Declared dependency in `package.xml` |
| **Traction Actuators** | Dual In-Wheel Brushless DC Motors| In-wheel hub motors with quadrature encoders | Modeled in `diffbot.urdf` |
| **Chassis Dimensions** | Differential Drive Service Base | $0.50\text{ m (L)} \times 0.30\text{ m (W)} \times 0.15\text{ m (H)}$, mass $5.0\text{ kg}$ | Parameterized in `diffbot.urdf` |
| **Wheel Geometry** | Dual Traction + Rear Caster | Drive wheel radius $r = 0.10\text{ m}$, track width $L = 0.30\text{ m}$ | Parameterized in `diffbot.urdf` |
| **Optical Mount** | Forward Camera Frame | Coordinate offset $(x=+0.10, y=0.0, z=+0.15\text{ m})$ | Static TF in `mapping_real.launch.py` |
| **Power Supply** | High-Discharge DC Battery | Dedicated logic bus (Jetson) & power bus (Motors) | Hardware architecture |

---

## Technology Stack

| Layer | Technology | Version / Configuration | Purpose |
| :--- | :--- | :--- | :--- |
| **Compute Hardware** | NVIDIA Jetson Orin | ARM64 Ampere/Orin Architecture | Central onboard edge processor |
| **Operating System** | Ubuntu Linux | 22.04 LTS (Jammy Jellyfish) | Native embedded Linux environment |
| **Robotics Middleware**| ROS 2 | Humble Hawksbill | Inter-node DDS communication and lifecycle control |
| **Perception Sensing**| SLAMTEC RPLidar C1 | `rplidar_ros` on `/dev/ttyUSB0` | 2D laser range scanning (12 m envelope) |
| **Scan Filtering** | `laser_filters` | `LaserScanAngularBoundsFilterInPlace` | Masks rear $120^\circ$ chassis occlusion ($[-120^\circ, +120^\circ]$) |
| **2D SLAM** | `slam_toolbox` | Ceres Solver (Sparse Normal Cholesky) | Real-time online asynchronous occupancy mapping |
| **Localization** | `nav2_amcl` | Adaptive Monte Carlo (500–2000 particles) | Probabilistic pose estimation in verified maps |
| **Global Planning** | Nav2 `NavfnPlanner` | A\* Search Algorithm (`use_astar: true`) | Optimal, admissible global trajectory generation |
| **Local Control** | Nav2 `DWBLocalPlanner`| Dynamic Window Approach @ 20 Hz | Real-time obstacle avoidance and velocity dispatch |
| **Kinematic Modeling**| URDF (Unified Robot Desc.)| `diffbot.urdf` | Rigid body transforms, inertia, and visual frames |
| **Service Logic** | Python 3 | `rclpy` Action Client / Timers | Autonomous multi-waypoint dining room traversal |
| **Remote Access** | OpenSSH | Headless SSH, X11 Display Redirection | Remote code deployment, diagnostics, and monitoring |

---

## How ORBIT Works

```mermaid
flowchart TD
    A["1. SYSTEM INITIALIZATION\nJetson boots, launches staged lifecycle nodes (t=0s to 10s)"] --> B["2. PERCEPTION ACQUISITION\nRPLidar C1 streams /scan; laser_filters masks rear blind spot"]
    B --> C["3. SPATIAL LOCALIZATION\nAMCL registers /scan_filtered against pre-built dining map"]
    C --> D["4. SERVICE TASK DISPATCH\nnavigator.py queues multi-table delivery goals (x, y, yaw)"]
    D --> E["5. A* GLOBAL PATH PLANNING\nNavfn computes shortest collision-free trajectory on costmap"]
    E --> F["6. DWB TRAJECTORY ROLLOUTS\nEvaluates 25 speed & 25 angular samples at 20 Hz"]
    F --> G["7. MOTION EXECUTION\nZLAC8015D drives wheel motors (v <= 0.22 m/s, w <= 1.0 rad/s)"]
    G --> H{"Dynamic Obstacle\nEncountered?"}
    H -- Yes --> I["Trigger Recovery Behaviors\nWait (5s) -> Clear Costmap -> Spin -> BackUp"]
    I --> E
    H -- No --> J["8. DOCKING & FOOD HANDOVER\nArrival tolerance <= 0.25m; advance to next table"]
    J --> D
```

### Operational Phase Sequence

1. **Staged System Boot:** When launched, ROS 2 initializes hardware bringup and laser filtering at $t=0\text{ s}$, activates localization lifecycle managers at $t=5\text{ s}$, controllers at $t=6\text{ s}$, behavior servers at $t=7\text{ s}$, behavior tree navigators at $t=8\text{ s}$, and navigation lifecycle managers at $t=10\text{ s}$.
2. **Scan Processing:** Laser pulses are emitted at 10 Hz across 360°. The `laser_filters` node evaluates beam angles and suppresses reflections from the rear chassis ($[-2.0944\text{ rad}, +2.0944\text{ rad}]$ preserved).
3. **Map-Based Localization:** AMCL correlates range returns against the occupancy grid using a Likelihood Field model, publishing the correction transform `map -> odom`.
4. **Mission Planning:** High-level goals are issued to `nav2_bt_navigator`. The A\* planner evaluates grid cells $f(n) = g(n) + h(n)$ to generate the global path.
5. **Dynamic Control:** DWB projects 2.5-second forward kinematic trajectories at 20 Hz, discarding trajectories intersecting obstacle inflation zones and selecting the optimal velocity command.
6. **Task Completion:** Upon reaching the dining station within $\Delta xy \le 0.25\text{ m}$, the robot holds position for dish handover before advancing to the next waypoint.

---

## Installation and Setup

### 1. Prerequisites (NVIDIA Jetson Orin)
Ensure Ubuntu 22.04 LTS and ROS 2 Humble are installed:
```bash
sudo apt update && sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-laser-filters \
    ros-humble-rplidar-ros \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rviz2 \
    python3-colcon-common-extensions
```

### 2. Configure Serial Port Permissions
Grant the user account access to the serial USB communication devices:
```bash
sudo usermod -a -G dialout $USER
```
*(Log out and log back in for changes to apply).*

### 3. Clone and Build
```bash
# Create colcon workspace
mkdir -p ~/orbit_ws/src
cd ~/orbit_ws/src

# Clone ORBIT repository
git clone https://github.com/Kanishkar843/ORBIT.git

# Build path_planning_pkg
cd ~/orbit_ws
colcon build --symlink-install --packages-select path_planning_pkg

# Source workspace
source /opt/ros/humble/setup.bash
source ~/orbit_ws/install/setup.bash
```

### 4. Running Phase 1: SLAM Mapping
Drive the robot manually via keyboard teleoperation to generate an indoor occupancy grid:
```bash
ros2 launch path_planning_pkg mapping_real.launch.py
```
*Optional serial port override:*
```bash
ros2 launch path_planning_pkg mapping_real.launch.py serial_port:=/dev/ttyACM0
```
Save the generated map once the room has been fully mapped:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_dining_map
```

### 5. Running Phase 2: Autonomous Service Navigation
Launch autonomous navigation with the saved map:
```bash
ros2 launch path_planning_pkg navigation_real.launch.py map:=/path/to/my_dining_map.yaml
```
*(If `map` is omitted, the launch script defaults to verified map `path_planning_pkg/maps/map_1777558867.yaml`).*

Set the initial pose using the **2D Pose Estimate** button in RViz.

### 6. Executing Autonomous Delivery Sequences
Dispatch sequential table delivery waypoints:
```bash
ros2 run path_planning_pkg navigator
```
Or inject a custom single table goal via CLI:
```bash
ros2 run path_planning_pkg goal_publisher --ros-args \
    -p x:=1.5 -p y:=0.5 -p yaw:=90.0
```

---

## Project Structure

```
ORBIT/
├── .gitignore                          # Ignores ROS logs, build/install artifacts, secrets
├── README.md                           # Master research-grade project documentation
├── docs/                               # Comprehensive engineering deep-dives
│   ├── ARCHITECTURE.md                 # System diagrams, TF tree, lifecycle gantt charts
│   ├── HARDWARE.md                     # Jetson Orin, RPLidar C1, ZLAC8015D, kinematic specs
│   ├── NAVIGATION.md                   # A* mathematical formulation, DWB tuning, AMCL, costmaps
│   ├── RESEARCH.md                     # Academic mapping to CCU Taiwan 2027 internship (P6, P7, P8)
│   ├── SETUP.md                        # Step-by-step installation, building, and launch guides
│   └── TESTING.md                      # Static code QA, map analysis, proposed evaluation framework
└── path_planning_pkg/                  # ROS 2 Humble Navigation Package (Real Hardware)
    ├── package.xml                     # Package manifest, ROS 2 dependencies, metadata
    ├── setup.py                        # Python setuptools build script, console entrypoints
    ├── setup.cfg                       # Script directory configurations
    ├── config/                         # Parameter configuration manifests
    │   ├── amcl.yaml                   # Likelihood Field particle filter localization params
    │   ├── laser_filters.yaml          # Rear blind spot angular masking (-120° to +120°)
    │   ├── mapper_params_online_async.yaml # Ceres solver online asynchronous SLAM parameters
    │   ├── nav2_astar_planning.yaml    # Navfn global planner configured with A* search
    │   ├── nav2_bt.yaml                # Behavior Tree navigator & recovery plugin definitions
    │   └── nav2_controller.yaml        # DWB local planner velocity, acceleration, & sample limits
    ├── launch/                         # Structured launch orchestration scripts
    │   ├── mapping_real.launch.py      # Real-hardware SLAM mapping & teleoperation launch
    │   └── navigation_real.launch.py   # Staged autonomous Nav2 navigation launch
    ├── maps/                           # Verified real-hardware experimental map artifacts
    │   ├── map_1777558867.pgm          # Real indoor occupancy grid bitmap (39.0 KB)
    │   ├── map_1777558867.yaml         # Map metadata, resolution (0.05m), origin coordinates
    │   ├── my_map.pgm                  # High-definition perimeter scan bitmap (41.4 KB)
    │   └── my_map.yaml                 # Occupancy thresholds and origin coordinates
    ├── path_planning_pkg/              # Python source module
    │   ├── __init__.py                 # Package initialization
    │   └── nodes/                      # ROS 2 Python nodes
    │       ├── __init__.py             # Nodes module initialization
    │       ├── goal_publisher.py       # Single-pose CLI injection tool
    │       └── navigator.py            # Multi-goal NavigateToPose action client sequencer
    ├── resource/                       # ROS 2 ament resource index markers
    │   └── path_planning_pkg           # Resource index token
    ├── rviz/                           # Pre-configured RViz visualization profiles
    │   ├── rviz_mapping.rviz           # Display config for live SLAM map building
    │   └── rviz_navigation.rviz        # Display config for costmaps, particles, and paths
    └── urdf/                           # Kinematic description & physical specifications
        └── diffbot.urdf                # Differential drive robot URDF (links, joints, materials)
```

---

## Technical Challenges & Engineering Solutions

### 1. Physical Chassis Self-Obstruction in Laser Scanning
- **Challenge:** The RPLidar C1 is mounted toward the front of the mobile base. Laser pulses emitted across the rear $120^\circ$ wedge reflected off the food-carrying service trays and internal chassis frame, causing the costmap to generate lethal obstacles immediately on top of the robot's footprint ($r < 0.22\text{ m}$).
- **Solution:** Implemented `laser_filters/LaserScanAngularBoundsFilterInPlace` in `laser_filters.yaml`. All scan returns outside $[-2.0944\text{ rad}, +2.0944\text{ rad}]$ are masked out before passing to SLAM and Nav2. Downstream nodes subscribe exclusively to `/scan_filtered`.

### 2. Double-RSP Coordinate Conflict & TF Extrapolation Jitter
- **Challenge:** Concurrently launching `robot_state_publisher` in both `hardware_bringup` and `navigation_real.launch.py` caused duplicate TF publishers to broadcast conflicting timestamps for `base_link`, resulting in fatal "extrapolation into the future" errors inside Nav2 costmaps.
- **Solution:** Refactored launch architecture to enforce strict ownership: `robot_state_publisher` is owned solely by `hardware_bringup.launch.py`. Redundant instances in navigation launches were eliminated, preserving single-source transform integrity.

### 3. Edge Compute Resource Contention & Lifecycle Bond Timeouts
- **Challenge:** Initializing Nav2 costmaps, AMCL, planners, controllers, and RViz simultaneously on the NVIDIA Jetson Orin caused momentary CPU spikes that exceeded Nav2 lifecycle manager bond timeouts, causing nodes to crash during startup.
- **Solution:** Designed a **staged initialization sequence** using ROS 2 `TimerAction` primitives ($t=0\text{s}$ bringup, $t=5\text{s}$ localization, $t=6\text{s}$ controllers, $t=7\text{s}$ recoveries, $t=8\text{s}$ behavior tree, $t=10\text{s}$ navigation lifecycle). This eliminated startup race conditions entirely.

### 4. Controller Deadband Misconfiguration for Differential Kinematics
- **Challenge:** Default controller parameters configured a lateral velocity deadband of `min_y_velocity_threshold: 0.5`. On a non-holonomic differential robot that cannot move laterally ($v_y \equiv 0$), this masked controller state evaluation bugs.
- **Solution:** Corrected `min_y_velocity_threshold: 0.001` in `nav2_controller.yaml` to match forward motion thresholds, and capped forward velocity to $0.22\text{ m/s}$ to ensure dish stability.

---

## Testing and Evaluation

### Verified Experimental Mapping Artifacts
The repository includes real occupancy grid maps generated by operating the physical robot platform with `slam_toolbox`:
- **`map_1777558867.pgm` (39.0 KB):** Demonstrates real indoor structural mapping with closed boundary perimeters, doorway passages, and furniture clusters at $0.05\text{ m/cell}$ resolution.
- **`my_map.pgm` (41.4 KB):** High-density occupancy scan exhibiting crisp, non-duplicated wall profiles, proving effective loop closure and low odometric drift.

### Static Code Verification & Non-Destructive Quality Assurance
- **Python AST Syntax Validation:** All nodes (`navigator.py`, `goal_publisher.py`, launch scripts) compile cleanly with Python 3.10+.
- **ROS 2 XML/YAML Validation:** `package.xml`, URDF descriptions, and parameter manifests conform to official ROS 2 schemas.
- **Credential & Secret Audit:** Complete filesystem scan confirms zero credentials, private SSH keys, passwords, or machine-specific tokens are present.

### Proposed Quantitative Evaluation Framework
For formal academic benchmarking in a dedicated indoor research test arena, the following **proposed experimental protocol** is defined:

| Metric | Target Research Value | Measurement Method |
| :--- | :--- | :--- |
| **Global Path Compute Latency** | $< 15\text{ ms}$ | Benchmarked across 100 A\* iterations on Jetson Orin |
| **Cross-Track Trajectory Error** | $\text{RMSE} < 0.08\text{ m}$ | Deviation between planned A\* path and executed odometry |
| **Table Docking Arrival Precision**| $\Delta xy \le 0.15\text{ m}, \Delta\theta \le 5^\circ$ | Physical error relative to indoor table markers |
| **Dynamic Obstacle Reaction Time**| $< 100\text{ ms}$ (20 Hz cycle) | Latency between obstacle appearance and deceleration initiation |
| **Edge Compute Resource Footprint**| CPU Core Avg $< 45\%$, RAM $< 2.2\text{ GB}$ | Profiling via `tegrastats` on NVIDIA Jetson Orin |
| **Service Mission Success Rate** | $> 95\%$ across 50 trials | Percentage of completed delivery cycles without manual intervention |

---

## Research Relevance & CCU Internship Alignment

This physical robotics platform directly demonstrates practical research foundations aligned with three preferred research internship topics at the **College of Engineering, National Chung Cheng University (CCU), Taiwan (2027)**:

```
+-----------------------------------------------------------------------------------+
|               NATIONAL CHUNG CHENG UNIVERSITY (CCU), TAIWAN                      |
|                  College of Engineering Research Alignment                        |
+-----------------------------------------------------------------------------------+
       |                                  |                                  |
       v                                  v                                  v
+-----------------------+      +-----------------------+      +-----------------------+
|  PROJECT P6           |      |  PROJECT P7           |      |  PROJECT P8           |
|  LiDAR & Camera       |      |  Talk to Your Drone / |      |  Computer Vision &    |
|  Sensor Fusion        |      |  AI Autonomous LLMs   |      |  Deep Learning        |
|  Prof. Jui-Chiu Chiang|      |  Assoc.Prof Chao-Yang |      |  Prof. Wen-Nung Lie   |
+-----------------------+      +-----------------------+      +-----------------------+
| • Extrinsic spatial   |      | • Programmatic goal   |      | • Edge AI deployment  |
|   transforms (TF2)    |      |   APIs (navigator.py) |      |   on Jetson Orin      |
| • 2D/3D projection    |      | • LLM tool-calling &  |      | • Real-time obstacle  |
|   equations:          |      |   task decomposition  |      |   detection & patron  |
|   s[u,v,1]^T = K[R|T]P|      | • Closed-loop error   |      |   classification      |
| • Dense depth from    |      |   handling & recovery |      | • Visual docking and  |
|   sparse LiDAR scans  |      |   behaviors           |      |   handover alignment  |
+-----------------------+      +-----------------------+      +-----------------------+
```

1. **P6 — LiDAR and Camera Fusion for Autonomous Driving (Prof. Jui-Chiu Chiang):**
   ORBIT already instantiates an extrinsic transformation matrix in its launch files between the physical chassis and a forward camera frame (`static_tf_base_link_to_camera`). The mathematical pipeline implemented for LiDAR range filtering directly generalizes to sparse LiDAR depth completion and camera-LiDAR spatial co-registration for autonomous driving.
2. **P7 — Talk to Your Drone: Building an AI Flight System with ChatGPT (Assoc. Prof. Chao-Yang Lee):**
   ORBIT's decoupled action client (`navigator.py`) and parameter-driven goal injector (`goal_publisher.py`) provide the exact programmatic abstraction required for LLM agent integration. Natural language prompts ("Take tray to Table 4") can be parsed into structured function-calling parameters that dispatch directly to ORBIT's Nav2 action server.
3. **P8 — Computer Vision Applications Based on Deep Learning Techniques (Prof. Wen-Nung Lie):**
   Deploying ROS 2 on the NVIDIA Jetson Orin provides direct hands-on mastery of embedded AI acceleration (TensorRT, CUDA, unified memory). Integrating an RGB camera onto the existing `camera` TF mount enables deploying lightweight CNNs (YOLOv8-Nano) for patron classification and dining table detection.

*For complete mathematical derivations and research mapping, see [docs/RESEARCH.md](docs/RESEARCH.md).*

---

## Future Improvements

- [ ] **LiDAR-Camera Multimodal Fusion:** Integrate an Intel RealSense RGB-D sensor to project semantic bounding boxes into the Nav2 costmap.
- [ ] **Conversational LLM Voice Interface:** Deploy a quantized on-device speech-to-text and LLM pipeline to allow patrons and kitchen staff to dispatch food delivery tasks verbally.
- [ ] **Dynamic Trajectory Optimization:** Benchmark TEB (Timed Elastic Band) local planner against DWB to compare clearance metrics in narrow corridor environments.
- [ ] **Autonomous Docking System:** Implement an ArUco/AprilTag optical docking detector for automated wireless recharging at the home base.

---

## Maintainer

**Kanishkar B**  
Department of Artificial Intelligence & Machine Learning  
Sri Eshwar College of Engineering, India  
- **GitHub:** [@Kanishkar843](https://github.com/Kanishkar843)  
- **Target Application:** 2027 College of Engineering International Research Internship, National Chung Cheng University (CCU), Taiwan.

---

<div align="center">
  <sub><strong>ORBIT</strong> · Physical Edge Robotics & Autonomous Navigation · Engineered with Precision and Academic Rigor.</sub>
</div>
