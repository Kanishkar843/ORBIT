# ORBIT System Architecture

This document provides a detailed breakdown of the hardware interfaces, coordinate transformation trees, sensor processing pipelines, Nav2 navigation stack integration, and software lifecycle execution for the **ORBIT (LiDAR-Based Autonomous Food-Serving Robot)** platform.

---

## 1. High-Level System Architecture

The ORBIT platform executes an autonomous service workflow designed for indoor dining and food delivery. The robot integrates real hardware sensing, SLAM, probabilistic localization, global path planning, local obstacle avoidance, and dual-channel brushless DC motor control on an onboard NVIDIA Jetson Orin edge computer.

```mermaid
flowchart TD
    subgraph SENSING["Sensory Perception Layer"]
        LIDAR["RPLidar C1 (2D LiDAR)\n/dev/ttyUSB0 (12m range, 360° FOV)"]
        SCAN["Raw LaserScan (/scan)"]
        FILTER["laser_filters (scan_to_scan_filter_chain)\nRear Blind Spot Mask [-120° to +120°]"]
        SCAN_FILT["Filtered Scan (/scan_filtered)"]
        CAM["Forward RGB/Depth Camera (Proposed Frame)\nstatic_tf: base_link → camera"]
    end

    subgraph HARDWARE_ACTUATION["Actuation & Kinematics Layer"]
        MOTORS["ZLAC8015D Dual BLDC Motor Driver\n/dev/ttyACM0 (RS485/Serial)"]
        DIFF["Differential Drive Kinematics\n2x Drive Wheels + Rear Caster"]
        ODOM_PUB["Odometry Publisher & TF\nodom → base_footprint"]
    end

    subgraph SLAM_LOCALIZATION["Spatial Estimation Layer"]
        SLAM["slam_toolbox (Online Async)\nCeres Solver (Sparse Normal Cholesky)"]
        MAP_SRV["nav2_map_server\nOccupancy Grid Map (/map)"]
        AMCL["nav2_amcl (Adaptive Monte Carlo)\nLikelihood Field + Differential Motion Model"]
        TF_MAP["map → odom TF Broadcast"]
    end

    subgraph NAVIGATION_STACK["Nav2 Navigation Core"]
        BT["nav2_bt_navigator\nBehavior Tree XML Execution"]
        GLOBAL["nav2_planner (NavfnPlanner)\nA* Global Path Planning"]
        G_COST["Global Costmap\nStatic + Obstacle + Inflation Layers"]
        LOCAL["nav2_controller (DWBLocalPlanner)\nTrajectory Rollouts (20 Hz)"]
        L_COST["Local Costmap\nRolling Window + Inflation (r=0.65m)"]
        RECOVERY["nav2_behaviors\nSpin, BackUp, Wait"]
    end

    subgraph APPLICATION_LAYER["Service Application Layer"]
        NAV_NODE["navigator.py\nMulti-Goal Sequence Action Client"]
        GOAL_CLI["goal_publisher.py\nSingle Goal CLI Injector"]
        TELEOP["teleop_twist_keyboard\nManual Teleoperation Control"]
    end

    %% Flow connections
    LIDAR --> SCAN --> FILTER --> SCAN_FILT
    SCAN_FILT --> SLAM
    SCAN_FILT --> AMCL
    SCAN_FILT --> G_COST
    SCAN_FILT --> L_COST

    SLAM -.->|Phase 1 Mapping| MAP_SRV
    MAP_SRV --> AMCL
    AMCL --> TF_MAP
    ODOM_PUB --> NAVIGATION_STACK

    NAV_NODE -->|NavigateToPose Action| BT
    GOAL_CLI -->|/goal_pose| BT
    BT --> GLOBAL --> G_COST
    BT --> LOCAL --> L_COST
    BT --> RECOVERY
    LOCAL -->|/cmd_vel| MOTORS
    TELEOP -->|/cmd_vel| MOTORS
    MOTORS --> DIFF --> ODOM_PUB
```

---

## 2. Coordinate Transformation (TF) Tree

All coordinate transformations adhere strictly to the **ROS REP-103** (Standard Units of Measure & Coordinate Conventions) and **REP-105** (Coordinate Frames for Mobile Platforms) standards.

```mermaid
graph TD
    map["map (Global Fixed World Frame)"] -->|AMCL / SLAM Toolbox| odom["odom (Odometry World Frame)"]
    odom -->|zlac8015d_driver| base_footprint["base_footprint (2D Ground Projection)"]
    base_footprint -->|diffbot.urdf / static_tf (z=0.10m)| base_link["base_link (Chassis Origin)"]
    
    base_link -->|diffbot.urdf (x=0.0, y=0.15, z=0.0)| left_wheel["left_wheel (Drive Wheel)"]
    base_link -->|diffbot.urdf (x=0.0, y=-0.15, z=0.0)| right_wheel["right_wheel (Drive Wheel)"]
    base_link -->|diffbot.urdf / static_tf (x=-0.15, y=0.0, z=-0.05)| caster_wheel["caster_wheel (Support Caster)"]
    base_link -->|diffbot.urdf / static_tf (x=0.0, y=0.0, z=0.18)| lidar["lidar (RPLidar C1 Optical Center)"]
    base_link -.->|static_tf (x=0.10, y=0.0, z=0.15) [Configured]| camera["camera (Forward Optical Frame)"]

    classDef frame fill:#1e293b,stroke:#3b82f6,stroke-width:2px,color:#f8fafc;
    classDef planned fill:#0f172a,stroke:#eab308,stroke-width:2px,stroke-dasharray: 5 5,color:#fbbf24;
    class map,odom,base_footprint,base_link,left_wheel,right_wheel,caster_wheel,lidar frame;
    class camera planned;
```

### Frame Responsibilities and Spatial Offsets

| Frame ID | Parent Frame | Broadcast Authority | Transform (x, y, z, roll, pitch, yaw) | Purpose |
| :--- | :--- | :--- | :--- | :--- |
| `map` | *Global Origin* | `nav2_amcl` or `slam_toolbox` | Dynamic (based on scan-to-map registration) | Fixed inertial reference frame accounting for odometric drift. |
| `odom` | `map` | `zlac8015d_driver` (hardware bringup) | Dynamic (integrated wheel encoder odometry) | Continuous smooth local reference frame; accumulates drift over time. |
| `base_footprint` | `odom` | `zlac8015d_driver` | `(0, 0, 0, 0, 0, 0)` | 2D planar projection of the robot footprint on the ground plane. |
| `base_link` | `base_footprint` | `diffbot.urdf` (`robot_state_publisher`) / fallback static TF | `(0.00, 0.00, 0.10 m, 0, 0, 0)` | Geometric structural centroid of the physical robot chassis. |
| `lidar` | `base_link` | `diffbot.urdf` / fallback static TF | `(0.00, 0.00, 0.18 m, 0, 0, 0)` | Optical measurement center of the RPLidar C1 range scanner. |
| `left_wheel` | `base_link` | `diffbot.urdf` | `(0.00, +0.15, 0.00 m, -π/2, 0, 0)` | Left differential drive traction hub. |
| `right_wheel` | `base_link` | `diffbot.urdf` | `(0.00, -0.15, 0.00 m, -π/2, 0, 0)` | Right differential drive traction hub. |
| `caster_wheel` | `base_link` | `diffbot.urdf` / fallback static TF | `(-0.15, 0.00, -0.05 m, 0, 0, 0)` | Rear passive caster for tri-point planar stability. |
| `camera` *(Configured)* | `base_link` | Static transform publisher (`mapping_real.launch.py`) | `(0.10, 0.00, 0.15 m, 0, 0, 0)` | Forward-facing optical sensor frame for computer vision / fusion. |

> **TF Conflict Prevention:** The ROS 2 launch architecture guarantees that `robot_state_publisher` is owned solely by `hardware_bringup.launch.py`. Redundant state publishers in mapping or navigation launches are omitted to eliminate duplicate TF publication and time-synchronization jitter ("extrapolation into future" errors).

---

## 3. Sensor Acquisition and Filtering Pipeline

The physical robot chassis introduces self-occlusion in the rear 120° wedge. The RPLidar C1 360° scan is filtered using an angular bounds filter prior to passing data to SLAM or Nav2 costmaps.

```mermaid
flowchart LR
    RAW["RPLidar C1 Scan\nTopic: /scan\nRange: 0.05 - 12.0m\nFOV: 360° (2π rad)"] --> FILTER["laser_filters::LaserScanAngularBoundsFilterInPlace\nAngular Limits:\nLower: -2.0944 rad (-120°)\nUpper: +2.0944 rad (+120°)"]
    FILTER --> CLEAN["Filtered Scan\nTopic: /scan_filtered\nActive Arc: 240° Forward Wedge\nChassis Reflections Suppressed"]
    CLEAN --> SLAM_IN["slam_toolbox\nMap Generation"]
    CLEAN --> AMCL_IN["nav2_amcl\nParticle Distribution Update"]
    CLEAN --> COST_IN["ObstacleLayer\nGlobal & Local Costmaps"]
```

### Filtering Configuration Rationale

1. **Self-Obstruction Elimination:** Without angular bounds filtering, raw laser pulses reflect off the food-serving trays and chassis structure, causing the robot to falsely detect immediate obstacles at $r < 0.22\text{ m}$.
2. **Deterministic Computational Savings:** Discarding invalid rear range points reduces the raytracing overhead in Nav2 costmap 2D clearing/marking cycles.

---

## 4. Software Lifecycle & Staggered Initialization Architecture

On embedded edge hardware such as the NVIDIA Jetson Orin, launching all SLAM and Nav2 lifecycle nodes concurrently creates severe CPU and memory bus contention, leading to lifecycle bond timeouts. ORBIT employs a **staggered timer-action startup sequence**:

```mermaid
gantt
    title ORBIT Node Startup Sequence (navigation_real.launch.py)
    dateFormat X
    axisFormat %s s
    section Bringup
    laser_filters (/scan -> /scan_filtered) :0, 1
    map_server & amcl                       :5, 6
    section Localization Lifecycle
    lifecycle_manager_localization autostart :5, 6
    section Planning & Control
    planner_server (A* Global Planner)      :5, 6
    controller_server (DWB Local Planner)   :6, 7
    rviz2 (Remote Visualizer)               :6, 7
    section Behavior Engine
    recoveries_server (Spin/Backup/Wait)    :7, 8
    bt_navigator (Behavior Tree Engine)     :8, 9
    section Navigation Lifecycle
    lifecycle_manager_navigation autostart  :10, 11
```

---

## 5. Navigation & Decision-Making Pipeline

ORBIT uses Nav2's action-oriented client-server model for food-serving navigation:

```mermaid
sequenceDiagram
    autonumber
    actor User as Service Dispatcher / Script
    participant Nav as navigator.py (Action Client)
    participant BT as nav2_bt_navigator
    participant Global as planner_server (A* Navfn)
    participant Local as controller_server (DWB)
    participant Recover as recoveries_server
    participant Motors as zlac8015d_driver

    User->>Nav: Dispatch Goal Sequence (x, y, yaw)
    Nav->>BT: Send NavigateToPose Goal
    BT->>Global: ComputePathToPose (Global Costmap)
    Global-->>BT: Global Path (A* Trajectory)
    loop Control Cycle (20 Hz)
        BT->>Local: FollowPath (Local Costmap)
        alt Path Clear
            Local->>Motors: /cmd_vel (v_x <= 0.22 m/s, w_z <= 1.0 rad/s)
        else Path Obstructed
            BT->>Recover: Trigger Recovery (Spin / BackUp / ClearCostmap)
            Recover-->>BT: Recovery Finished
            BT->>Global: Recompute Path
        end
    end
    Local-->>BT: Goal Reached (within tolerance: xy <= 0.25m)
    BT-->>Nav: Goal Succeeded
    Nav->>Nav: Advance to Next Dining Table Waypoint
```

---

## 6. Edge Computing & Remote Development Topology

```mermaid
graph LR
    subgraph WORKSTATION["Developer Workstation (Linux / macOS / Windows)"]
        SSH_CLIENT["SSH Client / Terminal"]
        RVIZ_GUI["RViz2 Visualization Client (X11 / Local ROS 2)"]
    end

    subgraph NETWORK["Local Network / Robot Wi-Fi LAN"]
        WIFI(("802.11ac / Wi-Fi LAN\nStatic IP / DHCP Reservation"))
    end

    subgraph JETSON["ORBIT Onboard Edge Computer (NVIDIA Jetson Orin)"]
        SSH_SRV["OpenSSH Server (Port 22)"]
        ROS_CORE["ROS 2 Humble Middleware (CycloneDDS / FastDDS)"]
        HARDWARE_BUS["Hardware Serial Subsystem\n/dev/ttyUSB0 (LiDAR)\n/dev/ttyACM0 (Motors)"]
    end

    SSH_CLIENT <-->|Encrypted Shell Session| WIFI
    RVIZ_GUI <-->|DDS Discovery / ROS 2 Topics| WIFI
    WIFI <--> SSH_SRV
    WIFI <--> ROS_CORE
    ROS_CORE <--> HARDWARE_BUS
```

### Communication Interfaces

- **Remote SSH Administration:** Headless operation allows developers to start, monitor, and shut down ROS 2 nodes over an authenticated network shell.
- **DDS Middleware:** Native ROS 2 discovery protocols enable telemetry, `/scan_filtered`, `/map`, `/tf`, and costmap streaming directly to workstation RViz instances.
- **Hardware Bus Isolation:** LiDAR and motor controller are attached to discrete serial USB controllers, preventing packet starvation during simultaneous high-rate scan streaming and encoder polling.
