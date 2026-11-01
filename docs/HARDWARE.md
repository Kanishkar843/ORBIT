# ORBIT Hardware Specification & Integration Guide

This document specifies the physical computing hardware, sensor integration, actuator drivers, kinematic dimensions, electrical distribution, and serial communication buses for the **ORBIT (LiDAR-Based Autonomous Food-Serving Robot)** platform.

---

## 1. Hardware Verification Status

To uphold academic integrity and engineering credibility, all hardware components are categorized according to their verification level in the active project codebase.

| Component | Specification / Model | Status in Codebase | Source Evidence |
| :--- | :--- | :--- | :--- |
| **Compute Board** | NVIDIA Jetson Orin Series | **Verified** | Remote SSH deployment target, ARM64 ROS 2 Humble build targets |
| **Primary Range Sensor** | SLAMTEC RPLidar C1 | **Verified & Integrated** | `mapping_real.launch.py`, `rplidar_ros`, `laser_filters.yaml` |
| **Motor Controller** | ZLAC8015D Dual BLDC Driver | **Verified & Integrated** | `package.xml`, `hardware_bringup.launch.py` dependency |
| **Actuators** | Dual In-Wheel BLDC Motors | **Verified Kinematics** | `diffbot.urdf`, `nav2_controller.yaml` differential configuration |
| **Chassis Platform** | Custom Differential Service Base | **Verified Kinematics** | `diffbot.urdf` coordinate origins, wheel separation, inertias |
| **Secondary Optical Sensor**| Forward Camera Mount | **Configured in TF** | `mapping_real.launch.py` (`static_tf_base_link_to_camera`) |
| **Power Distribution** | DC Battery & Regulators | **Partially Verified** | Onboard mobile platform requirements (specific Ah to document) |

---

## 2. Onboard Computing Platform: NVIDIA Jetson Orin

The robot's central processing unit is an **NVIDIA Jetson Orin** system-on-module (SoM). The platform handles real-time sensor ingestion, SLAM computation, costmap generation, path planning rollouts, and remote communications.

```
+---------------------------------------------------------------+
|                      NVIDIA JETSON ORIN                       |
|  +---------------------------------------------------------+  |
|  | Ubuntu 22.04 LTS (aarch64)  | ROS 2 Humble Hawksbill    |  |
|  +---------------------------------------------------------+  |
|  | CUDA Toolkit & TensorRT     | FastDDS / CycloneDDS      |  |
|  +---------------------------------------------------------+  |
|       | USB Port 1 (/dev/ttyUSB0)    | USB Port 2 (/dev/ttyACM0) |
+-------|------------------------------|------------------------+
        |                              |
        v                              v
  RPLidar C1 (LiDAR)          ZLAC8015D Driver (Motors)
```

### Computing System Characteristics
- **Operating System:** Ubuntu 22.04 LTS (Jammy Jellyfish) for aarch64.
- **Middleware:** Robot Operating System 2 (ROS 2 Humble).
- **Communication Protocol:** SSH (OpenSSH Server) for secure, headless wireless operations.
- **Resource Management Strategy:** The launch configuration separates localization and navigation lifecycles across a timed sequence (t=0s to t=10s) to prevent thread starvation and memory bottlenecks during cold starts on edge hardware.

---

## 3. Perception Hardware: SLAMTEC RPLidar C1

The primary perception sensor is the **SLAMTEC RPLidar C1**, a high-precision, low-profile 2D laser range scanner.

```
       Front of Robot (+X)
           \  0°  /
            \    /
  -120°      \  /       +120°
    \         \/         /
     \                  /
      \    RPLIDAR     /       <--- Active Sensor Arc (240° FOV)
       \     C1       /
========\============/=========
|                             |
|       ORBIT CHASSIS         |  <--- Chassis Occlusion Wedge (120°)
|                             |       (Masked by laser_filters)
===============================
             180°
```

### Sensor Specifications
- **Measurement Radius:** Up to 12.0 meters (`max_laser_range: 12.0` in SLAM configuration).
- **Scan Frequency:** 10 Hz nominal.
- **Native Field of View:** 360° (2π radians).
- **Effective Navigational Arc:** 240° (-2.0944 rad to +2.0944 rad) after chassis obstruction masking via `laser_filters`.
- **Operating Interface:** USB-to-UART bridge on `/dev/ttyUSB0` (baud rate 460800).
- **Coordinate Reference Frame:** `lidar`, elevated 0.18 m above `base_link`.

---

## 4. Actuation and Motor Control: ZLAC8015D Driver

Motion control is executed through the **ZLAC8015D dual-channel intelligent brushless DC motor driver**.

```
                +---------------------------------------+
                |          ZLAC8015D CONTROLLER         |
                |   - Dual-channel brushless servo      |
                |   - Closed-loop speed / position      |
                |   - Serial interface (/dev/ttyACM0)   |
                +---------------------------------------+
                       |                        |
                       v                        v
          +------------------------+  +------------------------+
          |    LEFT DRIVE WHEEL    |  |   RIGHT DRIVE WHEEL    |
          |  BLDC In-Wheel Motor   |  |  BLDC In-Wheel Motor   |
          |  Radius: 0.10 m        |  |  Radius: 0.10 m        |
          |  Position: y = +0.15 m |  |  Position: y = -0.15 m |
          +------------------------+  +------------------------+
```

### Motor Driver Features
- **Control Interface:** Bidirectional serial protocol receiving velocity commands (`cmd_vel`) and returning quadrature encoder feedback.
- **Driver Integration:** Managed by the sibling package `zlac8015d_driver/launch/hardware_bringup.launch.py`.
- **Closed-Loop Feedback:** Provides wheel odometry transformed into the `odom -> base_footprint` TF transform.
- **Safety Features:** Hardware over-current protection, thermal shutdown, and automatic motor braking upon loss of serial keepalive pulses.

---

## 5. Physical Chassis and Kinematic Model

The physical structure is modeled in `path_planning_pkg/urdf/diffbot.urdf`.

```
                        Top View
             +---------------------------+
             |                           |
             |       [  LIDAR  ]         |  x = 0.00 m, z = 0.18 m
             |       [ CAMERA  ]         |  x = 0.10 m, z = 0.15 m
             |                           |
    +--------+---------------------------+--------+
    | [LEFT] |                           | [RIGHT]|
    | WHEEL  |                           | WHEEL  |
    | (0.1m) |                           | (0.1m) |
    +--------+---------------------------+--------+
             |                           |
             |                           |  Wheel Track: 0.30 m
             |                           |  Chassis Length: 0.50 m
             |                           |  Chassis Width: 0.30 m
             |       [ CASTER  ]         |  x = -0.15 m, z = -0.05 m
             +---------------------------+
```

### Kinematic Specifications Table

| Parameter | Value | Reference File |
| :--- | :--- | :--- |
| **Kinematic Architecture** | Two-wheel differential drive + passive caster | `diffbot.urdf` |
| **Chassis Dimensions (L × W × H)**| 0.50 m × 0.30 m × 0.15 m | `diffbot.urdf` (`base_link` box) |
| **Chassis Mass** | 5.0 kg | `diffbot.urdf` inertial tag |
| **Wheel Radius ($r$)** | 0.10 m | `diffbot.urdf` cylinder geometry |
| **Wheel Track ($L$)** | 0.30 m (separation between wheel centers) | `diffbot.urdf` joint origins ($y = \pm 0.15\text{ m}$) |
| **Maximum Linear Velocity ($v_{\max}$)**| 0.22 m/s | `nav2_controller.yaml` (`max_vel_x`) |
| **Maximum Angular Velocity ($\omega_{\max}$)**| 1.00 rad/s | `nav2_controller.yaml` (`max_vel_theta`) |
| **Maximum Linear Acceleration ($a_{x}$)**| 2.50 m/s² | `nav2_controller.yaml` (`acc_lim_x`) |
| **Maximum Angular Acceleration ($\alpha_{\theta}$)**| 3.20 rad/s² | `nav2_controller.yaml` (`acc_lim_theta`) |
| **Inflation Safety Margin** | 0.65 m radius | `nav2_astar_planning.yaml` (`inflation_radius`) |

---

## 6. Serial Port Mapping & Udev Rules Configuration

To guarantee deterministic port binding on the Linux OS regardless of the USB plug order, persistent device symlinks should be configured using `udev` rules.

### Recommended Udev Configuration (`/etc/udev/rules.d/99-orbit-robot.rules`)

```bash
# RPLidar C1 (Silicon Labs CP210x or CH340 USB-UART bridge)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar", MODE="0666"

# ZLAC8015D Motor Controller (STM32 / FTDI USB-Serial interface)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="zlac_motor", MODE="0666"
```

Apply rules via:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

## 7. Power Supply and Safety Infrastructure

The onboard mobile hardware operates from an isolated DC battery supply:
1. **High-Current Drive Bus:** Supplies nominal DC voltage to the ZLAC8015D driver for high-torque wheel motor operation.
2. **Logic & Compute Bus:** Regulated 12V / 19V supply to the NVIDIA Jetson Orin edge platform.
3. **Sensor Supply:** Filtered 5V DC line powering the RPLidar C1 optical motor and transceiver.
4. **Emergency Stop (E-Stop):** Hardware inline kill switch disconnecting driver gate power while preserving Jetson compute states for post-incident diagnostics.
