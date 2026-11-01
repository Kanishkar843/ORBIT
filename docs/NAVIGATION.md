# ORBIT Navigation & Path Planning Architecture

This document details the mathematical algorithms, parameter optimizations, costmap configurations, and recovery behaviors powering the autonomous navigation system of the **ORBIT (LiDAR-Based Autonomous Food-Serving Robot)** platform.

---

## 1. Navigation Overview

ORBIT's autonomous navigation stack operates within the **ROS 2 Nav2 Humble** framework. It bridges high-level service requests ("deliver food to Table 3") with real-time hardware motion control through a hierarchical planning architecture:

```
[ Dining Goal Request (x, y, yaw) ]
                |
                v
  +---------------------------+
  |    Global Path Planner    |  --> NavfnPlanner (A* Graph Search)
  |      (Global Costmap)     |      Resolution: 0.05 m, Inflation: 0.65 m
  +---------------------------+
                |
          Global Waypoints
                |
                v
  +---------------------------+
  |    Local Motion Planner   |  --> DWBLocalPlanner (Dynamic Window)
  |      (Local Costmap)      |      Replanning Rate: 20 Hz, Sim Time: 2.5 s
  +---------------------------+
                |
          Velocity Vector
                |
                v
  +---------------------------+
  |   ZLAC8015D Motor Driver  |  --> Differential Drive Actuation
  |    (cmd_vel: v_x, w_z)    |      v_max = 0.22 m/s, w_max = 1.0 rad/s
  +---------------------------+
```

---

## 2. Global Path Planning: A* Search Algorithm

The global planner calculates an optimal, collision-free geometric trajectory across the static floor plan.

### Configuration (`nav2_astar_planning.yaml`)
```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: false
    plugins: ['GridBased']
    GridBased:
      plugin: nav2_navfn_planner/NavfnPlanner
      tolerance: 0.5
      use_astar: true
      allow_unknown: true
```

### Algorithmic Formulation
Rather than relying on standard Dijkstra flood-filling, ORBIT explicitly enables **A\* search** (`use_astar: true`). The node priority queue evaluates each grid cell $n$ using the cost function:

$$f(n) = g(n) + h(n)$$

Where:
- $g(n)$: Exact path cost accumulated from the start pose $\mathbf{x}_{\text{start}}$ to cell $n$, factoring both traversal distance and costmap inflation values.
- $h(n)$: Euclidean distance heuristic to the target dining station $\mathbf{x}_{\text{goal}}$:
  $$h(n) = \sqrt{(x_{\text{goal}} - x_n)^2 + (y_{\text{goal}} - y_n)^2}$$

Because $h(n)$ never overestimates the true geodesic distance, the heuristic is strictly **admissible and consistent**, guaranteeing minimum path length while dramatically pruning node expansions compared to Dijkstra search on embedded Jetson hardware.

---

## 3. Local Trajectory Generation: DWB Local Planner

The local controller translates the global trajectory into safe, dynamically feasible velocity commands (`cmd_vel`) at a high control frequency (**20 Hz**).

### Configuration (`nav2_controller.yaml`)
```yaml
controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.22           # Tuned for stable food-tray transport
      min_vel_y: 0.0
      max_vel_y: 0.0           # Non-holonomic differential constraint
      max_vel_theta: 1.0
      acc_lim_x: 2.5
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_theta: -3.2
      vx_samples: 25
      vy_samples: 5
      vtheta_samples: 25
      sim_time: 2.5
      linear_granularity: 0.05
      angular_granularity: 0.025
```

### Dynamic Window Approach Principles
1. **Dynamic Window Search Space:** The admissible velocity set $\Omega$ is restricted to velocities reachable within the next acceleration step $\Delta t$:
   $$\Omega = \left\{ (v, \omega) \mid v \in [v_c - a_x \Delta t, v_c + a_x \Delta t], \; \omega \in [\omega_c - \alpha_\theta \Delta t, \omega_c + \alpha_\theta \Delta t] \right\} \cap V_{\text{limits}}$$
2. **Trajectory Rollouts:** For each $(v_i, \omega_j)$ candidate, the simulator projects forward kinematics for $t_{\text{sim}} = 2.5\text{ s}$ across 25 forward velocity and 25 rotational samples.
3. **Food Service Speed Tuning:** Forward velocity is constrained to $v_{\max} = 0.22\text{ m/s}$ (reduced from 0.26 m/s) with acceleration capped at $2.5\text{ m/s}^2$ to guarantee that liquid dishes and food trays remain completely stable during emergency obstacle stops.

---

## 4. Multi-Layer Costmap Architecture

ORBIT organizes environmental representations into distinct 2D costmap layers (`nav2_costmap_2d`):

```
+-----------------------------------------------------------+
| INFLATION LAYER                                           |
| Decays cost outward: radius = 0.65 m, scaling factor = 5.0|
+-----------------------------------------------------------+
                           |
+-----------------------------------------------------------+
| OBSTACLE LAYER                                            |
| Ingests /scan_filtered (max obstacle range = 2.5 m)      |
+-----------------------------------------------------------+
                           |
+-----------------------------------------------------------+
| STATIC MAP LAYER                                          |
| Pre-mapped walls, service counters, tables (/map)         |
+-----------------------------------------------------------+
```

### Cost Function Formulation
For any cell $c$ located at distance $d$ from an obstacle within inflation radius $r_{\text{inflation}} = 0.65\text{ m}$:

$$\text{Cost}(c) = \text{LETHAL\_OBSTACLE} \cdot \exp\left( -k \cdot (d - r_{\text{inscribed}}) \right)$$

Where $k = 5.0$ (`cost_scaling_factor`) and $r_{\text{inscribed}} = 0.22\text{ m}$ (`robot_radius`). This produces a smooth repulsive potential field ensuring the robot maintains generous clearance around tables, chairs, and diners.

---

## 5. Probabilistic State Estimation: AMCL

Localization inside pre-mapped environments is performed via **Adaptive Monte Carlo Localization** (`nav2_amcl`).

### Configuration Highlights (`amcl.yaml`)
- **Particle Count:** Dynamically adapts between $N_{\min} = 500$ and $N_{\max} = 2000$ particles based on pose confidence.
- **Motion Model:** `nav2_amcl::DifferentialMotionModel` with odometry noise parameters ($\alpha_1 = 0.2, \alpha_2 = 0.2, \alpha_3 = 0.2, \alpha_4 = 0.2, \alpha_5 = 0.1$).
- **Laser Sensor Model:** Likelihood Field with 360 maximum beams, standard deviation $\sigma_{\text{hit}} = 0.2\text{ m}$.
- **Input Topic:** Remapped to `/scan_filtered` to eliminate false localization updates from the blocked chassis rear.

---

## 6. Mission Execution & Waypoint Traversal

The robot's service delivery logic is implemented in `path_planning_pkg/nodes/navigator.py`.

```python
DEFAULT_GOALS = [
    (1.0,  0.0,   0.0),    # Service station pickup
    (1.0,  1.5,  90.0),    # Table 1 delivery
    (0.0,  1.5, 180.0),    # Table 2 delivery
    (0.0,  0.0, -90.0),    # Return to home dock
]
```

### Action Client Lifecycle
1. Connects asynchronously to Nav2 action server `navigate_to_pose`.
2. Converts planar `(x, y, yaw)` goal coordinates into standard 3D quaternions:
   $$q_z = \sin\left(\frac{\text{yaw}}{2}\right), \quad q_w = \cos\left(\frac{\text{yaw}}{2}\right)$$
3. Monitors `distance_remaining` via asynchronous feedback callbacks.
4. Handles goal completion status (`STATUS_SUCCEEDED`) and immediately queues the subsequent delivery stop.

---

## 7. Recovery Behaviors

When dynamic obstacles (such as moving servers or patrons) temporarily block all global paths, the behavior tree triggers hierarchical recovery mechanisms (`nav2_behaviors`):

| Recovery Action | Plugin | Behavior |
| :--- | :--- | :--- |
| **Wait** | `nav2_behaviors/Wait` | Robot pauses for 5 seconds to let dynamic human obstacles pass. |
| **Clear Costmap** | `nav2_costmap_2d/ClearEntireCostmap` | Clears stale obstacle readings from dynamic objects. |
| **Spin** | `nav2_behaviors/Spin` | In-place rotation to refresh 240° LiDAR coverage around the robot. |
| **BackUp** | `nav2_behaviors/BackUp` | Reverses by 0.15 m along clear ground if trapped in an acute corner. |
