# Phase 2 — Localization Status & Reference

Companion to [`plan.md`](plan.md), [`phase1-status.md`](phase1-status.md),
[`hardware.md`](hardware.md), and [`architecture.md`](architecture.md).

Phase 2 = localize on the saved map using `nav2_amcl`. The robot knows the
room; now it needs to know **where it is in the room** at runtime.

> **STATUS: Not yet started.** This document is the pre-session plan.
> Fill in the session log (§10) as you run sessions.

---

## 1. Purpose and scope

**Goal.** Bring up `nav2_amcl` + `nav2_map_server` + `nav2_lifecycle_manager`
on the Jetson, set an initial pose in RViz, drive the robot, and watch the
particle cloud converge to a tight cluster.

**Why this phase matters (from plan.md).**
- This is your first hands-on encounter with **lifecycle nodes** — one of the
  more interesting design patterns in ROS 2. Every Nav2 component uses it.
- A working `map → odom` TF from AMCL is the prerequisite for Phase 3 (Nav2
  path planning) — the planner needs to know where the robot is in the map.

**What changes vs Phase 1.**
- The host Docker **slam container is no longer needed**. SLAM is done.
- AMCL runs on the **Jetson** (not the host) — it's ~10× lighter than
  slam_toolbox mapping mode. The 8 GB Jetson handles it fine alongside bringup
  and lidar.
- The host runs **RViz only** (same Humble Docker `rviz` container as Phase 1).
- The map is **static** — `map_server` loads it once and never changes it.
  The robot's live sensor data goes into Nav2's costmap layers (Phase 3), not
  the static map.

**Scope.** One room, `living_room_v1`. Single initial pose set by hand in
RViz. No automatic relocalization, no multi-map. Those are Phase 7 items.

---

## 2. Acceptance criteria

Phase 2 is done when ALL of these are true:

- [ ] `map_server`, `amcl`, and `lifecycle_manager` start cleanly on the
      Jetson with no `WARN`/`ERROR` about missing topics or TF.
- [ ] After clicking "2D Pose Estimate" in RViz at the robot's actual
      position, the `/particle_cloud` (PoseArray) appears as a spread of
      arrows centered on the estimate.
- [ ] After driving 2–3 m the particle cloud converges to a tight cluster
      (≤ ~30 cm spread) and **stays** tight during further driving.
- [ ] `ros2 run tf2_ros tf2_echo map odom` (on Jetson) shows a stable,
      sensible transform — not wildly drifting.
- [ ] `ros2 run tf2_tools view_frames` (on Jetson) shows the complete chain
      `map → odom → base_footprint → base_link → laser_link`.
- [ ] The robot can be driven to several points in the room and AMCL keeps
      up — no "lost" state (particle cloud spreading back out).

Stretch (nice-to-have):
- AMCL auto-recovers if you pick it up and put it down elsewhere (kidnapped
  robot problem). This requires `recovery_alpha_fast` > 0.
- Save the initial pose to the params file so the next session starts near
  the correct location without a manual "2D Pose Estimate" click.

---

## 3. What we need to create

Two new files in `cat_patrol_robot`, and one convenience script:

| File | Purpose |
|---|---|
| `config/amcl_params.yaml` | AMCL + map_server + lifecycle_manager params tuned for this robot |
| `launch/localization.launch.py` | Brings up map_server + amcl + lifecycle_manager in one command |
| `~/myscripts2/t3_amcl.sh` | Convenience script (domain-28 + cyclonedds env, mirrors t1/t2/t2.5) |

These are created in §4 below. The CMakeLists already installs `config/` and
`launch/` into the ament share directory, so no CMakeLists changes needed.

---

## 4. Files to create (do this before the first session)

### 4a. `config/amcl_params.yaml`

Key differences from the existing `yahboomcar_nav/params/dwa_nav_params.yaml`
AMCL section:

| Param | Old value | Our value | Reason |
|---|---|---|---|
| `scan_topic` | `/scan` (default) | `/scan_filtered` | Only the filtered forward arc is trustworthy on this robot |
| `robot_model_type` | `DifferentialMotionModel` | `OmnidirectionalMotionModel` | X3 is mecanum — can strafe; diff model under-represents possible motions |
| `laser_min_range` | `-1.0` | `0.15` | Matches our SLAM filter; avoids chassis near-field hits |
| `laser_max_range` | `100.0` | `8.0` | RPLiDAR A1 reliable range; avoid bogus long returns |

Create this file:

```bash
cat > /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/config/amcl_params.yaml << 'EOF'
amcl:
  ros__parameters:
    use_sim_time: false

    # --- Motion model ---
    # OmnidirectionalMotionModel because X3 is mecanum (can strafe).
    robot_model_type: "nav2_amcl::OmnidirectionalMotionModel"
    # alpha1-5: noise coefficients (rotation-from-rotation, rotation-from-translation,
    # translation-from-translation, translation-from-rotation, strafe-from-translation).
    # Start conservative (0.2); tighten if the cloud is too spread after a straight drive.
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2

    # --- Laser model ---
    scan_topic: /scan_filtered
    laser_model_type: "likelihood_field"
    laser_min_range: 0.15
    laser_max_range: 8.0
    max_beams: 60
    laser_likelihood_max_dist: 2.0

    # Beam weights (must sum to 1.0).
    # z_hit: fraction explained by expected map wall.
    # z_rand: random / unexplained returns (cats, glass, virtual walls we painted).
    # z_max: beam went beyond max range (sensor missed).
    # z_short: shorter than expected (moving obstacle in the way).
    # Raise z_rand slightly vs defaults because our painted virtual walls
    # produce real "through-wall" laser returns that don't match the map.
    z_hit: 0.5
    z_rand: 0.4
    z_max: 0.05
    z_short: 0.05
    sigma_hit: 0.2
    lambda_short: 0.1

    # --- Particle filter ---
    min_particles: 500
    max_particles: 2000
    pf_err: 0.05
    pf_z: 0.99
    resample_interval: 1

    # --- Update thresholds ---
    # Only update the filter after the robot has moved this much.
    update_min_d: 0.25     # metres
    update_min_a: 0.2      # radians (~11°)

    # --- Frame ids (must match our TF tree) ---
    global_frame_id: "map"
    odom_frame_id: "odom"
    base_frame_id: "base_footprint"
    tf_broadcast: true     # AMCL publishes map → odom

    # --- Recovery ---
    # Both 0.0 = no random particle injection (safe start).
    # Set recovery_alpha_fast: 0.1 if the robot gets "lost" too easily.
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0

    # --- Beam skip (filter out moving obstacles in scan) ---
    do_beamskip: false
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3

    transform_tolerance: 1.0
    save_pose_rate: 0.5

map_server:
  ros__parameters:
    use_sim_time: false
    yaml_filename: "/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/maps/living_room_v1.yaml"
    topic_name: "map"
    frame_id: "map"

lifecycle_manager_localization:
  ros__parameters:
    use_sim_time: false
    autostart: true
    node_names: ["map_server", "amcl"]
EOF
```

### 4b. `launch/localization.launch.py`

```bash
cat > /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/launch/localization.launch.py << 'EOF'
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/config/amcl_params.yaml',
        description='Full path to AMCL/map_server params YAML',
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_file_arg,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
    ])
EOF
```

### 4c. `~/myscripts2/t3_amcl.sh`

```bash
cat > ~/myscripts2/t3_amcl.sh << 'EOF'
#!/usr/bin/env bash
export ROS_DOMAIN_ID=28
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds.xml
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch cat_patrol_robot localization.launch.py
EOF
chmod +x ~/myscripts2/t3_amcl.sh
```

### 4d. Build

After creating the files above:

```bash
export ROS_DOMAIN_ID=28
source /opt/ros/humble/setup.bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws
colcon build --symlink-install --packages-select cat_patrol_robot
source install/setup.bash
```

---

## 5. Architecture

```
Jetson (Humble)                          Host peter-pen (Humble Docker)
────────────────────────────────         ────────────────────────────────
t1.sh   yahboomcar_bringup              humble-docker-rviz-1
        ├── base_node (MCU)                 rviz2
        ├── imu_filter                         Map       /map  [Transient Local]
        ├── ekf_filter  → /odom               PoseArray /particle_cloud
        ├── robot_state_publisher             LaserScan /scan_filtered
        └── yahboom_joy_X3                    Pose      /amcl_pose
                                              TF        (enable for debugging)
t2.sh   sllidar_ros2  → /scan
                                         (slam Docker NOT needed in Phase 2)
t2.5.sh scan_front_filter
        /scan → /scan_filtered

t3_amcl.sh   localization.launch.py
        ├── map_server   → /map [Transient Local]
        ├── amcl         → /particle_cloud, /amcl_pose
        │                  map → odom TF
        └── lifecycle_manager (autostart: configures + activates both)

TF chain (same structure as Phase 1, different publisher for map→odom):
  map → odom          AMCL (was slam_toolbox in Phase 1)
  odom → base_footprint   EKF (same as Phase 1)
  base_footprint → ...    robot_state_publisher (same as Phase 1)
```

---

## 6. Pre-flight checklist

### 6a. One-time (before first session)

- [ ] Create the three files in §4 and run the colcon build.
- [ ] Verify build: `ros2 pkg executables cat_patrol_robot | grep -v patrol_node`
      should show nothing new (localization.launch.py is a launch file, not
      an executable, but build must succeed cleanly).
- [ ] Verify launch file is reachable:
      `ros2 launch cat_patrol_robot localization.launch.py --show-args`
- [ ] Stop the host slam Docker container (no longer needed):
      On host: `cd ~/humble-docker && sudo docker compose stop slam`
- [ ] Ensure host RViz Docker is available:
      On host: `cd ~/humble-docker && sudo docker compose up -d rviz`
      (or `docker compose start rviz` if already created).

### 6b. Every session

- [ ] Robot at the tape mark (starting position matters — you'll click the
      initial pose in RViz to match it).
- [ ] Battery > 11.5 V: `ros2 topic echo /voltage --once`
- [ ] Reboot Jetson if swap > 512 MiB: `free -h`
- [ ] `sudo systemctl stop ollama` (optional, frees ~100 MB)
- [ ] Verify USB ports (chassis = CH340 7523, lidar = CP2102 by-id symlink).

---

## 7. Session procedure

### Step A — Jetson terminals (in order)

**Every terminal — paste first:**
```bash
export ROS_DOMAIN_ID=28
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds.xml
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
```

Or use the scripts in `~/myscripts2/`:

**T1 — bringup:**
```bash
bash ~/myscripts2/t1.sh
```
Wait for: `cmd_vel trim: trim_vy_per_vx=0.0120`

**T2 — lidar:**
```bash
bash ~/myscripts2/t2.sh
```
Wait for: `SLLidar health status : OK`

**T2b — scan filter:**
```bash
bash ~/myscripts2/t2.5.sh
```
Wait for: `Keeping scan angles` log line.

**T3 — localization (new for Phase 2):**
```bash
bash ~/myscripts2/t3_amcl.sh
```

Success signs from T3:
- `[lifecycle_manager_localization] Configuring map_server`
- `[lifecycle_manager_localization] Configuring amcl`
- `[lifecycle_manager_localization] Activating map_server`
- `[lifecycle_manager_localization] Activating amcl`
- `[amcl] Received a 290 X 141 map` (or similar size)
- `[lifecycle_manager_localization] Managed nodes are active`

**No `exit code -9`** — if AMCL OOMs (unlikely, it's light), see §8.

Check memory after T3 is active:
```bash
free -h
# expect ≥ 3 GiB available (AMCL is much lighter than slam_toolbox)
```

### Step B — Host RViz

On **host** (after T3 is stable):

```bash
export ROS_DOMAIN_ID=28
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/bots/cyclonedds.xml
export ROS_LOCALHOST_ONLY=0
# inside the rviz Docker container, or directly if Jazzy RViz is OK for display:
sudo docker exec -it humble-docker-rviz-1 bash
# then inside:
export DISPLAY=:0
ros2 run rviz2 rviz2
```

**RViz displays for Phase 2:**

| Display | Topic | Notes |
|---|---|---|
| Fixed Frame | `map` | |
| Map | `/map` | **Durability: Transient Local** |
| LaserScan | `/scan_filtered` | |
| PoseArray | `/particle_cloud` | The AMCL particle cloud — set color to distinctive |
| Pose (arrow) | `/amcl_pose` | The best-estimate pose |
| TF | optional | Enable only to debug; costs Jetson RAM |

### Step C — Set initial pose

1. In RViz click **"2D Pose Estimate"** (toolbar at top).
2. Click on the map at the robot's **actual physical location** (where it is
   on the tape mark).
3. Drag the arrow to set the heading (which way the robot faces).
4. Release — you should see a spread of arrows (the particle cloud) appear
   centered on your estimate.

If no particle cloud appears:
- Check `/particle_cloud` topic exists: `ros2 topic list | grep particle`
- Check AMCL is active: `ros2 lifecycle get /amcl`

### Step D — Drive and verify convergence

- Drive slowly using the joystick (~0.10 m/s).
- After 1–2 m of driving, the particle cloud should tighten significantly.
- After a turn, it tightens further (rotation is very informative for AMCL).
- **Convergence test:** drive a small loop and return to start. The `/amcl_pose`
  arrow should be close to where you actually are.

### Step E — Verify TF chain

On Jetson (new terminal, sourced):
```bash
ros2 run tf2_ros tf2_echo map odom
# expect: steady transform, not wildly jumping

ros2 run tf2_tools view_frames
# produces /tmp/frames_XXXX.pdf — copy to host to view
```

---

## 8. Pitfalls and recovery

| Symptom | Cause | Fix |
|---|---|---|
| `[lifecycle_manager] Failed to configure map_server` | yaml_filename wrong or map file missing | Check path: `ls /home/jetson/yahboomcar_ros2_ws/.../maps/living_room_v1.*`; verify absolute path in amcl_params.yaml |
| `[amcl] Received map with 0 cells` | Map loaded but empty; yaml origin/size issue | Check living_room_v1.yaml has the absolute image path (not relative) |
| `[amcl] WARN: Failed to transform laser scan` | TF not ready; bringup not up, or AMCL started before bringup | Restart T3 after T1/T2/T2b are all healthy |
| No `/particle_cloud` topic in RViz | AMCL not active (lifecycle not reached Active state) | Check T3 for `Managed nodes are active`; `ros2 lifecycle get /amcl` |
| Particle cloud doesn't converge after driving | Wrong initial pose; wrong motion model; scan_topic still `/scan` not `/scan_filtered` | Re-click 2D Pose Estimate at the correct location; verify `scan_topic: /scan_filtered` in params |
| Particle cloud diverges / robot gets "lost" | Driving too fast; localization can't keep up; or hit a dynamic obstacle | Slow down; stop; re-click 2D Pose Estimate |
| `map → odom` TF jumpy | AMCL struggling; too many dynamic obstacles in scan | Reduce `max_beams` to 30; enable `do_beamskip: true` |
| AMCL exits -9 | OOM (unlikely but possible with large particle counts) | Reduce `max_particles` to 1000; `sudo systemctl stop ollama` |
| `[amcl] WARN: Cannot call set_parameters on node` | lifecycle not yet in configure state | Wait 5 s; check lifecycle_manager log for errors |
| `/map` in RViz shows "no map received" | Durability not Transient Local; or map_server not active | Set Map display Durability to **Transient Local**; check T3 log |
| Robot model in wrong position in RViz | Wrong initial pose estimate | Re-click 2D Pose Estimate |
| Virtual walls (painted in GIMP) cause AMCL to drift | Laser sees through painted walls → beam weights wrong | Slightly raise `z_rand` (to 0.5) and lower `z_hit` (to 0.4); painted walls are a small fraction of the scan |

---

## 9. Concept anchors (lifecycle nodes — the Phase 2 learning goal)

From plan.md: *"This is your first encounter with lifecycle nodes — nav2_amcl is
a managed node, and Nav2 itself is built on the same pattern."*

The pattern:

```
Unconfigured  →  Configured  →  Active  →  Deactivated  →  Finalized
     ↑                ↑             ↑
  on_configure()  on_activate()  on_deactivate()
```

- `on_configure()`: allocate resources, load params, subscribe to topics — but
  don't start publishing yet.
- `on_activate()`: start publishing, start timers. The node is now "running."
- `on_deactivate()`: stop publishing but keep resources — can re-activate fast.
- `on_cleanup()`: free resources, return to Unconfigured.

`lifecycle_manager` with `autostart: true` drives this automatically at
startup: it calls configure then activate on each node in `node_names` in
order. That's why you see the sequence in T3's log.

**Where to read the code:**
- `rclcpp_lifecycle/include/rclcpp_lifecycle/lifecycle_node.hpp` — the base
  class interface. Look at the virtual `on_*` signatures.
- `nav2_amcl/src/amcl_node.cpp` — a real production lifecycle node. Find the
  `on_configure`, `on_activate`, `on_deactivate` methods and read what they
  do at each state.
- `nav2_lifecycle_manager/src/lifecycle_manager.cpp` — the manager itself.
  How it issues `change_state` service calls to each managed node.

---

## 10. Commands cheat-sheet

**Every Jetson terminal:**
```bash
export ROS_DOMAIN_ID=28
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds.xml
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
```

**Start stack (Jetson):**
```bash
bash ~/myscripts2/t1.sh       # T1: bringup
bash ~/myscripts2/t2.sh       # T2: lidar
bash ~/myscripts2/t2.5.sh     # T2b: scan filter
bash ~/myscripts2/t3_amcl.sh  # T3: localization
```

**Verify AMCL is active:**
```bash
ros2 lifecycle get /amcl
ros2 lifecycle get /map_server
```

**Check TF chain:**
```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_tools view_frames
```

**Inspect topics:**
```bash
ros2 topic hz /particle_cloud
ros2 topic echo /amcl_pose --once
ros2 topic hz /scan_filtered
```

**Manually trigger AMCL update (if it seems stuck):**
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}'
```
(Better: use RViz "2D Pose Estimate" button instead.)

**If AMCL loses localization — reset without restarting:**
Click "2D Pose Estimate" in RViz again at the robot's current position.

**Stop the slam Docker on host (one-time):**
```bash
cd ~/humble-docker && sudo docker compose stop slam
```

---

## 11. Phase 2 session log

### Session 1 — _date_

- Stack versions / build SHA: _(colcon build completed on?)_
- Battery at start / end:
- Particle cloud appeared after pose estimate: yes/no
- Convergence: tight after ___ m of driving
- `map → odom` TF: stable / jumpy
- Anything unusual:
- Outcome:

### Session 2 — _date_

_(copy template above)_

### Final results

- AMCL params that worked: alpha1-5 = ___, max_particles = ___
- Convergence time from cold estimate: ___ seconds / ___ metres driven
- Localization stable during full room drive: yes/no
- Phase 2 acceptance criteria: all met / blockers: ___

---

## 12. Open questions / forward links

- **Phase 3 (Nav2 full stack).** Phase 2's `map_server` + `amcl` +
  `lifecycle_manager` become the localization half of Phase 3. Phase 3 adds
  the planner, controller, costmaps, behavior server, and BT navigator — all
  as additional lifecycle nodes managed by the same (or an extended)
  `lifecycle_manager`.
- **AMCL vs slam_toolbox localization mode.** Phase 1 notes mention
  slam_toolbox has a localization-only mode that can use the `.posegraph`
  files saved alongside the `.pgm`. We chose nav2_amcl per the plan — but
  slam_toolbox localization is worth knowing about as an alternative.
- **Initial pose automation.** After Phase 2 is tuned, `set_initial_pose: true`
  + `initial_pose: {x, y, yaw}` in the params file can be used to seed AMCL
  at the tape-mark position automatically. Useful once the robot always starts
  at the same spot.
- **Kidnapped robot.** If `recovery_alpha_fast: 0.1` is set, AMCL injects
  random particles when the filter's total weight drops below a threshold,
  allowing recovery from being picked up and moved. Useful for a cat-patrol
  robot that might get nudged by a cat.
