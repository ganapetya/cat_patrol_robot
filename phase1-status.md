# Phase 1 — Status & Reference

Companion to [`plan.md`](plan.md), [`hardware.md`](hardware.md),
[`architecture.md`](architecture.md), and [`phase0-status.md`](phase0-status.md).
This is the practical "what we plan to do for Phase 1, what we actually
did, and what we learned" document.

Phase 1 = build a 2D map of one room with `slam_toolbox`, save it to
disk, and write a single-command launch file that reproduces the SLAM
session. **Scope deliberately small** — one room only. Multi-room
mapping is deferred to Phase 7.

When this doc was written, Phase 1 had not yet started; section 11
now records Session 1 (in progress). Use this doc as the canonical
command reference — especially `/scan_filtered`, the lighter SLAM
params, and host-side RViz.

---

## 1. Purpose and scope

**Goal.** Map ONE room with `slam_toolbox` (online async mode), save the
resulting occupancy grid as a `.pgm` + `.yaml` pair, and capture the
session as a reproducible `slam.launch.py` so future sessions are a
single command.

**Why this phase exists.** SLAM is the gateway phase. Once we have a
trustworthy map, every later phase becomes possible:
- Phase 2 (AMCL) needs a saved map to localize against.
- Phase 3 (Nav2) needs a map for the global costmap.
- Phase 4 (patrol manager) needs a map to record waypoints in.
- Phase 6 (cat detection in the map frame) needs map-frame geometry.

**Scope.** One room. The user has chosen to focus on a single space
rather than the whole flat. This avoids the full-flat complications:
travelling through narrow doorways while building a single map, mecanum
behaviour transitioning between carpet and smooth floor (Phase 0 finding
in [`phase0-status.md`](phase0-status.md) section 4c #19), and the time
it takes to drive a large area at safe SLAM speeds.

**Cross-references.**
- [`plan.md`](plan.md) — Phase 1 section, the original arc.
- [`hardware.md`](hardware.md) section 7 — RPLiDAR A1 specs and topics.
- [`architecture.md`](architecture.md) — where SLAM sits in the data flow
  (the dashed-border block in the diagram, currently inactive).
- [`phase0-status.md`](phase0-status.md) — Phase 0 findings, especially
  the surface-dependence of mecanum rotation (section 4c lessons #19,
  #20) and the calibration record (section 5).

---

## 2. Acceptance criteria (the "done when…" list)

Phase 1 is done when ALL of these are true:

- [ ] A clean `.pgm` + `.yaml` map of the chosen room, committed under
      `cat_patrol_robot/maps/<room_name>.pgm` and `<room_name>.yaml`.
- [ ] A `slam.launch.py` in `cat_patrol_robot/launch/` that brings up
      bringup + lidar + `slam_toolbox` + RViz in one command, accepting
      a `room:=<name>` arg if multiple rooms get mapped later.
- [ ] The map covers the chosen room without major holes, smearing, or
      duplicated walls. "No major holes" = if I look at the saved
      `.pgm`, I can identify the room shape unambiguously.
- [ ] Loop closure has fired at least once during a mapping session
      (visible in the `slam_toolbox` console output as a log line and
      in RViz as a visible "snap" of the map).
      **Note:** our Jetson-8GB params file sets `do_loop_closing: false`
      to avoid OOM kills — loop closure is deferred until we have headroom
      or a beefier machine. A clean perimeter map without loop closure
      is acceptable for Phase 1 on this hardware.
- [ ] All sessions documented in section 11 below.

Stretch (nice-to-have, not required):
- A floor-type annotation overlay on the map (carpet vs smooth-floor
  area), in preparation for Phase 4 surface-aware patrol planning.
- Map serialization (`.posegraph` files) saved alongside the `.pgm` so
  future sessions can resume the SLAM graph rather than start fresh.

---

## 3. Pre-flight checklist

Two sub-lists: things you do once, and things you do at the start of
every mapping session.

### 3a. One-time setup

- Confirm `slam_toolbox` is installed:
      `ros2 pkg prefix slam_toolbox`
  → expect `/opt/ros/humble`. Already verified during planning.
- Confirm `nav2_map_server` is installed:
      `ros2 pkg prefix nav2_map_server`
  → expect `/opt/ros/humble`. Already verified.
- Pick the room. Note approximate dimensions on paper so you can later
  sanity-check the saved map's extent.
- Clear trip hazards along the routes you'll drive. The robot will be
  going slowly, but it can still get hung up on cables or rugs.
- Decide on a tape mark for the starting position. Put a piece of tape
  on the floor at the front-center of where the robot will sit at
  session start. This makes "did I return home?" answerable.
- Create the maps directory:
      `mkdir -p ~/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/maps`
- Build `cat_patrol_robot` once (installs `scan_front_filter`):
      `cd ~/yahboomcar_ros2_ws/yahboomcar_ws && colcon build --packages-select cat_patrol_robot --symlink-install`
- **RViz runs on the host PC (Noble + ROS Jazzy)**, not on the Jetson.
  Install `ros-jazzy-rviz2` on the host. Save your working RViz layout
  there (Displays: Map `/map` with **Transient Local** durability,
  LaserScan `/scan_filtered`, Fixed Frame `map`, Views Target Frame
  `base_footprint`).
- SLAM params for this robot live at
  [`config/mapper_params_online_async.yaml`](config/mapper_params_online_async.yaml)
  (lighter than upstream defaults — required on 8 GB Jetson).

### 3b. Every-session checklist

- Battery > ~11.5 V:
      `ros2 topic echo /voltage --once`
  Below ~11.0 V the chassis will start to misbehave under load; near
  empty the robot moves very slowly — charge before mapping.
- Trim active in bringup launch args:
      `trim_vy_per_vx:=0.012`
  (The Phase 0 calibration value. Without it the chassis will curve
  right and SLAM will work harder.)
- Robot at the tape-mark start.
- LiDAR motor spinning. If it was paused with `/stop_motor`:
      `ros2 service call /start_motor std_srvs/srv/Empty`
- All terminals freshly sourced if you've rebuilt anything since the
  last session:
      `source /opt/ros/humble/setup.bash`
      `source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash`
- A clear path from the start position to the perimeter of the room.

---

## 4. Mapping session procedure (step-by-step recipe)

A SLAM session uses **six Jetson terminals** plus **RViz on the host**.
Each Jetson stack on its own terminal so any one can be killed without
taking down the others.

**Topic chain (do not mix these up):**

| Topic | Publisher | Consumer |
|---|---|---|
| `/scan` | `sllidar_node` | `scan_front_filter` only |
| `/scan_filtered` | `scan_front_filter` | `slam_toolbox`, RViz LaserScan |
| `/map` | `slam_toolbox` | RViz Map (QoS: **Transient Local**) |

**Startup order:** T1 bringup → wait for `/driver_node` → T2 lidar →
T2b scan filter → wait ~5 s → T3 SLAM → wait ~20 s → host RViz.

### Terminal 1 — bringup (chassis + EKF + IMU + joystick)

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py \
    chassis_serial_port:=/dev/ttyUSB1 \
    use_joystick:=true \
    trim_vy_per_vx:=0.012
```

Joystick max speeds default to **0.15 m/s** linear and **0.5 rad/s** turn
(full stick = that speed). To go even slower: `joy_xspeed_limit:=0.10`.
To restore Yahboom defaults: `joy_xspeed_limit:=1.0 joy_yspeed_limit:=1.0 joy_angular_speed_limit:=5.0`.

Wait for `cmd_vel trim: trim_vy_per_vx=0.0120` log line before
continuing.

Verify (new Jetson terminal):

```bash
ros2 node list | grep driver_node
```

Joystick: driving is **enabled by default** (Start toggles **buzzer** on
this USB pad, not drive enable). Full stick ≈ **0.15 m/s** with defaults
below.

### Terminal 2 — LiDAR

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_launch.py \
    serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
    serial_baudrate:=115200 \
    frame_id:=laser_link
```

### Terminal 2b — front-half scan filter (required on this robot)

The RPLidar sits low; upper chassis structure blocks the **rear 180°**.
Those hits look like phantom walls and ruin SLAM. This node keeps only the
forward ±90° arc and publishes `/scan_filtered`.

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 run cat_patrol_robot scan_front_filter
```

In RViz set **LaserScan → Topic** to `/scan_filtered` (not `/scan`).

If front/rear look swapped, tune angles (degrees behind the robot kept out):

```bash
ros2 run cat_patrol_robot scan_front_filter --ros-args \
    -p trusted_angle_min:=1.5708 -p trusted_angle_max:=-1.5708
```

### Terminal 3 — slam_toolbox (online async, **lighter params — required**)

Always use our params file (not bare `online_async_launch.py`). It
points SLAM at `/scan_filtered`, caps memory use on the 8 GB Jetson, and
sets `do_loop_closing: false`.

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=false \
    slam_params_file:=/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/config/mapper_params_online_async.yaml
```

Wait **20 seconds** after start. You will see many `Message Filter
dropping message` lines — **normal at startup**. Success signs:

- `Registering sensor: [Custom Described Lidar]`
- Drop messages **stop** after ~20 s
- **No** `process has died … exit code -9` (OOM — close host RViz and retry)

Do **not** Ctrl+C during the first 20 s thinking it failed.

### Terminal 4 — RViz on **host PC** (not Jetson)

On **peter-pen** (Ubuntu Noble, ROS Jazzy):

```bash
export ROS_DOMAIN_ID=28
export ROS_LOCALHOST_ONLY=0
source /opt/ros/jazzy/setup.bash
ros2 run rviz2 rviz2 -d /home/bots/all/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_rviz/rviz/mapping.rviz
```

Or your saved config from the host if you exported one.

**RViz display settings (critical):**

| Display | Topic | Notes |
|---|---|---|
| **Global Options → Fixed Frame** | `map` | |
| **Views → Target Frame** | `base_footprint` | camera follows robot |
| **Map** | `/map` | **Durability: Transient Local** |
| **LaserScan** | `/scan_filtered` | half-circle forward arc |
| **TF** | (default) | RobotModel often errors on host (no mesh packages) — use TF axes |

If Map says **"no map received"**: check Durability is Transient Local;
drive until `/map` width/height > 0; confirm host sees `/map` with
`ros2 topic list --no-daemon`.

Host `ros2` CLI: append `--no-daemon` if the daemon misbehaves, e.g.
`ros2 --no-daemon topic list`.

### Terminal 5 — driving (joystick on Jetson, default)

Use the joystick plugged into the **Jetson** (not the host). Press
**Start** only for buzzer — sticks drive without an enable toggle.

Optional keyboard fallback on Jetson:

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Slow with `x` / `c` before driving.

### Reset map mid-session

Ctrl+C **Terminal 3 (SLAM) only**, restart T3 command above. For a fully
clean map also Ctrl+C T1, reposition robot at tape mark, restart T1 → T2
→ T2b → T3.

### Terminal 6 — (optional) inspect / save

Use spare terminals for `ros2 topic echo`, `map_saver_cli`, etc. (see
section 10).

### Driving rules

These are non-negotiable for a clean map:

- **Speed: 0.10–0.15 m/s linear.** Faster causes scan-matching to lag
  behind actual motion → map smearing.
- **Avoid in-place rotations on smooth floor.** Phase 0 finding: ~190 cm
  of unwanted translation per attempted in-place spin (see
  [`phase0-status.md`](phase0-status.md) section 4c #19). Use ARCING
  turns instead — keep wheels rolling forward, with small angular.z
  added.
- **In-place rotations are fine on carpet.** Mecanum rollers grip
  carpet pile, the chassis pivots cleanly. Use this when scanning a
  corner that's hard to approach via arcing.
- **Drive perimeter first.** Go around the room close to the walls
  (~50 cm offset) so the LiDAR sees them clearly. Then sweep through
  the middle.
- **Only trust the forward 180° of LiDAR.** Rear hemisphere hits the
  robot superstructure — always run `scan_front_filter` (T2b). Map the
  full room by **turning the robot** so the forward arc sees each wall.
- **Loop back to start at least once.** With `do_loop_closing: false`
  you won't get an automatic snap, but revisiting the start still helps
  scan matching refine locally.
- **If you scrape something or the chassis jolts, drive a small loop
  near where you are** before continuing. Sudden physical disruptions
  introduce localization spikes that scan-matching will smooth out
  given another nearby observation.

---

## 5. Saving the map

Once the room looks good in RViz, save the map without taking down the
SLAM session.

In a new terminal (don't use one of the existing ones):

```bash
source /opt/ros/humble/setup.bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/maps
ros2 run nav2_map_server map_saver_cli -f my_room
```

Replace `my_room` with whatever name fits (e.g. `living_room_v1`).
The trailing `-f <name>` is the **base name** of the output files; the
saver appends `.pgm` and `.yaml` extensions itself.

You'll get two files:

| File | Format | Contents |
|---|---|---|
| `<name>.pgm` | Plain grayscale PGM image | The occupancy grid as pixels: white = free, black = occupied, gray = unknown. Open in any image viewer. |
| `<name>.yaml` | YAML | Metadata: image filename, resolution (m/pixel, default 0.05), origin in world coords, free/occupied thresholds, negate flag. |

**The `.yaml` is what tools like `nav2_map_server` actually load**; it
points at the `.pgm`. Both must travel together.

Optional — also save the SLAM session itself (the pose graph):

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/maps/my_room'}"
```

This produces `.data` and `.posegraph` files that `slam_toolbox` can
later deserialize to *resume* mapping (vs starting a fresh SLAM run).
Useful when you want to add more area to an existing map across
multiple sessions.

After saving, commit the map files:

```bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot
git add maps/
git status
```

(Don't commit yet — wait until Phase 1 acceptance criteria are met.)

---

## 6. The `slam.launch.py` we'll write next (outline only)

Once a manual session has produced a usable map, we'll capture the
recipe as a single launch file so future runs are one command. This
section outlines its shape; the actual file lives at
`cat_patrol_robot/launch/slam.launch.py` and gets written as a
follow-up to this doc.

**Outline:**

- Argument: `use_sim_time` (default false).
- Argument: `rviz` (default true; pass `rviz:=false` for headless mode).
- Argument: `slam_params_file` (default points at our own copy of
  `mapper_params_online_async.yaml`, copied from
  `/opt/ros/humble/share/slam_toolbox/config/` so we can tweak it).
- Argument: `chassis_serial_port` and `lidar_serial_port` (forwarded to
  the bringup and lidar includes; same defaults as Phase 0).
- Include `yahboomcar_bringup_X3_launch.py` with `trim_vy_per_vx:=0.012`
  and `use_joystick:=true`.
- Include `sllidar_launch.py` with our by-id symlink and
  `frame_id:=laser_link`.
- Include `scan_front_filter` node (publishes `/scan_filtered`).
- Include `slam_toolbox`'s `online_async_launch.py` with
  [`config/mapper_params_online_async.yaml`](config/mapper_params_online_async.yaml).
- RViz on host only (document env vars); do not start RViz on Jetson.

**Why a separate launch file vs reusing `cat_patrol.launch.py`?**
`cat_patrol.launch.py` brings up the existing `patrol_node` and the
mail node — Phase 0/4 plumbing. `slam.launch.py` is a separate
single-purpose composition for mapping work. Keeping them separate
matches the "one launch file per task" principle: bringup is
hardware-only, slam is mapping-only, cat_patrol is application-only.
You compose them by including, not by accumulating into one file.


---

## 7. Pitfalls and how to recover

Likely failures, sorted by how often they bite, with one-line fixes.

| Symptom | Probable cause | Fix |
|---|---|---|
| Map display "no map received" in RViz | QoS mismatch (`/map` is Transient Local) or empty 0×0 map before driving | Map display: Topic `/map`, Durability **Transient Local**. Drive 30 s; check `width`/`height` on Jetson. |
| LaserScan "could not transform laser_link to map" | TF not ready, SLAM not up, or host not receiving `/tf` | Wait 20 s after SLAM start; host `ROS_DOMAIN_ID=28`; `ros2 run tf2_ros tf2_echo map laser_link` on host. |
| `slam_toolbox` dies `exit code -9` | OOM on 8 GB Jetson | Use our lighter params file; close host RViz while SLAM starts; don't run default upstream params. |
| Joystick no motion | `driver_node` crashed, or old enable-button workflow | `ros2 node list \| grep driver`; sticks work without Start (Start = buzzer on this pad). |
| Robot very slow | Low battery | Charge; check `/voltage`. |
| Host `ros2` daemon errors | Stale Jazzy daemon on peter-pen | `pkill -9 -f ros2cli.daemon`; use `ros2 --no-daemon topic list`. |
| Chaotic map / phantom walls near robot | Rear lidar hits robot superstructure | Run `scan_front_filter`; SLAM + RViz use `/scan_filtered`. |
| Map looks "smeared" — walls have ghost copies offset by a few cm | Driving too fast → scan matcher can't track | Slow to 0.10 m/s. Restart SLAM (no need to restart bringup or lidar). |
| Walls drift / duplicate the longer you drive | Odom drifting; loop closure hasn't fired | Drive back to a previously-seen area. Loop closure should fire and snap things into place. |
| `/scan` shows nothing in RViz | Either lidar motor stopped, or `frame_id` mismatch | `ros2 service call /start_motor std_srvs/srv/Empty`; verify `/scan`'s frame_id is `laser_link` not `laser`. |
| LiDAR sees beyond windows, sees mirror-reflections | Laser triangulation passes through glass and reflects off mirrors | Expected. Either accept the artefacts or edit them out of the saved `.pgm` later in GIMP. |
| Cats walk through scan plane | Moving obstacles get added to the map | SLAM tries to filter dynamic obstacles, but if it's bad, restart with cats elsewhere. |
| Smooth-floor in-place rotation produces a long arc | Mecanum rollers slip on smooth floor (Phase 0 finding) | Avoid in-place rotation on smooth floor — use arcing turns. |
| TF tree errors: "frame X does not exist" | Usually a node crashed (commonly EKF) | Check terminal 1 for crashed nodes. Restart bringup. |
| Map cells look noisy / speckled | LiDAR bouncing off shiny floor or returning bogus near-range | Slow down further. Increase `min_laser_range_meters` in slam params from 0 to 0.15. |
| `slam_toolbox` logs "Message Filter dropping message" | Normal at startup while TF tree settles | Ignore unless it persists more than ~5 seconds. |
| Mapping seems to do nothing after a long drive | TF chain broken or SLAM not getting scans | `ros2 run tf2_ros tf2_echo map odom`; `ros2 topic hz /scan_filtered`; confirm T2b filter running. |
| Robot doesn't move on `i` press | cmd_vel watchdog has stopped motors | Press `i` more firmly. Watchdog re-arms on first cmd_vel. |

---

## 8. Concept anchors (the study guide)

This is the "ask me later" section. Each bullet is a concept I (the
project's AI assistant) am ready to teach in detail when you query.
Don't try to learn them all up-front — pick whichever feels fuzzy
during a session.

### 8a. ROS 2 concepts to query me about

- **`nav_msgs/OccupancyGrid` message structure.** Header, MapMetaData
  (resolution, width, height, origin), and the int8 `data[]` array.
  Why is `data` a flat int8 not a 2D array? How does row-major indexing
  work?
- **The `/map` topic — why it's latched and low-frequency.** Latched
  topics in ROS 2 (transient_local QoS), the difference between "every
  subscriber gets the latest cached message" vs "subscribers only see
  future messages".
- **The `map → odom` transform as drift correction.** How does it
  *compose* with `odom → base_footprint` (from EKF) and the static
  TFs? Why don't we just have a `map → base_footprint` directly?
- **Launch-file composition.** `IncludeLaunchDescription`,
  `LaunchConfiguration`, `OpaqueFunction`. We touched the surface in
  Phase 0; in Phase 1's `slam.launch.py` we use them more.
- **YAML parameter files for slam_toolbox.** The
  `mapper_params_online_async.yaml` template, what each parameter does
  (`solver_plugin`, `ceres_*`, `correlation_search_space_*`,
  `loop_match_*`, `link_match_*`).
- **The `map_server` and `map_saver_cli` tools.** How does
  `map_saver_cli` ask `slam_toolbox` for the current map? Subscribe vs
  service call. Why does the saver need to be in the map's "save
  directory" or accept absolute paths?
- **`sensor_msgs/LaserScan` deeper dive.** `angle_min`,
  `angle_increment`, `range_min`, `range_max`, `ranges[]`,
  `intensities[]`, `scan_time`, `time_increment`. How
  `slam_toolbox` actually uses each.
- **TF lookup at a specific message timestamp.** In Phase 0 the drift
  checker used `tf2::TimePointZero`. The educational alternative is
  `tf2_ros::fromMsg(msg.header.stamp)` — when do you want which?
- **slam_toolbox modes.** Online async vs online sync vs offline vs
  lifelong vs localization. Which one we picked (online async) and
  why.
- **slam_toolbox localization-only mode** as a Phase 2 preview. It's
  a competing alternative to `nav2_amcl` — same job, different
  algorithm.
- **transient_local vs reliable QoS** specifically as it applies to
  the `/map` topic and why `slam_toolbox` chose what it did.

### 8b. SLAM theory to query me about

- **Occupancy grid representation.** free=0, occupied=100, unknown=-1.
  Cell coordinates vs world coordinates. The world↔grid conversion
  (origin + resolution + row-major flatten).
- **Bayesian log-odds update of cell occupancy.** Why log-odds
  instead of plain probability — additive updates, numerical
  stability. The Bayesian formulation of "I see a wall here" vs "I
  see clear space here".
- **Scan matching.** `slam_toolbox` uses Karto's correlative scan
  matcher under the hood. What that means: brute-force search of (x,
  y, yaw) offsets to align a new scan with an existing local map.
  Why it works at all (geometry of indoor environments). Where it
  fails (long featureless corridors, symmetric rooms, glass).
- **Pose graph SLAM.** Front-end (graph building: each scan becomes a
  node, scan-matches between nearby scans become edges) vs back-end
  (graph optimization: solve the joint configuration that minimizes
  edge errors). `slam_toolbox` uses Ceres Solver as the back-end.
- **Loop closure detection.** How `slam_toolbox` decides "I've been
  here before". The candidate-search radius, the match-quality
  threshold, why false loop closures are catastrophic.
- **Pose graph optimization on loop closure.** When a loop closes,
  what gets re-distributed? Every node in the cycle. Why does the
  whole map "snap" visibly when a loop closes — what's actually being
  recomputed.
- **Why `slam_toolbox` is graph-based and not particle-filter-based.**
  The contrast vs `gmapping` (the old ROS 1 default) and `nav2_amcl`
  (Phase 2's localization). Trade-offs: graph SLAM is more accurate
  for mapping; particle filters are better for localization on a
  fixed map.
- **The free-space rays.** Each LiDAR ray contributes a "free" update
  along its length and an "occupied" update at its endpoint. Why
  this is more useful than just marking endpoints.
- **Map drift vs odom drift.** Both exist. Which one the EKF fights,
  which one `slam_toolbox` fights, and why both layers are needed.

### 8c. Existing C++ code to read (no writing required)

A curated list of files to study, all already accessible. Each entry
is what's there to learn.

- **`slam_toolbox` source** at
  [github.com/SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox).
  Specific files:
  - `src/slam_toolbox_async.cpp` — the async-mode node entry-point.
    Compare to Phase 0's `odom_drift_checker_node.cpp` to see how a
    real production ROS 2 node is structured (lifecycle, callback
    groups, parameter callbacks, multi-threaded executor).
  - The Karto wrapper code — how a third-party C++ library gets
    integrated into a ROS 2 node.
  - Loop-closure detection logic — the actual search-and-match
    algorithm.
- **`nav2_map_server` source** at
  [github.com/ros-planning/navigation2/tree/main/nav2_map_server](https://github.com/ros-planning/navigation2/tree/main/nav2_map_server).
  How `nav_msgs/OccupancyGrid` is serialized to/from `.pgm` + `.yaml`.
  Concrete example of file I/O alongside ROS message handling.
- **`robot_localization`'s `ekf_filter_node`** — already touched in
  Phase 0; revisit how it composes with the TF tree once
  `slam_toolbox` adds the `map → odom` link.
- **`base_node_X3.cpp`** in this workspace at
  [`yahboomcar_ws/src/yahboomcar_base_node/src/base_node_X3.cpp`](../yahboomcar_base_node/src/base_node_X3.cpp).
  Reread as warm-up; the dead-reckoning math from Phase 0 will look
  closer to scan-matching geometry once you've read 8b.
- **`odom_drift_checker_node.cpp`** at
  [`odom_drift_checker/src/odom_drift_checker_node.cpp`](../odom_drift_checker/src/odom_drift_checker_node.cpp).
  Re-read with the new context: the TF lookup, the std::optional, the
  parameter idiom — they all reappear in `slam_toolbox`'s code at a
  larger scale.


---

## 9. ROS 2 idioms exercised in this phase

These are practical idioms the phase will exercise hands-on (vs the
deeper concepts in section 8 you can ask me about):

- **Composing one launch file from multiple existing ones.** Phase 0
  used `OpaqueFunction` lightly; Phase 1's `slam.launch.py` uses
  multiple `IncludeLaunchDescription`s.
- **Latched topic semantics.** `/map` is published with
  `transient_local` reliability and the cached most-recent message —
  RViz subscribes any time and immediately gets the current map.
  Compare with the streaming behaviour of `/scan` and `/odom`.
- **TF tree growth.** Phase 0's TF tree had 4 contributors. Phase 1
  adds `slam_toolbox` as a 5th, publishing `map → odom`. View it with
  `ros2 run tf2_tools view_frames`.
- **Service calls.** New ones in this phase: `/start_motor` and
  `/stop_motor` from the lidar driver, `/slam_toolbox/serialize_map`
  and `/slam_toolbox/deserialize_map`, `/slam_toolbox/save_map`.
- **Visual debugging in RViz vs terminal-only debugging.** Phase 0
  was almost entirely terminal-driven (`ros2 topic echo`, log lines).
  Phase 1 you really do need RViz — the map is geometric and the
  fastest debugger is your eyes.

---

## 10. Commands cheat-sheet

Copy-paste-ready commands. On the **Jetson**, source both setup files in
any new terminal. On the **host**, use Jazzy + `ROS_DOMAIN_ID=28`.

### Verify installation (Jetson)

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 pkg prefix slam_toolbox
ros2 pkg prefix nav2_map_server
ros2 pkg executables cat_patrol_robot | grep scan_front_filter
```

### Bringup — Jetson terminal 1

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py \
    chassis_serial_port:=/dev/ttyUSB1 \
    use_joystick:=true \
    trim_vy_per_vx:=0.012
```

Joystick caps (defaults): `joy_xspeed_limit=0.15`, `joy_yspeed_limit=0.15`,
`joy_angular_speed_limit=0.5`. Slower: add `joy_xspeed_limit:=0.10`.

### LiDAR — Jetson terminal 2

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_launch.py \
    serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
    serial_baudrate:=115200 \
    frame_id:=laser_link
```

### Scan filter — Jetson terminal 2b (**required**)

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 run cat_patrol_robot scan_front_filter
```

Fallback if `No executable found`: `chmod +x ~/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/scripts/scan_front_filter`

### slam_toolbox — Jetson terminal 3 (**always use lighter params**)

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=false \
    slam_params_file:=/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/config/mapper_params_online_async.yaml
```

### RViz — **host** terminal 4

```bash
export ROS_DOMAIN_ID=28
export ROS_LOCALHOST_ONLY=0
source /opt/ros/jazzy/setup.bash
ros2 run rviz2 rviz2 -d /home/bots/all/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_rviz/rviz/mapping.rviz
```

Map: `/map`, Durability **Transient Local**. LaserScan: `/scan_filtered`.

### Teleop fallback — Jetson terminal 5 (optional)

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Inspect during the session (Jetson)

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 topic hz /scan_filtered
ros2 topic hz /odom
ros2 topic hz /map
ros2 topic echo /map --once | grep -E 'width|height'
ros2 run tf2_ros tf2_echo map laser_link
ros2 node list | grep -E 'driver|slam|scan_front'
ros2 topic echo /voltage --once
```

### Inspect from host

```bash
export ROS_DOMAIN_ID=28
export ROS_LOCALHOST_ONLY=0
source /opt/ros/jazzy/setup.bash
ros2 --no-daemon topic list | grep -E 'map|scan_filtered|tf'
ros2 run tf2_ros tf2_echo map laser_link
```

### Save the map

```bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/maps
ros2 run nav2_map_server map_saver_cli -f my_room
```

### Serialize the SLAM session (optional, for resuming later)

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/maps/my_room'}"
```

### Pause / resume the lidar motor

```bash
ros2 service call /stop_motor std_srvs/srv/Empty
ros2 service call /start_motor std_srvs/srv/Empty
```

---

## 11. Phase 1 status log (filled in over time)

Empty for now. Fill in as you run sessions, the same way Phase 0's
calibration record filled in. Keep these terse and factual — surprises
and findings, not narration.

### Session 1 — 2026-06-20 (interrupted — battery)

- Room: _(fill in)_
- Start position (tape mark): yes
- Battery at start / end: low at end → robot very slow
- Trim used: `0.012`
- Stack: bringup + lidar + `scan_front_filter` + lighter SLAM params + host RViz (Jazzy)
- Findings:
  - Rear 180° LiDAR untrustworthy (robot structure) → `/scan_filtered` required
  - Chaotic map before filter; SLAM OOM with default params → lighter yaml
  - Host RViz: Map needs Transient Local QoS; LaserScan `/scan_filtered`
  - Joystick: Start = buzzer; drive enable defaulted on in `yahboom_joy_X3`
  - Joy speed capped at 0.15 m/s via bringup launch args
- Outcome: map not saved yet; resume after charge

### Session 2 — _date_

(copy template above)

### Final results

- Map file: `maps/<name>.pgm` + `.yaml`
- Resolution: ___ m/cell
- Map dimensions: ___ × ___ m
- Total drive distance during mapping: ___ m
- Drift observed (visual): ___
- Loop-closure events: ___
- Anything unusual:

### Calibration changes

- (record any time `trim_vy_per_vx` or other Phase 0 calibration
  values were re-measured or re-tuned because of Phase 1 observations)

---

## 12. Open questions / forward links

- **Phase 2 (AMCL).** The saved `.pgm` + `.yaml` from this phase
  becomes AMCL's input map. AMCL is a particle-filter-based
  localizer that estimates the robot's pose in the saved map at
  runtime. Same `map → odom` TF, different algorithm producing it.
- **Phase 7 (multi-room).** The plan's Phase 7 includes a
  multi-room item; that would extend Phase 1's per-room map approach
  to a set of saved maps with a "go to map X" command. Don't try to
  do this in Phase 1.
- **Surface-aware patrol planning.** Phase 0 added this as an open
  question in [`phase0-status.md`](phase0-status.md) section 6. The
  Phase 1 map is the substrate — once we have it, you can hand-paint
  a `floor_type.yaml` overlay (carpet vs smooth) and Phase 4's
  patrol manager can use that overlay to route in-place rotations
  onto carpet.
- **Map editing.** It's normal to clean up the saved `.pgm` in GIMP
  before using it for navigation: delete moving-obstacle artefacts
  (cats), delete phantom returns from glass and mirrors, fill in
  small holes. Considered "manual map curation" — not cheating; this
  is what professional navigation pipelines also do.
- **Multiple SLAM runs on the same room.** Each run produces a
  slightly different map even with identical driving (different
  starting noise, different random tie-breaks in the optimizer).
  Run 2-3 sessions and pick the cleanest one.
- **`slam_toolbox` lifelong mode.** Continuously update the map
  across sessions. Mentioned for context; Phase 1 uses online async
  for simplicity. Lifelong is a Phase 7 polish item.
