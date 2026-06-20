# Phase 1 — Status & Reference

Companion to [`plan.md`](plan.md), [`hardware.md`](hardware.md),
[`architecture.md`](architecture.md), and [`phase0-status.md`](phase0-status.md).
This is the practical "what we plan to do for Phase 1, what we actually
did, and what we learned" document.

Phase 1 = build a 2D map of one room with `slam_toolbox`, save it to
disk, and write a single-command launch file that reproduces the SLAM
session. **Scope deliberately small** — one room only. Multi-room
mapping is deferred to Phase 7.

When this doc is written, Phase 1 has not yet started; the status-log
section (11) is empty by design and fills in as you run sessions.

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
- (Optional, only if RViz freaks out with the default config) Re-base
  the SLAM RViz config from `yahboomcar_rviz/rviz/mapping.rviz` or from
  `slam_toolbox`'s own `slam_toolbox_default.rviz` at
  `/opt/ros/humble/share/slam_toolbox/config/`.

### 3b. Every-session checklist

- Battery > ~11.5 V:
      `ros2 topic echo /voltage --once`
  Below ~11.0 V the chassis will start to misbehave under load.
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

A SLAM session uses 5 terminals. Each on its own terminal so any one
can be killed without taking down the others.

### Terminal 1 — bringup (chassis + EKF + IMU + joystick)

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py \
    chassis_serial_port:=/dev/ttyUSB1 \
    use_joystick:=true \
    trim_vy_per_vx:=0.012
```

Wait for `cmd_vel trim: trim_vy_per_vx=0.0120` log line before
continuing.

### Terminal 2 — LiDAR

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_launch.py \
    serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
    serial_baudrate:=115200 \
    frame_id:=laser_link
```

### Terminal 3 — slam_toolbox (online async mode)

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=false
```

`slam_toolbox` will start subscribing to `/scan`, `/odom`, and TF.
Within a few seconds you should see it log `Message Filter dropping
message: frame ...` (transient; ignore) and then settle into normal
operation. Once normal it logs scan-matching diagnostics every few
seconds.

### Terminal 4 — RViz

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/slam_toolbox/config/slam_toolbox_default.rviz
```

Or use the Yahboom mapping config:
```bash
ros2 run rviz2 rviz2 -d ~/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_rviz/rviz/mapping.rviz
```

You should see the live `/scan` dots, the robot model (URDF), and the
map starting to fill in as walls and obstacles are observed.

### Terminal 5 — teleop_twist_keyboard

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Slow down before driving:
- Press `x` four to six times → linear speed ~0.10–0.15 m/s.
- Press `c` four times → angular speed ~0.5 rad/s.

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
- **Loop back to start at least once.** The whole point of pose-graph
  SLAM is loop closure; without it, the map drifts. When you return to
  near your tape-mark start, watch the `slam_toolbox` console for a log
  line containing "Loop Closure" and watch RViz for a visible snap of
  the map.
- **Repeat the loop one or two more times.** First loop closes the
  pose graph; subsequent loops let `slam_toolbox` refine the
  optimization. The map gets visibly cleaner each time.
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
- Include `slam_toolbox`'s `online_async_launch.py` with our params
  file.
- Conditionally include `rviz2` with the SLAM config.

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
| Map looks "smeared" — walls have ghost copies offset by a few cm | Driving too fast → scan matcher can't track | Slow to 0.10 m/s. Restart SLAM (no need to restart bringup or lidar). |
| Walls drift / duplicate the longer you drive | Odom drifting; loop closure hasn't fired | Drive back to a previously-seen area. Loop closure should fire and snap things into place. |
| `/scan` shows nothing in RViz | Either lidar motor stopped, or `frame_id` mismatch | `ros2 service call /start_motor std_srvs/srv/Empty`; verify `/scan`'s frame_id is `laser_link` not `laser`. |
| LiDAR sees beyond windows, sees mirror-reflections | Laser triangulation passes through glass and reflects off mirrors | Expected. Either accept the artefacts or edit them out of the saved `.pgm` later in GIMP. |
| Cats walk through scan plane | Moving obstacles get added to the map | SLAM tries to filter dynamic obstacles, but if it's bad, restart with cats elsewhere. |
| Smooth-floor in-place rotation produces a long arc | Mecanum rollers slip on smooth floor (Phase 0 finding) | Avoid in-place rotation on smooth floor — use arcing turns. |
| TF tree errors: "frame X does not exist" | Usually a node crashed (commonly EKF) | Check terminal 1 for crashed nodes. Restart bringup. |
| Map cells look noisy / speckled | LiDAR bouncing off shiny floor or returning bogus near-range | Slow down further. Increase `min_laser_range_meters` in slam params from 0 to 0.15. |
| `slam_toolbox` logs "Message Filter dropping message" | Normal at startup while TF tree settles | Ignore unless it persists more than ~5 seconds. |
| Mapping seems to do nothing after a long drive | TF chain is broken; check map → odom is being published | `ros2 run tf2_ros tf2_echo map odom`. If that fails, slam_toolbox isn't getting scans aligned to odom. Check `/scan` and `/odom` are both publishing. |
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

Copy-paste-ready commands. Source `setup.bash` in any new terminal
first (Phase 0 prelude).

### Verify installation

```bash
ros2 pkg prefix slam_toolbox
ros2 pkg prefix nav2_map_server
ls /opt/ros/humble/share/slam_toolbox/launch/
ls /opt/ros/humble/share/slam_toolbox/config/
```

### Bringup with trim — terminal 1

```bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py \
    chassis_serial_port:=/dev/ttyUSB1 \
    use_joystick:=true \
    trim_vy_per_vx:=0.012
```

### LiDAR — terminal 2

```bash
ros2 launch sllidar_ros2 sllidar_launch.py \
    serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
    serial_baudrate:=115200 \
    frame_id:=laser_link
```

### slam_toolbox online async — terminal 3

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
```

To use a custom params file (recommended once you start tweaking):

```bash
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=false \
    slam_params_file:=/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/config/mapper_params_online_async.yaml
```

### RViz — terminal 4

```bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/slam_toolbox/config/slam_toolbox_default.rviz
```

Or:

```bash
ros2 run rviz2 rviz2 -d ~/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_rviz/rviz/mapping.rviz
```

### Teleop (slowed down) — terminal 5

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# then press x several times to drop linear speed to ~0.10-0.15 m/s
# and c several times to drop angular to ~0.5 rad/s
```

### Inspect during the session

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_tools view_frames
ros2 service list | grep slam_toolbox
ros2 topic echo /voltage --once
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

### Session 1 — _date_

- Room:
- Start position (tape mark):
- Battery at start / end:
- Trim used:
- Total drive time:
- Loop closures observed (count + approximate locations):
- Outcome (map saved? smearing? holes?):
- Surprises / findings:

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
