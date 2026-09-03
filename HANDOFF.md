# RAVEN + Junbin's Planner — Session Handoff

Context doc for starting a fresh chat. Written 2026-09-03.

---

## 1. What this project is

Goal: a **semantic-aware path planner** — RayFronts picks *what/where* to go
(semantic targets), a collision-avoiding local planner figures out *how* to get
there, running in Isaac Sim against a Shimizu construction-site scene.

Target architecture (from the RAVEN README diagram):

```
Isaac Sim  --interface-->  AirStack           --sensor topics-->  RayFronts
                           - sim interface                        - 3D mapping
                           - sensor drivers                       - semantic logic
                           - global planner   <--global waypts--  - behavior tree
                           - local planner
                           - collision avoid
```

RayFronts = target selector. AirStack = flight + collision avoidance.

---

## 2. Repos and branches

| Path | Repo | Branch | Notes |
|---|---|---|---|
| `/home/avid/RAVEN` | `AviDube/RAVEN` | `raven` | meta-repo, submodules below |
| `/home/avid/RAVEN/AirStack` | `AviDube/AirStack` | `raven` | **old numbered-layer layout** (`autonomy/3_local/...`) |
| `/home/avid/RAVEN/RayFronts` | `AviDube/RayFronts` | `raven` | |
| `/home/avid/AirStack` | `castacks/AirStack` | `junbin/planning_demo` | Junbin's reference checkout (read-only source) |

**Nothing is on `main`** — deliberately. Meta-repo was on `main`; a `raven`
branch was created to match the submodules.

**Unpushed:** two commits on `AviDube/AirStack@raven` (`f1041447`, `25de8475`).
Push them before doing anything destructive.

---

## 3. Current state — what actually works

Verified working end-to-end:

- Isaac Sim boots the Shimizu scene cleanly (no crash)
- Arm → takeoff → hover at ~3 m, `takeoff_state: COMPLETE`
- Global Plan → `integrated_planner` autonomously explores
- `/cmd_trajectory` @ ~1 Hz → `/robot_1/interface/cmd_velocity` @ ~9.7 Hz
- Chase camera publishing on `/sim/chase/image` @ ~22–25 Hz

**Known defect:** motion is *erratic* — two controllers fight (see §5.3).

---

## 4. How to run and test

### Full clean restart
```bash
cd /home/avid/RAVEN && ./restart_raven_fresh.sh
```
Recreates isaac-sim + robot + rayfront_docker and launches the RayFronts
mapping server. Takes ~3 min.

### Faster (no RayFronts)
```bash
cd /home/avid/RAVEN/AirStack
docker compose --env-file .env -f docker-compose.yaml up -d --force-recreate isaac-sim robot
```

### Wait for boot
```bash
until docker exec isaac-sim tmux capture-pane -t isaac -p -S -30 2>/dev/null \
  | grep -qi "Ready for takeoff\|poll timeout\|ekf2 missing\|Incompatible size"; do sleep 5; done
```

### ⚠️ ROS2 CLI gotcha — read this before debugging anything
Every `ros2` command needs the domain and workspace overlay, or you get a
**silently empty/wrong view** of the system:
```bash
docker exec airstack-robot-1 bash -c \
  "export ROS_DOMAIN_ID=1 && source /opt/ros/humble/setup.bash && \
   source /root/AirStack/robot/ros_ws/install/setup.bash && <cmd>"
```
`ROS_DOMAIN_ID` is derived from `ROBOT_NAME` in the container's `.bashrc`
(robot_1 → 1). Without it you're on domain 0 and see ~7 topics instead of ~200.
This wasted hours.

### Arm + takeoff (automated)
```bash
timeout 12 ros2 topic pub -r 2 /robot_1/behavior/behavior_tree_commands \
  behavior_tree_msgs/msg/BehaviorTreeCommands \
  "{commands: [{condition_name: 'Auto Takeoff Commanded', status: 2},
   {condition_name: 'Fixed Trajectory Commanded', status: 0},
   {condition_name: 'Global Plan Commanded', status: 0},
   {condition_name: 'Pause Commanded', status: 0},
   {condition_name: 'Rewind Commanded', status: 0},
   {condition_name: 'Disarm Commanded', status: 0},
   {condition_name: 'Land Commanded', status: 0},
   {condition_name: 'Autonomously Explore Commanded', status: 0}]}"
```
Then Global Plan = same message with `Auto Takeoff Commanded: 0` and
`Global Plan Commanded: 2`.

**Must publish repeatedly (`-r 2`), not `--once`.** A single `--once` publish is
silently dropped if the subscriber hasn't matched yet — this produced several
bogus "arm didn't register" results.

The behavior tree is a **priority selector**: the `Auto Takeoff Commanded`
branch must be cleared (status 0) before the tree will ever reach the
`Global Plan Commanded` branch.

### Verify
```bash
ros2 topic echo /robot_1/interface/is_armed --once                       # data: true
ros2 topic echo /robot_1/takeoff_landing_planner/takeoff_state --once    # COMPLETE
ros2 topic echo /robot_1/interface/mavros/local_position/pose --once     # z ~2.52
docker exec isaac-sim tmux capture-pane -t isaac -p -S -3                # no poll timeout
```
Ground reads z ≈ −0.48, so z ≈ 2.52 is ~3.0 m AGL.

### Look at the drone (chase camera)
Grab a frame to a host-visible path and view it:
```python
# saves to /root/AirStack/chase_frame.png == /home/avid/RAVEN/AirStack/chase_frame.png
# subscribe /sim/chase/image (rgb8 640x640), reshape, PIL save
```
(Full script pattern was: subscribe, skip 2 frames, `np.frombuffer(...).reshape(h,w,-1)`, PIL save.)

---

## 5. Hard-won findings — do not re-litigate these

### 5.1 `pid_controller` is mandatory (THE big one)
It is the **only** node publishing a continuous
`cmd_roll_pitch_yawrate_thrust` stream to MAVROS. Remove it and PX4 arms,
receives *no* control input, and the SITL link dies — surfacing as
`[simulator_mavlink] poll timeout` and **looking exactly like an Isaac Sim
crash**. A/B verified:

| Config | Result |
|---|---|
| baseline (all nodes) | ✅ takeoff |
| droan + disparity_expansion + **pid_controller** off | ❌ crash at arm |
| droan + disparity_expansion off, **pid_controller** on | ✅ takeoff |

This one issue caused ~a full day of misdirected debugging into PhysX tensor
errors, GPU state, shader caches, and scene content. **If you see
`poll timeout` after arming, check the control stream first.**

### 5.2 Junbin's own launch file fails here for the same reason
`shimizu_local.launch.xml` drops `pid_controller` in favour of
`pid_path_tracker`, but `pid_path_tracker` only publishes once it *has* a
trajectory, so nothing covers the arm→takeoff window. Not an environment
difference — a real gap in that config. (Partially fixed, see 5.4.)

### 5.3 Two controllers fight → erratic motion (current defect)
With both running: `cmd_velocity` @12.6 Hz (pid_path_tracker flying the
exploration path) *and* `cmd_roll_pitch_yawrate_thrust` @12.3 Hz
(pid_controller holding the takeoff point). Net = small oscillation around
the start instead of exploring.

### 5.4 The hold-publish fix (done, in `25de8475`)
`pid_path_tracker_cmd_vel_stamped` now publishes a zero-velocity hold when it
has odometry but no trajectory. This **does** keep PX4 alive through arming
(verified: armed, no poll timeout). But with `pid_controller` removed the
drone never climbs — `takeoff_state` sticks at `TAKING_OFF` until PX4
auto-disarms — because the climb is executed by
`trajectory_controller → pid_controller`.

### 5.5 The ModalAI template crash fix (in the baseline commit)
`e57_with_model_ai.usd` ships a baked-in ModalAI drone template whose
stereo-camera subtree makes `omni.physx.tensors.plugin` issue a broken
rigid-body query every physics step
(`Pattern '.../spirit_uav/base_link/front_stereo' did not match any rigid
bodies` / `Incompatible size of velocity tensor ... shape (2, 6)`).
`Shimizu_Launch.py` deactivates the whole template after load, then re-applies
the dome light (that subtree also carried the scene's only working light —
deactivating it alone blacks out the viewport). Deactivating only the
`front_stereo` subtree is **not** sufficient (ekf2 still starves).

### 5.6 Hardcoded demo bounds (removed)
`astar_vdb.cpp` and `integrated_node.cpp` both had hardcoded ~9 m boxes near
world origin ("modal ai demo bound at RIC mocap"). Our drone operates near
(−50, −10), so these silently rejected every candidate. Removed in both.

### 5.7 Chase camera parenting
Must parent under `/World/base_link/body` (the PhysX-simulated rigid body),
**not** `/World/base_link` (a static reference Xform PhysX never writes to).
Wrong parent = camera doesn't follow, and it also caused a hang.

### 5.8 Sensor topic mismatch
Junbin's defaults target an Ouster rig
(`/robot_1/sensors/ouster/point_cloud`, frame `ouster`). Ours is
`/robot_1/sensors/lidar/point_cloud`, frame `lidar` (verified live). A wrong
topic **fails silently** as an always-empty map that never rejects anything —
worse than an error. Overridden in `global.launch.xml`.

### 5.9 `integrated_planner` gating
It does nothing until `enable_exploration` is toggled via
`/robot_1/behavior/global_plan_toggle` (std_srvs/Trigger). The behavior tree's
Global Plan action does call this. Calling it manually **toggles**, so calling
it when already enabled turns exploration *off*.

---

## 6. Decided next step — migrate to AirStack `main` (option 2)

`main`'s `trajectory_controller/README.md` documents the intended chain:

```
Trajectory Controller  →  PID Controller        →  Flight Computer
(pure pursuit,            (cascaded PID:           roll/pitch/yaw-rate/thrust
 publishes tracking_point) position + velocity)    → MAVROS
```

and `Trajectory.msg = nav_msgs/Odometry[]`, explicitly supporting *"stitched
segments output continuously by a local planner"*.

**So the planner should feed `trajectory_controller`, not bypass it with
`pid_path_tracker` + `cmd_velocity`.** That removes the fighting (§5.3) and the
arm-window gap (§5.2/5.4) by construction — there's one control chain.

Plan:
1. Push the two unpushed commits.
2. Migrate onto `main` (module/submodule layout, improved controllers).
3. Wire `integrated_planner` in as a module feeding `trajectory_controller`
   segments; **drop `pid_path_tracker` entirely**.
4. Wire RayFronts → goal waypoints into the planner.

`main` also uses submodules throughout, which the AirStack devs say makes
integration easier — that's why option 2 over patching the old layout.

---

## 7. Still open / not done

- **RayFronts → planner waypoints.** `integrated_planner` is currently
  *autonomous* (its own frontier selection). Its `/perspective_goals` hook is
  **dead code** — declared as a param, never wired to a `create_subscription`.
  Same in `goal_planner_node.cpp`. A real subscription must be added, or use
  `goal_planner_node` (which *does* genuinely subscribe `/goal_point`,
  Reliable + TransientLocal QoS).
- RayFronts currently publishes `nav_msgs/Path` on `/robot_1/global_plan`
  (baseline behavior; the `/goal_point` rewiring was reverted).
- droan's `seen_radius` limitation (rejects targets in unobserved space) is
  the original reason for swapping planners — still true if you re-enable droan.
- `domain_bridge` in Junbin's launch fails on a hardcoded `/home/robot/...`
  path — harmless noise, ignore.

---

## 8. Methodology that worked

Change **one thing at a time**, restart clean, test arm→takeoff, record the
result. That's what finally isolated §5.1 after a day of broad theorizing.
Broad multi-variable changes plus log-reading repeatedly produced confident
but wrong conclusions.
