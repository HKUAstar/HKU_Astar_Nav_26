# PID Navigation & Stale cmd_vel Fix Report

**Date:** 2026-05-08  
**Workspace:** `/home/sentry/AstarTraining/Old_nav/sim_nav`  
**Package:** `bot_sim` (catkin)  
**Planners:** `dstarlite` (grid-based), `dstarlite_esdf` (ROG-Map ESDF)

---

## 1. Session Overview

This session covered iterative development and debugging of a PID velocity controller integrated into the D\*-Lite navigation planners. Key topics addressed:

1. PID integration into D\*-Lite planners
2. Corner overshooting fix (lookahead tuning)
3. RViz cmd_vel / raw_cmd_vel visualizers
4. Uphill slope boost
5. Stale cmd_vel drift bug — root cause analysis and fix
6. Unified 10 Hz control loop

---

## 2. File Inventory

| File | Role |
|------|------|
| `src/bot_sim/include/bot_sim/velocity_pid_controller.hpp` | Header-only PID velocity controller |
| `src/bot_sim/src/dstarlite.cpp` | Grid D\*-Lite planner main loop |
| `src/bot_sim/src/dstarlite_esdf.cpp` | ESDF D\*-Lite planner main loop (mirrors dstarlite.cpp) |
| `src/bot_sim/launch_real/dstarlite.launch` | Launch file — real robot, grid planner |
| `src/bot_sim/launch_real/dstarlite_esdf.launch` | Launch file — real robot, ESDF planner |
| `../tf_test.rviz` | RViz config with cmd_vel marker displays |

---

## 3. Changes Made

### 3.1 Corner Overshoot Fix

**Problem:** Lookahead of 12 nodes caused the direction vector to cut diagonally across corners, overshooting turns.

**Fix:** Lookahead reduced from 12 → 5 nodes (index `cur->succ` direction computation in both planners).

---

### 3.2 RViz cmd_vel Visualizers

Two `cmd_vel_marker` node instances added to launch files:
- One subscribing to `/cmd_vel` → publishing `/cmd_vel_marker`
- One subscribing to `/dstarlite/raw_cmd_vel` → publishing `/dstarlite/raw_cmd_vel_marker`

`tf_test.rviz` updated with two `Marker` displays for those topics.

Also: `raw_cmd_vel_pub_` added to planners to publish the pre-PID desired velocity before correction.

---

### 3.3 Slope Boost

**Function:** `slope_boost_scale(z_angle)` in both planners.  
**Source:** `z_angle` derived from TF pitch component (already computed in planner).

**Behavior:**
| z_angle | Scale |
|---------|-------|
| < 4°    | 1.0× (no boost) |
| 4°–15°  | Linear ramp 1.0× → 1.6× |
| > 15°   | 1.6× (capped) |

**Launch parameters:**
```xml
<param name="slope_boost_enabled"    value="true"/>
<param name="slope_boost_start_deg"  value="4.0"/>
<param name="slope_boost_full_deg"   value="15.0"/>
<param name="slope_boost_max_scale"  value="1.6"/>
<param name="slope_boost_uphill_sign" value="-1.0"/>
```

---

### 3.4 Stale cmd_vel Drift Fix (Main Fix)

#### Root Cause Analysis

Three compounding causes:

**Cause A — `holdPreviousOutput` perpetuating stale commands**

The planner ran at 100 Hz; SLAM (map→virtual_frame TF) updates at ~10 Hz. 9 out of 10 loop cycles returned `FeedbackStatus::NoNewSample` (TF existed but timestamp hadn't advanced). The old code called `holdPreviousOutput(desired)`, which re-published the last corrected nonzero `/cmd_vel` verbatim. The robot was almost always driving on a stale command with no fresh PID feedback.

**Cause B — `limitedFallback` on TF Unavailable**

When SLAM dropped out entirely (TF missing or expired), `FeedbackStatus::Unavailable` triggered `limitedFallback()`. That function magnitude-clamped the raw desired velocity and published it — no PID correction, no feedback, just open-loop planner command. Since the planner has no knowledge localization is gone, it kept issuing nonzero desired velocity → robot drifted at full speed with zero feedback.

**Cause C — 100 Hz / 10 Hz mismatch masked the transition**

Because `holdPreviousOutput` was the normal path (90% of cycles), there was no clear boundary between "TF is just slow" and "TF is actually gone." The robot was already in a semi-open-loop state. When SLAM finally dropped fully, `NoNewSample → Unavailable` produced no behavioral change (both paths published nonzero velocity), so drift was invisible until the robot had moved significantly.

#### Fix Applied

**`velocity_pid_controller.hpp`:**
- `FeedbackStatus::NoNewSample` → `holdPreviousOutput(desired)` (unchanged — preserves last PID output for valid slow-TF case)
- `FeedbackStatus::Unavailable` → `stoppedFallback()` (NEW — resets PID state, returns zero Twist)
- `stoppedFallback()` at line ~662: resets integrators and returns zero
- `estimateMeasuredVelocity()`: added TF age check — if `ros::Time::now() - stamp > feedback_timeout_` → set stale flag → return `Unavailable`

**`dstarlite.cpp` / `dstarlite_esdf.cpp`:**
- Added `is_tf_stale()` helper (line ~534): checks `transformStamped.header.stamp` age against `cmd_vel_tf_timeout` param; calls `publish_stop()` and returns `true` if stale
- Planner loop calls `is_tf_stale()` before goal handling AND before `publish()`
- `control_rate_hz` param (default 10.0): `ros::Rate rate(control_rate_hz)` — loop matches SLAM rate
- `cmd_vel_tf_timeout` param (default 0.3 s)

**Launch parameters added:**
```xml
<param name="control_rate_hz"     value="10.0"/>
<param name="cmd_vel_tf_timeout"  value="0.3"/>
```

---

### 3.5 PID Parameters (launch files)

```xml
<param name="pid_feedforward"      value="0.9"/>
<param name="pid_kp_x"             value="0.8"/>
<param name="pid_kp_y"             value="0.8"/>
<param name="pid_ki_x"             value="0.5"/>
<param name="pid_ki_y"             value="0.5"/>
<param name="pid_kd_x"             value="0.2"/>
<param name="pid_kd_y"             value="0.2"/>
<param name="pid_integral_limit"   value="0.8"/>
<param name="pid_output_limit"     value="3.0"/>
<param name="pid_accel_limit"      value="5.0"/>
<param name="pid_deadband"         value="0.01"/>
```

---

## 4. Build Command

```bash
cd /home/sentry/AstarTraining/Old_nav/sim_nav
source /opt/ros/noetic/setup.bash
source ../livox_ws/devel/setup.bash --extend
catkin_make --pkg bot_sim -j2
```

Binaries verified at ~23:25 timestamp:
- `devel/lib/bot_sim/dstarlite` — 315960 bytes
- `devel/lib/bot_sim/dstarlite_esdf` — 27277856 bytes

---

## 5. System Architecture Notes

- **TF topology:** `map → virtual_frame` published by HDL localization at ~10 Hz
- **Planner input:** TF for current pose; ESDF/grid for obstacle data
- **Planner output:** `/cmd_vel` (PID-corrected), `/dstarlite/raw_cmd_vel` (pre-PID desired)
- **Downstream:** `/mcu_communicator` subscribes to `/cmd_vel`
- **SLAM rate:** ~10 Hz (HDL graph SLAM / localization)

---

## 6. Key Lessons

- **PID loop rate must match sensor update rate.** Running at 10× the TF rate caused `holdPreviousOutput` to dominate, degrading the feedback loop to near open-loop behavior.
- **Always define explicit stop behavior for sensor dropout.** An open-loop fallback (`limitedFallback`) is dangerous when the sensor (SLAM) is the only source of pose truth.
- **TF freshness must be checked at the planner level**, not just inside the PID controller, because the planner may have cached stale pose data from a prior valid transform.
- **`cmd_vel_tf_timeout = 0.3 s`** is a conservative but safe threshold for a 10 Hz SLAM source (allows 3 missed frames before stopping).
