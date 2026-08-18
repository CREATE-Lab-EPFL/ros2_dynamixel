# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A ROS 2 Humble package (`dynamixel_interface`) with a single node, `dynamixel_node`, that drives Dynamixel servo motors: publishes joint position/velocity and subscribes to torque/position commands. It is consumed as an external dependency by other robot-control repos (e.g. `~/github/SoftVMC`'s soft arm), so parameter names/semantics and topic contracts are a cross-repo API — don't change them without checking downstream usage in sibling repos.

## Build

```bash
cd ~/github/ros2_dynamixel
source /opt/ros/humble/setup.bash
colcon build --packages-select dynamixel_interface --event-handlers console_direct+
source install/setup.bash
```

## Run

```bash
ros2 run dynamixel_interface dynamixel_node [--ros-args -p <param>:=<value> ...]
```

There is no automated test suite and no linter configured for this package. Verify changes by:

1. A truly clean rebuild — `rm -rf build/dynamixel_interface install/dynamixel_interface` before `colcon build` (an incremental build can silently relink a stale object file without recompiling). It must compile warning-free: `CMakeLists.txt` enables `-Wall -Wextra -Wpedantic`.
2. Running the node with no hardware attached: it will fail at `portHandler_->openPort()` with `"Port initialization failed"` and then idle — that's expected in a dev environment without motors on `/dev/ttyUSB0`, not a regression. It's enough to confirm parameters parse and the node doesn't crash; a real functional check needs actual hardware.

## Architecture

Everything lives in one file: `src/dynamixel_interface/src/dynamixel_node.cpp`. No header split, no launch files, no config files.

The node has three mutually exclusive modes, resolved once at startup from `position_motor_ids` and `control_mode`:

- **torque mode** (default) — all `motor_ids` are current-controlled. Subscribes `/goal_torque`.
- **position mode** (`control_mode:=position`, `position_motor_ids` empty) — all `motor_ids` are position-controlled. Subscribes `/goal_position`.
- **mixed mode** (`position_motor_ids` non-empty) — `motor_ids` are forced torque-controlled regardless of `control_mode` (that param is silently ignored in this case). `position_motor_ids` hold whatever position they were in at node startup and are **not** commandable over ROS at all — no subscription is created for them.

Torque commands go through a configurable current conversion: `goal_current = torque / kt` when `use_kt` is true (default `kt = 0.00115 Nm/mA`), or the raw value passed straight through when `use_kt` is false (in which case `kt` is still declared/read but never applied). `kt` is motor-series-specific: different consumer repos pass different values at launch — e.g. SoftVMC's 9-motor arm uses `kt:=0.354` vs. the `0.00115` default calibrated for the ADAPT hand/finger. Treat `kt`'s default as belonging to the hand/finger use case specifically, not as a universal constant.

`torqueCallback`/`positionCallback` silently drop the incoming message if its array length doesn't match `motor_ids.size()` — no warning is logged. Keep this in mind when debugging "commands aren't reaching the motors."

Threading: `goal_command_storage_` is written by the subscription callback (ROS executor thread) and read by `publishState()` on a 1kHz wall timer; `goal_mutex_` guards the handoff. The timer callback is also where the actual SyncWrite/SyncRead I/O to the motors happens — callbacks themselves never touch the hardware.

The serial port is hardcoded to `/dev/ttyUSB0` in the constructor — unlike `baudrate`, it is not a launch parameter.

## Parameters

`motor_ids`, `position_motor_ids`, `control_mode`, `baudrate`, `velocity_filter_alpha`, `use_kt`, `kt` — see README.md for the full defaults/meaning table, the topic contract, and worked examples per motor series (default hand/finger calibration vs. SoftVMC's arm calibration). Keep README.md and this file consistent if either changes.
