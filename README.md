# ros2_dynamixel

ROS2 package for controlling Dynamixel servo motors. It publishes joint positions and velocities and subscribes to torque/position commands. Built for ROS2 Humble, this driver is shared across multiple robot platforms (e.g. the ADAPT hand/finger and the SoftVMC soft arm) — each consumer supplies its own motor calibration via launch parameters.

## Overview

The package exposes a single node, `dynamixel_node`, which can operate in:

- **torque mode** — current-controlled motors, commanded via `/goal_torque`
- **position mode** — position-controlled motors, commanded via `/goal_position`
- **mixed mode** — when `position_motor_ids` is set: `motor_ids` stay torque-controlled (via `/goal_torque`), while `position_motor_ids` hold their startup position and are not independently commandable over ROS. `control_mode` is ignored in this case.

Joint state for `motor_ids` is published at ~1 kHz on `/joint_positions` and `/joint_velocities`.

## Requirements

- ROS 2 Humble
- `dynamixel_sdk` (ROS 2 package)
- Dynamixel motors connected on `/dev/ttyUSB0` (hardcoded, not a launch parameter)

## Build

From the package root:

```bash
cd ~/github/ros2_dynamixel
source /opt/ros/humble/setup.bash
colcon build --packages-select dynamixel_interface --event-handlers console_direct+
```

## Run

After building:

```bash
source install/setup.bash
ros2 run dynamixel_interface dynamixel_node
```

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `motor_ids` | `[1, 2]` | IDs of torque-controlled motors |
| `position_motor_ids` | `[]` | IDs of position-controlled motors in mixed mode |
| `control_mode` | `"torque"` | `"torque"` or `"position"` — ignored when `position_motor_ids` is non-empty (mixed mode always puts `motor_ids` in torque mode) |
| `baudrate` | `1000000` | Dynamixel communication baudrate |
| `velocity_filter_alpha` | `0.1` | Smoothing factor (EMA) for velocity output |
| `use_kt` | `true` | Enables the `kt` conversion in torque mode |
| `kt` | `0.00115` | Torque-to-current conversion constant, in `Nm/mA` |

In torque mode, commands received on `/goal_torque` become a goal current:

```text
goal_current = torque / kt      # when use_kt is true
goal_current = torque           # when use_kt is false — kt is ignored
```

The correct `kt` depends on the motor series, so different consumers of this package configure it differently — see Examples below.

## Topics

| Topic | Direction | Type | Active in | Units |
| --- | --- | --- | --- | --- |
| `/goal_torque` | subscribe | `std_msgs/Float64MultiArray` | torque mode, mixed mode | Nm (or raw current if `use_kt:=false`) |
| `/goal_position` | subscribe | `std_msgs/Float64MultiArray` | position mode only | degrees, relative to startup position |
| `/joint_positions` | publish | `std_msgs/Float64MultiArray` | always, for `motor_ids` | degrees, relative to startup position |
| `/joint_velocities` | publish | `std_msgs/Float64MultiArray` | always, for `motor_ids` | degrees/s, filtered |

Command arrays must match `motor_ids` in length and order — a mismatched length is silently dropped (the callback returns without applying it).

## Examples

### 1) Default: ADAPT hand/finger calibration

No arguments needed — these are all defaults:

```bash
ros2 run dynamixel_interface dynamixel_node
```

Equivalent explicit form:

```bash
ros2 run dynamixel_interface dynamixel_node --ros-args \
  -p motor_ids:="[1,2]" \
  -p control_mode:="torque" \
  -p use_kt:=true \
  -p kt:=0.00115
```

### 2) Custom `kt` for a different motor series

Different motor series need a different `kt`. For example, the SoftVMC soft-arm package drives 9 larger motors and launches with:

```bash
ros2 run dynamixel_interface dynamixel_node --ros-args \
  -p motor_ids:="[11,12,13,14,15,16,17,18,19]" \
  -p control_mode:="torque" \
  -p use_kt:=true \
  -p kt:=0.354
```

### 3) Disable the `kt` conversion

Set `use_kt:=false` to send `/goal_torque` values straight through as goal current, with no scaling applied. `kt` is ignored in this mode and can be omitted:

```bash
ros2 run dynamixel_interface dynamixel_node --ros-args \
  -p motor_ids:="[1,2]" \
  -p control_mode:="torque" \
  -p use_kt:=false
```

## Notes

- The correct `kt` depends on the motor series — don't reuse a value across different Dynamixel models.
- `0.00115` is the calibration for the ADAPT hand/finger motors; other consumers of this package pass their own `kt` at launch (see Examples).
- With `position_motor_ids` set (mixed mode), position motors hold whatever position they were in at node startup — they are not commandable over ROS.
- The serial port is hardcoded to `/dev/ttyUSB0`.
