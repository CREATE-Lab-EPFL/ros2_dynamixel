# ros2_dynamixel

ROS2 package for controlling Dynamixel servo motors. It publishes joint positions and velocities and subscribes to torque/position commands. This package is intended for the ADAPT soft-robot hand and is built for ROS2 Humble.

## Overview

The package exposes a single node, `dynamixel_node`, which can operate in:

- torque mode for current-controlled motors
- position mode for position-controlled motors
- mixed mode when `position_motor_ids` is provided

The default setup is torque mode with the current KT calibration enabled.

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

The node accepts the following runtime parameters. Defaults are shown below.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `motor_ids` | `[1, 2]` | IDs of torque-controlled motors |
| `position_motor_ids` | `[]` | IDs of position-controlled motors in mixed mode |
| `control_mode` | `"torque"` | `"torque"` or `"position"` |
| `baudrate` | `1000000` | Dynamixel communication baudrate |
| `velocity_filter_alpha` | `0.1` | Smoothing factor for velocity output |
| `use_kt` | `true` | Enables the KT conversion in torque mode |
| `kt` | `0.00115` | Torque-to-current conversion constant in `Nm/mA` |

The conversion used in torque mode is:

```text
goal_current = torque / kt
```

When `use_kt` is `false`, the incoming torque value is used directly without the conversion.

## Examples

### 1) Default setup: use the built-in KT value

```bash
cd ~/github/ros2_dynamixel
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run dynamixel_interface dynamixel_node --ros-args \
  -p motor_ids:="[1,2]" \
  -p control_mode:="torque" \
  -p baudrate:=1000000 \
  -p use_kt:=true \
  -p kt:=0.00115
```

### 2) Custom KT override: disable the default conversion

```bash
cd ~/github/ros2_dynamixel
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run dynamixel_interface dynamixel_node --ros-args \
  -p motor_ids:="[1,2]" \
  -p control_mode:="torque" \
  -p baudrate:=1000000 \
  -p use_kt:=false \
  -p kt:=0.0009
```

This second example keeps the node active but bypasses the default `KT` scaling and uses the custom value instead.

## Notes

- Keep in mind that the correct `kt` depends on the motor series.
- The default value `0.00115` is the calibration currently used in this package.
- If you use `position_motor_ids`, the node behaves in mixed mode: torque motors stay in current mode and position motors hold their commanded positions.
