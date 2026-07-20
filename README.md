# ainex-ros2

A ROS 2 (Humble) port of the Hiwonder **AiNex** humanoid stack, plus a
ROS-version-agnostic motion daemon that lets the same walking engine drive
either the real robot or a Gazebo digital twin.

This repository contains **only the reusable, vendor-derived port** — the
generic robotics plumbing that any AiNex owner could run. Application-specific
behaviour (mission logic, perception intelligence, personality) is **not**
here.

## What's included

| Package | What it is |
|---|---|
| `ros2_ws/src/ainex_interfaces` | The AiNex message/service definitions, ported from ROS 1 to ROS 2 IDL (12 msgs, 10 srvs). |
| `ros2_ws/src/ainex_bridge_interfaces` | Bus-servo message types mirroring the vendor's `ros_robot_controller` msgs. |
| `ros2_ws/src/ainex_bridge` | A ROS 2 facade node exposing the familiar vendor topics/services (`/walking/command`, `/app/set_walking_param`, head control, bus-servo I/O) and forwarding them to the motion daemon over a local socket. Also `sim_adapter`, which streams daemon joint output into the Gazebo controller. |
| `ros2_ws/src/ainex_camera` | Camera bring-up: `v4l2_camera` + `image_proc` rectification + `web_video_server`, preserving the original `/snapshot` URL shape. |
| `ros2_ws/src/ainex_gazebo` | The AiNex robot description + Gazebo Classic sim: URDF/xacro ported to `gazebo_ros2_control`, a flat world with a fixed camera, and a launch file. |
| `motiond/` | A dependency-free Python daemon that owns the vendor walking engine and serial servo bus, exposing a small newline-JSON Unix-socket protocol (walk commands, params, servo I/O, IMU, temps, battery). Runs the engine identically on hardware or, with `--sim`, streams joint angles to Gazebo with the servos unplugged. |
| `shims/` | A ROS-free client library (`motiond_client`) and drop-in compatibility layer (`compat`, `imu_source`) so mission code can talk to the daemon instead of rospy — the pathway that makes application code portable across ROS versions. |
| `tests/` | Hardware and end-to-end smoke tests. |

## What's deliberately NOT included

- **The vendor's original ROS 1 source.** The ports here are derived from the
  Hiwonder AiNex SDK that ships on the robot's SD card. That upstream code is
  Hiwonder's and is not redistributed here — obtain it from your own robot.
  These packages reference vendor config files (calibration, gait params,
  init pose) by their on-robot paths; supply your robot's own copies.
- **Application/mission logic and perception intelligence.** This repo is the
  motion/sim substrate only.

## Architecture

```
  ROS 2 tools ──► ainex_bridge (facade) ──┐
                                          ├─► motiond ──► walking engine ──► serial servo bus
  mission code ──► shims/compat ──────────┘        └──(--sim)──► UDP ──► sim_adapter ──► Gazebo
```

The motion daemon is the single owner of the serial bus (servos + IMU live on
the same STM32 link), so everything that needs the hardware goes through it.
Because the daemon speaks a plain socket protocol rather than ROS, the same
mission code runs unchanged whether the middleware underneath is ROS 1, ROS 2,
or nothing.

## Build

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build
```

## Status

Ported and desktop-verified; motion-daemon → engine chain validated on
hardware (real walking engine driven from ROS 2, servos safely unplugged).
Per-package `README.md` files carry apt dependencies and hardware-verify
checklists.

## License

The ported packages are provided as-is. AiNex, Hiwonder, and the upstream SDK
are trademarks/property of their respective owners; this is an independent
port, not affiliated with or endorsed by Hiwonder.
