# AiNex ROS2 Port

- `ros1_src/` — vendor's complete ROS1 (Noetic) workspace, extracted from the SD backup image. Read-only reference.
- `ros2_ws/` — the ROS2 (Humble) workspace being built up package by package.

Build (desktop, amd64 iteration): `docker run --rm -v $PWD/ros2_ws:/ws -w /ws ros:humble bash -c "source /opt/ros/humble/setup.bash && colcon build"`

Status:
- [x] ainex_interfaces — 12 msgs + 10 srvs, builds clean (field defs cleaned of ROS1-isms)
- [x] motiond — py3.8 motion daemon: walking engine .so + IK + servo bus + action groups + IMU (calibrated) + servo temps, UDS JSON protocol
- [x] ainex_bridge (+ainex_bridge_interfaces) — ROS2 facade: full vendor topic/service surface -> motiond socket; builds clean, smoke-tested
- [x] ainex_camera — v4l2_camera + rectify + web_video_server launch; snapshot URL verified compatible; exposure tune pre-loaded
- [x] shims — ROS-free client library + drop-in MotionManager/GaitManager/ImuFeed for mission code; line-by-line MIGRATION.md
- [x] hardware bring-up phase 1 (2026-07-20): motiond runs the real walking engine on-robot (sim mode, servos unplugged); Humble container built on-Pi; END-TO-END PASS: ROS2 /walking/command -> facade -> daemon -> engine -> 60 live gait datagrams (tests/ros2_e2e_test.sh)
- [ ] hardware bring-up phase 2: muscles-on supervised test (motiond real mode owns serial; ROS1 stack stopped)
- [ ] burn-in vs ROS1 container
