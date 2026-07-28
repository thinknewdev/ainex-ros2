# AiNex ROS2 Port

ROS 2 (Humble) port of the Hiwonder AiNex humanoid: message/service interfaces, a ROS 2
bridge/facade, camera, a Gazebo sim, and a standalone motion daemon (`motiond`) with ROS-free
client shims so existing mission code runs against ROS 2 unchanged.

- `ros2_ws/` — the ROS2 (Humble) workspace: interfaces, bridge, camera, gazebo.
- `motiond/` — standalone motion daemon (walking engine + IK + servo bus + IMU + servo temps).
- `shims/` — ROS-free client + drop-in MotionManager/GaitManager/ImuFeed (see MIGRATION.md).
- `deploy/` — on-robot deployment: camera-runtime Dockerfile, systemd service, and the ROS1 -> ROS2 cutover runbook (`CUTOVER.md`).
- `tests/` — end-to-end test scripts.

Build (desktop, amd64 iteration): `docker run --rm -v $PWD/ros2_ws:/ws -w /ws ros:humble bash -c "source /opt/ros/humble/setup.bash && colcon build"`

Deploy on the robot: see `deploy/CUTOVER.md` for the full ROS1 -> ROS2 procedure.

Status:
- [x] ainex_interfaces — 12 msgs + 10 srvs, builds clean (field defs cleaned of ROS1-isms)
- [x] motiond — py3.8 motion daemon: walking engine .so + IK + servo bus + action groups + IMU (calibrated) + servo temps, UDS JSON protocol
- [x] ainex_bridge (+ainex_bridge_interfaces) — ROS2 facade: full vendor topic/service surface -> motiond socket; builds clean, smoke-tested
- [x] ainex_camera — v4l2_camera + rectify + web_video_server launch; snapshot URL verified compatible; exposure tune pre-loaded
- [x] shims — ROS-free client library + drop-in MotionManager/GaitManager/ImuFeed for mission code; line-by-line MIGRATION.md
- [x] hardware bring-up phase 1 (2026-07-20): motiond runs the real walking engine on-robot (sim mode, servos unplugged); Humble container built on-Pi; END-TO-END PASS: ROS2 /walking/command -> facade -> daemon -> engine -> 60 live gait datagrams (tests/ros2_e2e_test.sh)
- [x] ainex_camera on-robot (2026-07-28): deps built into a Humble runtime container with camera-device access; `ainex_camera` colcon-built on the Pi; serves the legacy `:8080/snapshot` endpoint unchanged
- [x] **full cutover to pure ROS2 (2026-07-28)**: ROS1 `bringup.launch` stopped and boot-disabled (`start_node`); ROS2 camera runs as a managed service; camera + vision (live object detection), motion telemetry (IMU/battery), and motion commands (head move) all verified on the pure-ROS2 stack. Robot boots to ROS2 with no ROS1 processes. See `deploy/CUTOVER.md`.
- [x] **ROS2 motion bridge live + ROS1 fully severed (2026-07-28)**: `motiond` relocated to a shared socket (with a `/tmp` symlink for legacy consumers) so `ainex_bridge` exposes the full ROS2 motion topic/service surface; `start_node` masked, its hard-dependency stripped from all other units, and the vendor web api made master-optional (runs pure-motiond). **Cold power cycle verified: boots 100% ROS2, zero ROS1 processes.**
- [ ] walking under load re-tune on the ROS2 stack (supervised, fall-capable)
- [ ] burn-in vs ROS1 container
