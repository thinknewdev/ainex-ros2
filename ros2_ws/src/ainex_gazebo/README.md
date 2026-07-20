# ainex_gazebo (ROS2 Humble)

Gazebo **Classic 11** simulation of the Hiwonder AiNex humanoid, ported from the
ROS1 `ainex_simulations` packages (`ainex_description` + `ainex_gazebo`).
Vendor URDF/xacro and STL meshes are merged into this single package.

> **Gazebo Classic EOL note:** Gazebo Classic 11 reached end-of-life in
> January 2025 and receives no updates. The vendor xacro and plugins target
> Classic, so this package deliberately stays on Classic via
> `ros-humble-gazebo-ros-pkgs` rather than rewriting for gz-sim/Ignition.
> This is a local development tool only.

## Dependencies (apt, on top of ros-humble)

```
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-web-video-server \
  ros-humble-xacro
```

## Launch

```bash
# GUI (default)
ros2 launch ainex_gazebo sim.launch.py

# Headless (gzserver only — camera sensors still render server-side,
# needs at least software GL on GPU-less machines)
ros2 launch ainex_gazebo sim.launch.py headless:=true

# Custom video port / world
ros2 launch ainex_gazebo sim.launch.py video_port:=8081 world:=/path/to/my.world
```

The robot spawns at the origin, `z=0.25`, arms pre-posed
(`l_sho_roll=-1.403, l_el_yaw=-1.226, r_sho_roll=1.403, r_el_yaw=1.226` — same
pose the ROS1 launch set via `spawn_model -J`; here it is done with
ros2_control `initial_value`s because `spawn_entity.py` has no `-J`).

## Control interface (for the adapter team)

The ROS1 controller (`ainex_controller.py` in `gazebo_sim` mode) published one
`std_msgs/Float64` per joint to `/<joint>_controller/command`
(24 x `effort_controllers/JointPositionController`). The ROS2 equivalent is a
**single** group controller:

- Topic: `/joint_group_position_controller/commands`
- Type: `std_msgs/msg/Float64MultiArray`
- `data`: **24 joint positions in radians, in EXACTLY the fixed order below.**

### Fixed joint order (index -> joint)

| Index | Joint       | Index | Joint       | Index | Joint       | Index | Joint      |
|-------|-------------|-------|-------------|-------|-------------|-------|------------|
| 0     | r_hip_yaw   | 6     | l_hip_yaw   | 12    | r_sho_pitch | 18    | l_el_yaw   |
| 1     | r_hip_roll  | 7     | l_hip_roll  | 13    | l_sho_pitch | 19    | r_el_yaw   |
| 2     | r_hip_pitch | 8     | l_hip_pitch | 14    | l_sho_roll  | 20    | l_gripper  |
| 3     | r_knee      | 9     | l_knee      | 15    | r_sho_roll  | 21    | r_gripper  |
| 4     | r_ank_pitch | 10    | l_ank_pitch | 16    | l_el_pitch  | 22    | head_pan   |
| 5     | r_ank_roll  | 11    | l_ank_roll  | 17    | r_el_pitch  | 23    | head_tilt  |

Indices 0–13 match the walking-module `joint_index` map in the ROS1
`ainex_controller.py`; 14–23 follow the vendor servo-ID order (IDs 15–24).
The same list (single source of truth for the sim) lives in
`config/ainex_controllers.yaml`.

Quick test (all zeros except pre-posed arms):

```bash
ros2 topic pub --once /joint_group_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,-1.403,1.403, 0,0,-1.226,1.226, 0,0, 0,0]}"
```

Joint states are on `/joint_states` (via `joint_state_broadcaster`) — same
topic the ROS1 controller subscribed to in `gazebo_sim` mode.

## Topics

| Topic                                       | Type                          | Notes                                    |
|---------------------------------------------|-------------------------------|------------------------------------------|
| `/joint_group_position_controller/commands` | `std_msgs/Float64MultiArray`  | 24 radians, fixed order above            |
| `/joint_states`                             | `sensor_msgs/JointState`      | 200 Hz controller_manager                |
| `/sim_camera/image_raw`                     | `sensor_msgs/Image`           | fixed world camera, 640x480 @ 15 fps     |
| `/camera/image_raw`                         | `sensor_msgs/Image`           | robot head camera, 640x480 @ 30 fps      |
| `/imu_data`                                 | `sensor_msgs/Imu`             | gazebo_ros_imu_sensor on `imu_link`      |

## Dashboard video

`web_video_server` runs on port **8081** (`video_port` launch arg):

- Stream: `http://<host>:8081/stream?topic=/sim_camera/image_raw`
- Snapshot: `http://<host>:8081/snapshot?topic=/sim_camera/image_raw`
- Index of all image topics: `http://<host>:8081/`

## Porting decisions (ROS1 -> ROS2 Humble)

- `libgazebo_ros_control.so` -> `libgazebo_ros2_control.so`
  (`gazebo_ros2_control`), controllers yaml passed via plugin `<parameters>`.
- URDF `<transmission>` tags -> `<ros2_control type="system">` block with
  `gazebo_ros2_control/GazeboSystem` hardware plugin
  (`urdf/ros2_control.xacro`). The ROS1 effort interface + per-joint PID
  (`effort_controllers/JointPositionController`, gains in the old
  `position_controller.yaml`) is replaced by a **position** command interface —
  Gazebo enforces positions directly, so no PID tuning is needed and one
  `position_controllers/JointGroupPositionController` covers all 24 joints.
- `libgazebo_ros_imu.so` (model plugin, removed in ROS2) ->
  `libgazebo_ros_imu_sensor.so` attached to an `<imu>` sensor on `imu_link`,
  publishing `/imu_data` (remap of `~/out`), matching the ROS1 topic name.
- Head camera plugin: ROS1 camelCase params (`cameraName`, `imageTopicName`,
  `hackBaseline`, ...) -> Humble `camera_name` / `frame_name` /
  `hack_baseline`; topics become `/camera/image_raw` + `/camera/camera_info`.
- `package://ainex_description/...` -> `package://ainex_gazebo/...` (packages
  merged); `$(find ainex_description)` -> `$(find ainex_gazebo)`.
- Physics: vendor `max_step_size 0.01` @ 1000 Hz (a 10x realtime target) ->
  `0.001` @ 1000 Hz for stable 1x realtime with the 200 Hz controller manager.
- ROS1 `.launch` XML files (`empty_world/spwan_model/position_controller/
  gazebo.launch`) collapsed into one `launch/sim.launch.py` using the
  `gazebo_ros` gzserver/gzclient launch includes, `spawn_entity.py`, and
  chained controller spawners.
- World gains a fixed `sim_camera` model (static, 1.5 m in front of spawn) for
  the dashboard; the vendor world had no camera.

## Runtime verify checklist (not yet run — needs a Gazebo-capable host)

Build here only validated `colcon build` + xacro expansion + XML well-formedness.
On a machine with the apt deps installed:

1. `ros2 launch ainex_gazebo sim.launch.py headless:=true` — no red errors;
   `[gazebo_ros2_control]` logs "Loading controller_manager".
2. `ros2 control list_controllers` shows `joint_state_broadcaster` and
   `joint_group_position_controller` both `active`.
3. `ros2 topic hz /joint_states` ≈ 200 Hz; `ros2 topic echo /joint_states -n1`
   lists all 24 joints.
4. Publish the quick-test command above — robot holds pose / joints move in
   Gazebo (run GUI mode to observe; robot may need a supported squat pose from
   the adapter to stand, exactly as in ROS1).
5. `ros2 topic hz /sim_camera/image_raw` ≈ 15 Hz, `/camera/image_raw` ≈ 30 Hz.
6. Open `http://localhost:8081/stream?topic=/sim_camera/image_raw` — live MJPEG
   of the robot; `/snapshot?topic=/sim_camera/image_raw` returns a JPEG.
7. `ros2 topic echo /imu_data -n1` returns a sane orientation/accel
   (z ≈ +9.8 when standing).
8. Index order spot-check: publish a nonzero value at index 22 only
   (`head_pan`) and confirm only the head pans.
