# ainex_bridge

ROS2 Humble facade for the AiNex humanoid. Replicates the vendor ROS1
`ainex_controller` topic/service surface and forwards everything over a Unix
socket (`/tmp/motiond.sock`, newline-delimited JSON) to the local **motiond**
daemon, which owns the walking engine and servo bus. This node contains no
motion logic.

## Surface table

| ROS name | Kind | Type | motiond op |
|---|---|---|---|
| `/walking/command` | service | `ainex_interfaces/srv/SetWalkingCommand` | `{"op":"command","command":...}` (start/stop/enable/disable/enable_control/disable_control) |
| `/walking/set_param` | service | `ainex_interfaces/srv/SetWalkingParam` | `{"op":"set_param","params":{...}}` (msg fields -> vendor yaml keys, 1:1 names) |
| `/walking/get_param` | service | `ainex_interfaces/srv/GetWalkingParam` | `{"op":"get_param"}` -> mapped into `WalkingParam` (`period_times` always 0, per vendor) |
| `/walking/is_walking` | service | `ainex_interfaces/srv/GetWalkingState` | `{"op":"state"}` -> `state = walking` |
| `/walking/is_walking` | pub topic | `std_msgs/Bool` | polls `{"op":"state"}` at 5 Hz, publishes on transitions (mirrors vendor `walk_state_pub`) |
| `/app/set_walking_param` | sub topic | `ainex_interfaces/msg/AppWalkingParam` | `{"op":"app_param","speed":..,"height":..,"x":..,"y":..,"angle":..}` |
| `/app/set_action` | sub topic | `std_msgs/String` | `{"op":"run_action","name":...}` |
| `/head_pan_controller/command` | sub topic | `ainex_interfaces/msg/HeadState` | `{"op":"servos","duration_ms":int(duration*1000),"positions":[[23,pulse]]}` |
| `/head_tilt_controller/command` | sub topic | `ainex_interfaces/msg/HeadState` | same, servo id 24 |
| `/ros_robot_controller/bus_servo/set_position` | sub topic | `ainex_bridge_interfaces/msg/SetBusServosPosition` | `{"op":"servos","duration_ms":int(duration*1000),"positions":[[id,pos],...]}` |
| `/ros_robot_controller/bus_servo/get_position` | service | `ainex_bridge_interfaces/srv/GetBusServosPosition` | `{"op":"get_servos","ids":[...]}` |

Head angle -> pulse conversion (from vendor `servo_controller.yaml`, servos
23/24: init=500, min=0, max=1000, not flipped):
`pulse = 500 + round(angle_rad * 1000 * 180 / (pi * 240))`, clamped to 0..1000.
`HeadState.position` is radians, `HeadState.duration` seconds.

## Parameters

- `socket_path` (default `/tmp/motiond.sock`)
- `request_timeout` seconds (default `2.0`)
- `state_poll_hz` (default `5.0`, `0` disables the walking-state publisher)

## Build

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ainex_interfaces ainex_bridge_interfaces ainex_bridge
```

## Run

```bash
source ros2_ws/install/setup.bash
ros2 launch ainex_bridge bridge.launch.py
# or directly:
ros2 run ainex_bridge bridge --ros-args -p socket_path:=/tmp/motiond.sock
```

Quick checks:

```bash
ros2 service call /walking/command ainex_interfaces/srv/SetWalkingCommand "{command: start}"
ros2 topic pub -1 /app/set_walking_param ainex_interfaces/msg/AppWalkingParam "{speed: 3, height: 0.025, x: 0.01, y: 0.0, angle: 0.0}"
ros2 service call /walking/is_walking ainex_interfaces/srv/GetWalkingState
```

## sim_adapter — Gazebo digital-twin joint stream

`sim_adapter` is a second executable in this package for simulation: when
motiond runs with `--sim`, it emits one UDP datagram per walking tick (JSON
`{"t": <monotonic float>, "joints": {"<urdf_joint>": <radians>, ...}}`, default
`127.0.0.1:9910` — schema documented in `motiond/README.md`, "Sim mode").
`sim_adapter` converts that stream into `std_msgs/Float64MultiArray` on
`joint_group_position_controller/commands` for the ainex_gazebo controller.

Pipeline: `motiond --sim` → UDP 9910 → `sim_adapter` → Gazebo. The same
socket protocol clients (this bridge included) keep working unchanged.

### Joint order contract (ainex_gazebo)

The array is published in the FIXED order of the **`joint_order`** string-array
parameter, which **must be supplied at launch and must exactly match the
`joints:` list (names and order) of the ainex_gazebo
`joint_group_position_controller`** — a mismatch silently commands the wrong
joints. The proposed convention (default in `sim_adapter.launch.py` and in the
node) is the motiond walking joints in vendor `joint_index` order, then head:

```
r_hip_yaw r_hip_roll r_hip_pitch r_knee r_ank_pitch r_ank_roll
l_hip_yaw l_hip_roll l_hip_pitch l_knee l_ank_pitch l_ank_roll
r_sho_pitch l_sho_pitch head_pan head_tilt
```

(These are the `ainex_description` URDF joint names.)

Behaviour:

- unknown joint names in a datagram are ignored;
- joints missing from a datagram hold their last value (seeded all zeros);
- publishes at most `publish_rate` Hz (default 50), coalescing faster input;
- stops publishing after `staleness_timeout` s (default 1.0) without
  datagrams, and resumes when the stream returns.

### Parameters (sim_adapter)

- `udp_port` (default `9910`), `udp_bind` (default `127.0.0.1`)
- `joint_order` (string array, contract above)
- `publish_rate` (default `50.0` Hz), `staleness_timeout` (default `1.0` s)

### Run (sim_adapter)

```bash
ros2 launch ainex_bridge sim_adapter.launch.py            # default joint order
ros2 launch ainex_bridge sim_adapter.launch.py joint_order:='[r_hip_yaw, ..., head_tilt]'
# or directly:
ros2 run ainex_bridge sim_adapter --ros-args -p udp_port:=9910
```

The datagram→ordered-array conversion is plain python
(`parse_datagram` / `merge_joint_positions` in `ainex_bridge/sim_adapter.py`,
importable without rclpy); the no-ROS smoke test lives at
`ros2_port/tests/test_sim_smoke.py` (`python3 tests/test_sim_smoke.py`).
