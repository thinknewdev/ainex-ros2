# motiond — AiNex standalone motion daemon (no ROS)

`motiond.py` is a single-process Python 3.8 daemon that owns the AiNex's
walking engine and the serial servo bus. It exists because the vendor's
walking engine (`walking_module.so`) and leg IK (`kinematics.so`) are
compiled CPython **3.8** aarch64 extensions with no ROS linkage, while ROS2
Humble runs Python 3.10 — so the binaries must live in their own py3.8
process. A ROS2 facade node talks to this daemon over a local Unix socket.

It is a faithful port of the ROS1 `ainex_controller.py` node (engine init,
8 ms tick loop, command semantics, app parameter mapping) plus
`motion_manager.py`'s `.d6a` action-group player, with all servo IO going
directly through the pure-Python board SDK
(`ros_robot_controller_sdk.py`) instead of ROS topics/services.

## Requirements (on the robot)

- Python **3.8** (the container/env the vendor `.so` files were built for).
- `pyserial` and `PyYAML` installed in that env.
- On `PYTHONPATH`:
  - `walking_module.so` and `kinematics.so` — either importable as
    `ainex_kinematics.walking_module` / `ainex_kinematics.kinematics`
    (the stock ROS1 tree layout) or as bare top-level modules.
  - `ros_robot_controller_sdk.py` — importable as
    `ros_robot_controller.ros_robot_controller_sdk` or as a bare module.
- Access to the servo board serial device (default `/dev/rrc`, 1 Mbaud).
- The stock config yamls and action groups (defaults below).

The ROS1 `ros_robot_controller` node must NOT be running at the same time —
exactly one process may own the serial bus.

## Running

```sh
python3.8 motiond.py \
    --walking-param    /home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/config/walking_param.yaml \
    --walking-offset   /home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/config/walking_offset.yaml \
    --init-pose        /home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/config/init_pose.yaml \
    --servo-controller /home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/config/servo_controller.yaml \
    --action-path      /home/ubuntu/software/ainex_controller/ActionGroups \
    --imu-calib        /home/ubuntu/ros_ws/src/ainex_calibration/config/imu_calib.yaml \
    --serial-device    /dev/rrc \
    --socket           /tmp/motiond.sock
```

Two additional flags select simulation mode (see "Sim mode" below):
`--sim` (no serial bus; joint goals streamed over UDP for the Gazebo twin)
and `--sim-udp-port` (default 9910).

All arguments are optional; the values above are the defaults (the robot's
stock paths). On startup the daemon drives all servos to the recorded init
pose over 1 s, initializes `LegIK` + `WalkingModule`, then runs the tick
loop on the main thread and the socket server on a background thread.
`SIGTERM`/`SIGINT` stop the walking engine and close the serial port cleanly.

### systemd unit example

```ini
[Unit]
Description=AiNex motion daemon (py3.8 walking engine + servo bus)
After=multi-user.target

[Service]
Type=simple
User=ubuntu
Environment=PYTHONPATH=/home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/src:/home/ubuntu/ros_ws/src/ainex_driver/ros_robot_controller/src
ExecStart=/usr/bin/python3.8 /home/ubuntu/motiond/motiond.py
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

(If the daemon runs inside a py3.8 Docker container instead, wrap the
`docker run`/`docker exec` invocation in `ExecStart` and bind-mount the
serial device and `/tmp` so the socket is visible to the ROS2 side.)

## Protocol

Unix domain socket `/tmp/motiond.sock`, newline-delimited JSON, multiple
concurrent clients. One request object per line, one response object per
line. Every response has `"ok": true` or `"ok": false, "err": "..."`.

| Request | Response | Notes |
|---|---|---|
| `{"op":"command","command":"start"}` | `{"ok":true}` | Enable walking and start the gait. |
| `{"op":"command","command":"stop"}` | `{"ok":true}` | **Blocks** until the current step finishes and the engine reports stopped. |
| `{"op":"command","command":"enable"}` | `{"ok":true}` | Re-enable the walking tick without starting the gait. |
| `{"op":"command","command":"disable"}` | `{"ok":true}` | Blocking stop, then disable the walking tick. |
| `{"op":"command","command":"enable_control"}` | `{"ok":true}` | Sets the `init_pose_finish` gate (tick loop may run). |
| `{"op":"command","command":"disable_control"}` | `{"ok":true}` | Clears the gate; tick loop idles, servos untouched. |
| `{"op":"app_param","speed":3,"height":0.025,"x":0.0,"y":0.0,"angle":0.0}` | `{"ok":true}` | Vendor phone-app mapping (speed 1–4 selects period/dsp/swap presets, applies walking_offset.yaml corrections) plus the lean fix below. Clamps applied here. |
| `{"op":"set_param","params":{...}}` | `{"ok":true}` | Partial or full walking-param update; vendor clamps applied; pushed to the engine. |
| `{"op":"get_param"}` | `{"ok":true,"params":{...}}` | Live values pulled from the engine (`period_times` reported as 0, as vendor did). |
| `{"op":"servos","duration_ms":200,"positions":[[23,500],[24,500]]}` | `{"ok":true}` | Raw servo write (head ids 23/24 included — the facade converts angles to pulses itself). |
| `{"op":"get_servos","ids":[1,2]}` | `{"ok":true,"positions":[[1,512],[2,498]]}` | Real bus read; errors if a servo does not answer. |
| `{"op":"run_action","name":"stand"}` | `{"ok":true}` | **Blocks**: stop walking → disable → play the `.d6a` group to its final frame → re-init pose (incl. the 0.5 s settle) → re-enable. The reply is only sent after all of that. Errors if the file is missing or an action is already running. |
| `{"op":"imu"}` | `{"ok":true,"accel":[x,y,z],"gyro":[x,y,z]}` | Calibrated IMU, matching the ROS1 `/imu_corrected` topic. `accel` in m/s² (upright gravity ≈ +9.8 on Y), `gyro` in rad/s. See "IMU calibration" below. Errors with `no imu data ...` during the startup gyro-bias window or if the board sends no frames. In `--sim`: always `{"ok":false,"err":"sim mode"}`. |
| `{"op":"servo_temp","ids":[1,2]}` | `{"ok":true,"temps":[[1,42],[2,39]]}` | Servo temperatures in °C via the bus (same read the ROS1 `bus_servo/get_state` path used); errors if a servo does not answer. In `--sim`: fixed 35 °C. |
| `{"op":"state"}` | `{"ok":true,"walking":bool,"enabled":bool,"stopped":bool}` | `walking` is the vendor's `not stop`: true from `start` until the gait *actually* finishes. |
| anything else | `{"ok":false,"err":"..."}` | Malformed JSON, unknown op, bad fields, bus errors. |

Parameter clamps (owned by the daemon, not the facade): body height
(`init_z_offset`) 0.015–0.06 m, `x/y_move_amplitude` ±0.05 m,
`z_move_amplitude` 0–0.05 m, `angle_move_amplitude` ±10°,
`y_swap_amplitude` 0–0.05 m, `arm_swing_gain` 0–60° (radians).

Quick smoke test from the shell:

```sh
printf '{"op":"state"}\n' | nc -U /tmp/motiond.sock
printf '{"op":"app_param","speed":3,"height":0.025,"x":0.01,"y":0,"angle":0}\n{"op":"command","command":"start"}\n' | nc -U /tmp/motiond.sock
printf '{"op":"command","command":"stop"}\n' | nc -U /tmp/motiond.sock
```

## Sim mode (`--sim`)

`--sim` runs the SAME vendor walking engine and daemon protocol against a
Gazebo digital twin instead of hardware:

```sh
python3.8 motiond.py --sim --sim-udp-port 9910 [other flags as usual]
```

- The serial bus is **never opened** (`ros_robot_controller_sdk` does not even
  need to be importable). The walking `.so` files and the config yamls are
  still required — the gait math is real.
- Servo writes become no-ops recorded in an in-memory dict (last commanded
  pulse per id, seeded by the startup init-pose write). `{"op":"get_servos"}`
  answers from that dict; an id that was never written errors like a servo
  that does not answer.
- `{"op":"servo_temp"}` returns a fixed **35 °C** for every requested id.
- `{"op":"imu"}` returns `{"ok":false,"err":"sim mode"}` (a later phase may
  loop back a simulated IMU from Gazebo).
- Everything else (commands, params, actions, state) behaves as on hardware.

### Joint-stream datagrams

Each tick that would have written servos instead emits **one UDP datagram** to
`127.0.0.1:<--sim-udp-port>` (default **9910**), consumed by the
`ainex_bridge` `sim_adapter` ROS2 node:

```json
{"t": 12.345678, "joints": {"r_hip_yaw": 0.01, "l_knee": -0.62, "...": 0.0}}
```

- `t` — `time.monotonic()` at emission (float seconds; ordering/staleness
  only, not an epoch).
- `joints` — goal position in **radians** per joint, keyed by the URDF joint
  names from `ainex_description` (which match the daemon's internal names:
  `r_hip_yaw`, `r_hip_roll`, `r_hip_pitch`, `r_knee`, `r_ank_pitch`,
  `r_ank_roll`, the `l_*` mirrors, `r_sho_pitch`, `l_sho_pitch`, plus
  `head_pan` / `head_tilt`).
- A tick datagram carries **every joint the tick computed** (all 14 walking
  joints) merged with the last known head values. Note: when
  `arm_swing_gain == 0` the hardware path skips the `*_sho_pitch` writes but
  the datagram still carries the engine's computed values for them.
- A `{"op":"servos"}` write that includes head ids **23/24** immediately emits
  an extra datagram containing just the head joints, converted pulse→radian
  with the `servo_controller.yaml` coefficients (init 500, not flipped), under
  the URDF names `head_pan` / `head_tilt`. Those values are also merged into
  every subsequent tick datagram.
- Not (yet) streamed: `.d6a` action-group frames and non-head raw servo
  writes — they update the recorded-pulse dict only. A later phase may
  generalize the pulse→radian loopback to all joints.

Emission is fire-and-forget UDP on loopback; if nobody listens, datagrams are
dropped silently.

## IMU calibration

The IMU is on the STM32 servo board, on the same serial bus, so motiond
serves it too. The board auto-pushes IMU frames; a background thread polls
the SDK queue at ~100 Hz under the bus lock (a non-blocking queue read, no
serial round-trip — servo IO is never starved) and keeps the latest
calibrated sample for `{"op":"imu"}`.

The correction reproduces the ROS1 pipeline that produced `/imu_corrected`
(`ainex_peripherals/launch/imu.launch` → `imu_calib apply_calib`):

1. Scale the raw board report to SI: `accel = raw_g × 9.80665`,
   `gyro = radians(raw_deg_s)` (what `/ros_robot_controller/imu_raw` carried).
2. Accel calibration from `imu_calib.yaml`: `accel = SM(3×3) · accel − bias`.
3. Online gyro bias: the mean of the first 100 samples (robot stationary —
   it is, right after the init-pose settle) is subtracted from all later
   samples, exactly like `apply_calib` with its defaults
   (`calibrate_gyros: true`, `gyro_calib_samples: 100`). During that
   ~1–2 s window `{"op":"imu"}` returns an error, as the ROS1 node
   published nothing then either.

If the calib yaml is missing or invalid, motiond logs a startup warning and
degrades to step 1 only (identity SM, zero accel bias); the gyro bias
estimation still runs. The yaml path is `--imu-calib` (robot default above).

## Deliberate divergence: `init_x_offset` is NOT zeroed by app commands

The vendor's `set_app_walking_param_callback` hardcodes
`walking_param['init_x_offset'] = 0` on every app gait command, discarding
the tuned forward-lean offset stored in `walking_param.yaml` (this robot's
yaml carries `init_x_offset: -0.012`). Zeroing it makes the robot walk
without its calibrated lean and degrades gait stability.

**motiond keeps the yaml-loaded `init_x_offset`** (captured at startup)
whenever `{"op":"app_param"}` is processed. All other vendor-fixed values
in that mapping are preserved exactly as the vendor set them
(`init_y/roll/pitch/yaw_offset = 0`, `hip_pitch_offset = 15`,
`z_move_amplitude = 0.02`, `pelvis_offset = 5`, `arm_swing_gain = 0.5`,
then the per-speed presets).

Two smaller intentional deviations from ROS1, for robustness:

- The blocking waits in `stop`/`disable`/`run_action` time out after 10 s
  instead of hanging forever if the gait never settles (the vendor could
  deadlock if `stop` was issued while walking was disabled).
- The vendor's `move_to_init_pose` debug-file write for bad servo 11/12
  reads referenced a loop index instead of the value (would have crashed);
  motiond logs the suspicious read and applies the same clamp-to-500 logic.

## Things to verify on hardware (`# PORT-VERIFY` in the source)

- `bus_servo_read_position` return shape (`[position]` sequence).
- `/dev/rrc` udev alias exists inside the py3.8 container.
- `WalkingModule.get_walking_param()` returns a plain dict with the same
  keys as `walking_param.yaml`.
- The STM32 auto-reports IMU frames once `enable_reception(True)` is set
  (the ROS1 node relied on the same behaviour; if `{"op":"imu"}` keeps
  returning `no imu data`, the firmware may need an explicit report-enable).
- `bus_servo_read_temp` returns plain degrees C (`"<BBbB"` tail).
