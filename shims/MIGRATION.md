# Migration plan: the mission code & vision_service.py off rospy, onto motiond

Shim layer in this directory:

| file | provides |
|---|---|
| `motiond_client.py` | `MotiondClient` — reconnecting, thread-safe UDS/JSON client for every daemon op, plus the proposed `imu()` op |
| `compat.py` | `MotionManager`, `GaitManagerCompat`, `app_walk()`, `app_cmd()` — signature-identical replacements for the vendor classes / rospy touchpoints |
| `imu_source.py` | `ImuFeed` — background IMU poller with `_accel`/`_yaw`-equivalent surface |

Mission scripts add the shims to their path (or the shims get installed
alongside them):

```python
import sys
sys.path.insert(0, '/home/ubuntu/share/src/shims')   # wherever shims deploy on-robot
```

---

## Daemon protocol additions needed

### 1. `imu` — REQUIRED

```
{"op":"imu"} -> {"ok":true,"accel":[x,y,z],"gyro":[x,y,z]}
```

Why: the IMU is on the STM32 servo board, read over the **same serial link as
the servos** (`Board.get_imu()` in
`ros1_src/ainex_driver/ros_robot_controller/src/ros_robot_controller/ros_robot_controller_sdk.py`,
raw units g and deg/s). It is not an I2C/sysfs device — only the process
owning the serial port (motiond) can read it, so it must be a daemon op.

Required daemon-side semantics (to serve what `/imu_corrected` carried):

* `accel` in **m/s²**: raw board value × 9.80665
  (`ros_robot_controller_node.py::pub_imu_data`, `gravity = 9.80665`).
  Upright gravity lands on **+Y ≈ +9.8** — all of the mission code's fall/tilt math
  assumes this axis convention.
* `gyro` in **rad/s**: `math.radians(raw)` per axis.
* Apply the `imu_calib` scale/bias correction from
  `ainex_calibration/config/imu_calib.yaml` (that correction is exactly what
  turned `/ros_robot_controller/imu_raw` into `/imu_corrected` in
  `ainex_peripherals/launch/imu.launch`). If the daemon skips it, tilt/fall
  thresholds still roughly work but the gyro-measured turns (`turn_by`) will
  carry the uncalibrated bias.
* Return the **latest** sample (board streams ~100 Hz); clients poll at ~50 Hz.

### 2. `run_action` blocking — REQUIRED SEMANTIC (no new op)

The vendor `MotionManager.run_action()` **blocks until the .d6a finishes
playing**. Mission code depends on it: `recover_if_fallen()` runs
`mm.run_action('recline_to_stand'); time.sleep(1.2)` — the sleep is padding,
not the duration. The daemon's `run_action` must not reply `{"ok":true}`
until the action group completes (or a `{"op":"action_state"}` op must be
added and the compat shim updated to poll it).

### 3. `servo_temp` — OPTIONAL (graceful degradation exists)

```
{"op":"servo_temp","ids":[1,2,...]} -> {"ok":true,"temps":[[1,52],[2,49]]}   (°C)
```

`the mission code.servo_temps_ok()` reads servo temperatures via the
`/ros_robot_controller/bus_servo/get_state` service (`get_temperature=1`) to
refuse runs on cooked ankles and to set `SPEED_CAP`. There is no daemon op
for it. Its `except` branch already prints "temp check unavailable —
proceeding", so migration works without it — but the hot-legs safety gate is
lost until this op is added. Recommended.

---

## the mission code — line-by-line

Legend: `compat` = `from compat import ...`, `client` = the shared
`MotiondClient` behind it.

| line(s) | current (rospy / vendor) | replacement |
|---|---|---|
| 14 | `import ... rospy` | drop `rospy` from the import list |
| 19 | `from ainex_kinematics.gait_manager import GaitManager` | `from compat import GaitManagerCompat as GaitManager` |
| 20 | `from ainex_kinematics.motion_manager import MotionManager` | `from compat import MotionManager` |
| 21 | `from sensor_msgs.msg import Imu` | delete (replaced by `ImuFeed`) |
| 22 | `from ainex_interfaces.msg import AppWalkingParam` | delete |
| 23 | `from ainex_interfaces.srv import SetWalkingCommand` | delete |
| 139 | `app_pub = None` (module global) | delete (no publisher object anymore) |
| 291–295 | `def app_cmd(c): rospy.ServiceProxy('/walking/command', SetWalkingCommand)(c)` | `from compat import app_cmd` (same name, same swallow-and-print contract) — delete the local def |
| 297–310 | `def app_walk(x, angle, y=0.0):` builds `AppWalkingParam` (speed=APP_SPEED, height=APP_HEIGHT) and `app_pub.publish(msg)` | keep the local def but replace the msg+publish body with `compat.app_walk(x, angle, y=y, speed=APP_SPEED, height=APP_HEIGHT)`; **keep** the `urllib` `/hint` POST that follows — it is HTTP to the vision service, not ROS |
| 348 | `_accel = [0.0, 9.8, 0.0]` global | `imu = ImuFeed()` (module level); replace every read of `_accel` with `imu.accel` — sites: `tilt_deg()` L366, `SwayTracker.sample()` L378, `roll_deg()` L419, `fall_state()` L424+427, `recover_if_fallen()` L438, `imucheck` mode L1982 |
| 350 | `_yaw = {'deg': 0.0, 't': None}` | replace reads `_yaw['deg']` with `imu.gyro_deg` and resets `_yaw['deg'] = 0.0` with `imu.gyro_deg = 0.0` — sites: L646 (`turn_by` reset), L647–653 (turn loop reads), L871/L1015 (grab-gate heading), L1270/L1358/L1371 (cruise world-frame fusion), L1486/L1501 (proprioception) |
| 352–360 | `def _imu_cb(msg): ...` accel copy + yaw integration | delete — `ImuFeed`'s poll thread does both (same integrator: `deg += degrees(gyro_y) * dt`) |
| 579 | `while ... and not rospy.is_shutdown():` | `while time.time() < t_end:` (Ctrl-C still raises KeyboardInterrupt; SIGTERM handler at L1937 already exists) |
| 702–714 | `neutralize_gait()` mutates `gait.walking_param.<field>` and `gait.param_pub.publish(gp)` | **unchanged** — `GaitManagerCompat.walking_param` is attribute-style and `param_pub.publish()` maps to the daemon `set_param` op |
| 716–724 | `walk_forward()` uses `gait.walking_param`, `gait.set_step(...)`, `gait.get_gait_param()`, `gait.stop()` | **unchanged** — all reproduced on `GaitManagerCompat` (`set_step` maps to `set_param` + `command('start')`, waits on `state()['walking']` for `step_num != 0`) |
| 454–458 | `gait.stop() / gait.disable() / gait.enable()` in `recover_if_fallen` (and every other `gait.*` call: L790, L1029, L1031, L1139, L1889, L1914, L1938, L1962, L1967, L1989, L2002, L2011, L2018) | **unchanged** — mapped to `command('stop'/'disable'/'enable')` |
| 520, 1666–1720, 1874–1906 | `mm.set_servos_position(dur, [[id,pos],...])`, `mm.run_action(name)` | **unchanged** — compat `MotionManager` keeps the exact vendor varargs signature, backed by `servos` / `run_action` ops |
| 1731–1758 | `servo_temps_ok()` imports `ros_robot_controller.srv/.msg`, `rospy.ServiceProxy('/ros_robot_controller/bus_servo/get_state')` | replace body with the optional `servo_temp` daemon op when added; until then let the existing `except` fall through ("temp check unavailable — proceeding"). Simplest interim: wrap the whole body in `try` and return `True` |
| 1924 | `rospy.init_node('the mission code', anonymous=True)` | delete |
| 1925 | `rospy.Subscriber('/imu_corrected', Imu, _imu_cb, queue_size=1)` | `imu = ImuFeed()` (if not module-level) — requires the daemon `imu` op |
| 1926 | `app_pub = rospy.Publisher('/app/set_walking_param', AppWalkingParam, queue_size=1)` | delete (compat `app_walk` handles it) |
| 1927 | `gait = GaitManager()` | `gait = GaitManager()` — same line, now the compat class via the aliased import |
| 1928 | `mm = MotionManager(ACTIONS)` | **unchanged** (compat accepts and ignores the path; the daemon resolves action names) |
| 1929 | `time.sleep(0.3)  # let one IMU sample arrive` | `imu.wait_ready(2.0)` (actually verifies data is flowing) |

Everything else in the mission code (vision HTTP, narrator, banter, body-daemon
claim, MediaPipe) is ROS-free already and unchanged.

## vision_service.py — line-by-line

| line(s) | current | replacement |
|---|---|---|
| 19 | docstring "Run INSIDE the container (needs ROS for the head servos)" | update: needs only motiond |
| 28 | `import rospy` | delete |
| 29 | `from ainex_kinematics.motion_manager import MotionManager` | `from compat import MotionManager` |
| 126–128 | `head(pan, tilt, dur)` → `mm.set_servos_position(dur, [[PAN_ID, ...], [TILT_ID, ...]])` | **unchanged** — identical signature on the compat class |
| 805 | `rospy.init_node('vision_service', anonymous=True)` | delete |
| 806 | `mm = MotionManager(ACTIONS)` | **unchanged** (compat) |

That is vision_service's entire ROS surface: node init + head servo writes.

---

## Behavioral notes / semantic diffs to verify on hardware

1. **`is_walking` freshness** — vendor kept it hot via a subscriber; compat
   polls `state()` per read (~sub-ms on UDS). `set_step(step_num=N)` polls at
   100 Hz like the vendor's loop; a 5 s guard prevents an infinite wait if
   the engine never reports walking.
2. **`app_walk` failure mode** — vendor `publish()` on a dead node silently
   dropped; compat raises `MotiondError` if the daemon is down. Every mission
   call site already wraps `app_walk` in try/except or tolerates the raise
   (`app_halt` does), but confirm during the first HW run.
3. **Yaw integration cadence** — ROS delivered ~100 Hz IMU callbacks with
   board timestamps; `ImuFeed` polls at 50 Hz using wall-clock dt. Turn-glide
   constants (`_glide`) were learned live and self-correct, but expect the
   first `turn_by` of a run to land slightly differently.
4. **`GaitManagerCompat()` constructor** — like the vendor (which blocked on
   `wait_for_service('walking/get_param')`), it needs the daemon up: it
   fetches `get_param` at construction and raises `MotiondError` otherwise.
