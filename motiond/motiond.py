#!/usr/bin/env python3
# encoding: utf-8
#
# motiond.py - standalone AiNex motion daemon (NO ROS).
#
# Owns the vendor walking engine (walking_module.so + kinematics.so, CPython 3.8
# aarch64 builds) and the serial servo bus, and serves a newline-delimited JSON
# protocol on a Unix domain socket so a ROS2 (py3.10) facade node can drive the
# robot from a separate process.
#
# This is a faithful port of the ROS1 node
#   ainex_kinematics/scripts/ainex_controller.py  (engine init + 8ms tick loop)
#   ainex_kinematics/src/ainex_kinematics/motion_manager.py  (action groups)
# with all servo IO going directly through the pure-Python board SDK
#   ros_robot_controller/src/ros_robot_controller/ros_robot_controller_sdk.py
# instead of ROS topics/services.
#
# Python 3.8 compatible. Do not import rospy/rclpy here, ever.

import os
import sys
import json
import time
import copy
import math
import errno
import socket
import signal
import sqlite3
import argparse
import threading
from typing import Any, Dict, List, Optional

import yaml

# ---------------------------------------------------------------------------
# Imports of robot-only binaries / SDK.
#
# On the robot the vendor .so files live in the ainex_kinematics python package
# (from ainex_kinematics.kinematics import LegIK). When running motiond outside
# the ROS1 tree, the two .so files can instead sit directly on PYTHONPATH.
# ---------------------------------------------------------------------------
try:
    from ainex_kinematics.kinematics import LegIK
    from ainex_kinematics.walking_module import WalkingModule
except ImportError:
    from kinematics import LegIK              # type: ignore
    from walking_module import WalkingModule  # type: ignore

try:
    from ros_robot_controller.ros_robot_controller_sdk import Board
except ImportError:
    try:
        from ros_robot_controller_sdk import Board  # type: ignore
    except ImportError:
        # Only tolerable in --sim mode (no serial bus is opened there).
        # ServoBus.__init__ raises a clear error if the SDK is truly needed.
        Board = None  # type: ignore


def load_yaml(path):
    # type: (str) -> Dict[str, Any]
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def log(*args):
    print('[motiond]', *args, flush=True)


class JointPosition:
    # Same tiny struct the ROS1 controller used for present/goal joint state.
    def __init__(self):
        self.present_position = 0.0
        self.goal_position = 0.0


class ServoBus:
    """All servo IO for the daemon, straight through the serial board SDK.

    Replaces MotionManager's ROS topic/service path with direct
    Board.bus_servo_set_position / bus_servo_read_position calls.
    """

    def __init__(self, device='/dev/rrc', baudrate=1000000, timeout=5):
        # type: (str, int, int) -> None
        if Board is None:
            raise ImportError('ros_robot_controller_sdk is not on PYTHONPATH '
                              '(required unless running with --sim)')
        self.board = Board(device=device, baudrate=baudrate, timeout=timeout)
        # Reads only work while the SDK's receive thread is parsing frames.
        # The ROS1 ros_robot_controller node enabled this at startup too.
        self.board.enable_reception(True)
        # Serialize bus access between the tick loop and socket handlers.
        # (The SDK only locks reads internally; writes are raw port.write.)
        self._lock = threading.Lock()
        self.servo_position = {}  # type: Dict[int, int]  # last commanded pulse

    def set_servos_position(self, duration_ms, positions):
        # type: (float, List[List[int]]) -> None
        """duration_ms: milliseconds (0 = fast path used by the tick loop).
        positions: [[id, pulse], ...]"""
        with self._lock:
            for servo_id, pulse in positions:
                self.servo_position[int(servo_id)] = int(pulse)
            # SDK takes seconds; ROS1 MotionManager did duration/1000.0 as well.
            self.board.bus_servo_set_position(float(duration_ms) / 1000.0, positions)

    def get_servos_position(self, ids):
        # type: (List[int]) -> List[List[int]]
        """Read real positions for the given servo ids. [[id, pulse], ...]
        Raises IOError if a servo does not answer after the SDK's retries."""
        data = []
        with self._lock:
            for servo_id in ids:
                # PORT-VERIFY: bus_servo_read_position returns the unpacked
                # "<BBbh" tail, i.e. a 1-element sequence [position]; verify a
                # real read on hardware returns e.g. (512,) and not a scalar.
                result = self.board.bus_servo_read_position(servo_id)
                if result is None:
                    raise IOError('servo %d position read failed' % servo_id)
                data.append([int(servo_id), int(result[0])])
        return data

    def get_servos_temp(self, ids):
        # type: (List[int]) -> List[List[int]]
        """Read temperatures (deg C) for the given servo ids. [[id, temp], ...]
        Same bus read the ROS1 bus_servo/get_state service path used."""
        data = []
        with self._lock:
            for servo_id in ids:
                # PORT-VERIFY: bus_servo_read_temp unpacks "<BBbB" -> [temp];
                # confirm the value is plain degrees C on hardware.
                result = self.board.bus_servo_read_temp(servo_id)
                if result is None:
                    raise IOError('servo %d temperature read failed' % servo_id)
                data.append([int(servo_id), int(result[0])])
        return data

    def get_servos_vin(self, ids):
        # type: (List[int]) -> List[List[int]]
        """Read voltages (mV) for the given servo ids. [[id, mv], ...]"""
        data = []
        with self._lock:
            for servo_id in ids:
                result = self.board.bus_servo_read_vin(servo_id)
                if result is None:
                    raise IOError('servo %d vin read failed' % servo_id)
                data.append([int(servo_id), int(result[0])])
        return data

    def get_battery(self):
        # type: () -> Optional[int]
        """Battery voltage in mV from the board."""
        with self._lock:
            return self.board.get_battery()

    def get_imu(self):
        # type: () -> Optional[tuple]
        """Newest raw IMU report, or None if no fresh frame has arrived.
        Returns (ax, ay, az, gx, gy, gz[, mx, my, mz]) in board units:
        accel in g, gyro in deg/s (scaling to SI happens in ImuCorrector,
        matching the ROS1 ros_robot_controller_node's imu_raw topic).

        Board.get_imu() only drains the SDK's internal queue — the STM32
        pushes IMU frames spontaneously and the recv thread parses them, so
        this costs no serial round-trip. We still take the bus lock so IMU
        polling can never interleave with a servo read transaction.
        """
        with self._lock:
            return self.board.get_imu()

    def close(self):
        # type: () -> None
        try:
            self.board.port.close()
        except Exception as e:
            log('serial close error:', e)


class SimServoBus:
    """Drop-in replacement for ServoBus in --sim mode.

    No serial port is opened. Servo writes are no-ops recorded in an
    in-memory dict (last commanded pulse per id, seeded by the daemon's
    startup init-pose write); position reads answer from that dict;
    temperatures are a fixed 35 deg C; there is no IMU (a later phase may
    loop back a simulated IMU from Gazebo).
    """

    SIM_SERVO_TEMP_C = 35

    def __init__(self):
        # type: () -> None
        self._lock = threading.Lock()
        self.servo_position = {}  # type: Dict[int, int]  # last commanded pulse

    def set_servos_position(self, duration_ms, positions):
        # type: (float, List[List[int]]) -> None
        with self._lock:
            for servo_id, pulse in positions:
                self.servo_position[int(servo_id)] = int(pulse)

    def get_servos_position(self, ids):
        # type: (List[int]) -> List[List[int]]
        data = []
        with self._lock:
            for servo_id in ids:
                if int(servo_id) not in self.servo_position:
                    # Mirrors the real bus contract: a servo that was never
                    # commanded "does not answer".
                    raise IOError('servo %d position read failed (sim: never written)'
                                  % servo_id)
                data.append([int(servo_id), self.servo_position[int(servo_id)]])
        return data

    def get_servos_temp(self, ids):
        # type: (List[int]) -> List[List[int]]
        return [[int(servo_id), self.SIM_SERVO_TEMP_C] for servo_id in ids]

    def get_imu(self):
        # type: () -> Optional[tuple]
        return None  # no IMU in sim mode

    def close(self):
        # type: () -> None
        pass


    def get_servos_vin(self, ids):
        return [[int(i), 12600] for i in ids]

    def get_battery(self):
        return 12600

class ActionGroups:
    """Port of MotionManager.run_action: plays .d6a sqlite action group files,
    frame by frame, writing servos through the ServoBus."""

    def __init__(self, bus, action_path='/home/ubuntu/software/ainex_controller/ActionGroups'):
        # type: (ServoBus, str) -> None
        self.bus = bus
        self.action_path = action_path
        self.running_action = False
        self.stop_running = False

    def stop_action_group(self):
        # type: () -> None
        self.stop_running = True

    def run_action(self, act_name):
        # type: (str) -> bool
        """Blocking. Returns True if the group file existed and was played
        (or was interrupted), False if the file was not found or an action
        was already running."""
        if act_name is None:
            return False
        act_file = os.path.join(self.action_path, act_name + '.d6a')
        self.stop_running = False
        if not os.path.exists(act_file):
            log('cannot find action file', act_file)
            return False
        if self.running_action:
            return False
        self.running_action = True
        try:
            ag = sqlite3.connect(act_file)
            try:
                cu = ag.cursor()
                cu.execute('select * from ActionGroup')
                while True:
                    act = cu.fetchone()
                    if self.stop_running:
                        self.stop_running = False
                        break
                    if act is None:  # finished
                        break
                    # Row layout (vendor): [index, duration_ms, servo1, servo2, ...]
                    data = []
                    for i in range(0, len(act) - 2):
                        data.append([i + 1, act[2 + i]])
                    self.bus.set_servos_position(act[1], data)
                    time.sleep(float(act[1]) / 1000.0)
                cu.close()
            finally:
                ag.close()
        finally:
            self.running_action = False
        return True


class ImuCorrector:
    """Port of the ROS1 IMU pipeline that produced /imu_corrected:

    1. ros_robot_controller_node scaled the raw board report to SI units:
       accel = raw_g * 9.80665 (m/s^2), gyro = radians(raw_deg_s) (rad/s)
       -> published as imu_raw.
    2. imu_calib's apply_calib node (third_party/imu_calib, launched by
       ainex_peripherals/launch/imu.launch with
       ainex_calibration/config/imu_calib.yaml) then applied
           accel_corrected = SM(3x3) @ accel_raw - bias
       and subtracted an online gyro bias estimated as the mean of the first
       `gyro_calib_samples` (vendor default 100) samples while stationary.

    If the calib yaml is missing/invalid we degrade to identity SM and zero
    accel bias (i.e. raw + SI scaling only) with a startup warning; the
    online gyro bias estimation still runs either way (vendor default
    calibrate_gyros=true).
    """

    GRAVITY = 9.80665
    GYRO_CALIB_SAMPLES = 100  # imu_calib default 'gyro_calib_samples'

    def __init__(self, calib_path):
        # type: (str) -> None
        self.sm = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        self.bias = [0.0, 0.0, 0.0]
        self.calib_loaded = False
        try:
            calib = load_yaml(calib_path)
            sm = [float(v) for v in calib['SM']]
            bias = [float(v) for v in calib['bias']]
            if len(sm) != 9 or len(bias) != 3:
                raise ValueError('SM must have 9 entries and bias 3')
            self.sm = [sm[0:3], sm[3:6], sm[6:9]]
            self.bias = bias
            self.calib_loaded = True
            log('imu calib loaded from', calib_path)
        except Exception as e:
            log('WARNING: imu calib not loaded (%s: %s) — serving raw+scale '
                'IMU values (identity SM, zero accel bias)' % (type(e).__name__, e))

        # online gyro bias estimation state (recursive mean, like apply_calib)
        self.gyro_sample_count = 0
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.gyro_calibrated = False

    def correct(self, raw):
        # type: (tuple) -> Optional[Dict[str, Any]]
        """raw: board tuple (ax, ay, az, gx, gy, gz[, mx, my, mz]) in
        g / deg/s. Returns {'accel': [...], 'gyro': [...]} in m/s^2 / rad/s
        with calibration applied, or None while the gyro bias is still being
        estimated (mirrors apply_calib, which published nothing during its
        calibration window)."""
        ax = raw[0] * self.GRAVITY
        ay = raw[1] * self.GRAVITY
        az = raw[2] * self.GRAVITY
        gx = math.radians(raw[3])
        gy = math.radians(raw[4])
        gz = math.radians(raw[5])

        if not self.gyro_calibrated:
            n = self.gyro_sample_count + 1
            self.gyro_sample_count = n
            self.gyro_bias[0] = ((n - 1) * self.gyro_bias[0] + gx) / n
            self.gyro_bias[1] = ((n - 1) * self.gyro_bias[1] + gy) / n
            self.gyro_bias[2] = ((n - 1) * self.gyro_bias[2] + gz) / n
            if self.gyro_sample_count >= self.GYRO_CALIB_SAMPLES:
                self.gyro_calibrated = True
                log('gyro calibration complete (bias = [%.3f, %.3f, %.3f])' %
                    tuple(self.gyro_bias))
            return None

        accel = [
            self.sm[0][0] * ax + self.sm[0][1] * ay + self.sm[0][2] * az - self.bias[0],
            self.sm[1][0] * ax + self.sm[1][1] * ay + self.sm[1][2] * az - self.bias[1],
            self.sm[2][0] * ax + self.sm[2][1] * ay + self.sm[2][2] * az - self.bias[2],
        ]
        gyro = [gx - self.gyro_bias[0], gy - self.gyro_bias[1], gz - self.gyro_bias[2]]
        return {'accel': accel, 'gyro': gyro}


class MotionDaemon:
    # ------- constants copied verbatim from ainex_controller.Controller -------
    RADIANS_PER_ENCODER_TICK = 240 * 3.1415926 / 180 / 1000
    ENCODER_TICKS_PER_RADIAN = 180 / 3.1415926 / 240 * 1000

    joint_index = {'r_hip_yaw':   0,
                   'r_hip_roll':  1,
                   'r_hip_pitch': 2,
                   'r_knee':      3,
                   'r_ank_pitch': 4,
                   'r_ank_roll':  5,
                   'l_hip_yaw':   6,
                   'l_hip_roll':  7,
                   'l_hip_pitch': 8,
                   'l_knee':      9,
                   'l_ank_pitch': 10,
                   'l_ank_roll':  11,
                   'r_sho_pitch': 12,
                   'l_sho_pitch': 13}

    joint_id = {'r_hip_yaw':   12,
                'r_hip_roll':  10,
                'r_hip_pitch': 8,
                'r_knee':      6,
                'r_ank_pitch': 4,
                'r_ank_roll':  2,
                'l_hip_yaw':   11,
                'l_hip_roll':  9,
                'l_hip_pitch': 7,
                'l_knee':      5,
                'l_ank_pitch': 3,
                'l_ank_roll':  1,
                'r_sho_pitch': 14,
                'l_sho_pitch': 13,
                'l_sho_roll':  15,
                'r_sho_roll':  16,
                'l_el_pitch':  17,
                'r_el_pitch':  18,
                'l_el_yaw':    19,
                'r_el_yaw':    20,
                'l_gripper':   21,
                'r_gripper':   22,
                'head_pan':    23,
                'head_tilt':   24}

    joint_name = {value: key for key, value in joint_id.items()}

    body_height_range = [0.015, 0.06]
    x_amplitude_range = [-0.05, 0.05]
    y_amplitude_range = [-0.05, 0.05]
    step_height_range = [0.0, 0.05]
    angle_amplitude_range = [-10, 10]
    arm_swap_range = [0, math.radians(60)]
    y_swap_range = [0, 0.05]

    # How long "stop"/"disable"/"run_action" will wait for the gait to settle
    # before giving up. The ROS1 node waited forever (deadlock if walking was
    # never active); a bounded wait is a deliberate robustness divergence.
    STOP_WAIT_TIMEOUT_S = 10.0

    def __init__(self,
                 walking_param_path='/home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/config/walking_param.yaml',
                 walking_offset_path='/home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/config/walking_offset.yaml',
                 init_pose_path='/home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/config/init_pose.yaml',
                 servo_controller_path='/home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/config/servo_controller.yaml',
                 action_path='/home/ubuntu/software/ainex_controller/ActionGroups',
                 imu_calib_path='/home/ubuntu/ros_ws/src/ainex_calibration/config/imu_calib.yaml',
                 serial_device='/dev/rrc',
                 socket_path='/tmp/motiond.sock',
                 sim=False,
                 sim_udp_port=9910):
        # type: (...) -> None
        self.walking_param_path = walking_param_path
        self.socket_path = socket_path

        self.lock = threading.RLock()          # guards engine + params + flags
        self.action_lock = threading.Lock()    # one blocking run_action at a time
        self.shutdown_event = threading.Event()

        # ----- servo bus + action groups (direct SDK, no ROS) -----
        # PORT-VERIFY: /dev/rrc @ 1M baud is the SDK default used by the ROS1
        # ros_robot_controller node; confirm the udev alias exists in the
        # py3.8 container (else pass e.g. /dev/ttyACM0).
        self.sim = bool(sim)
        if self.sim:
            self.bus = SimServoBus()  # type: Any  # same surface as ServoBus
            self.sim_udp_addr = ('127.0.0.1', int(sim_udp_port))
            self.sim_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sim_lock = threading.Lock()   # guards sim_head_radians
            self.sim_head_radians = {}         # type: Dict[str, float]
            log('SIM MODE: serial bus not opened; joint stream -> '
                'udp://127.0.0.1:%d' % int(sim_udp_port))
        else:
            self.bus = ServoBus(device=serial_device)
        self.action_groups = ActionGroups(self.bus, action_path)

        # ----- IMU: poll the board queue on a background thread and keep the
        # latest calibrated sample available for the {"op":"imu"} handler.
        # In sim mode there is no board, so no corrector/poll thread; the
        # {"op":"imu"} handler answers {"ok":false,"err":"sim mode"}. -----
        self.imu_lock = threading.Lock()
        self.latest_imu = None  # type: Optional[Dict[str, Any]]
        if not self.sim:
            self.imu_corrector = ImuCorrector(imu_calib_path)
            self.imu_thread = threading.Thread(target=self._imu_poll_loop, daemon=True)
            self.imu_thread.start()

        # ----- pulse<->radian coefficients from servo_controller.yaml -----
        # ROS1 read rosparam '~controllers', which was loaded from this yaml
        # (top-level key 'controllers'). Same math, same flipped handling.
        self.joint_angles_convert_coef = {}  # type: Dict[int, List[float]]
        controllers = load_yaml(servo_controller_path)['controllers']
        for ctl_name, ctl_params in controllers.items():
            if isinstance(ctl_params, dict) and ctl_params.get('type') == 'JointPositionController':
                initial_position_raw = ctl_params['servo']['init']
                min_angle_raw = ctl_params['servo']['min']
                max_angle_raw = ctl_params['servo']['max']
                flipped = min_angle_raw > max_angle_raw
                if flipped:
                    self.joint_angles_convert_coef[ctl_params['servo']['id']] = \
                        [initial_position_raw, -self.ENCODER_TICKS_PER_RADIAN]
                else:
                    self.joint_angles_convert_coef[ctl_params['servo']['id']] = \
                        [initial_position_raw, self.ENCODER_TICKS_PER_RADIAN]

        # ----- initial posture from init_pose.yaml (top-level key 'init_pose') -----
        self.init_pose_data = load_yaml(init_pose_path)['init_pose']
        self.init_servo_data = []  # type: List[List[int]]
        for joint_name in self.init_pose_data:
            id_ = self.joint_id[joint_name]
            angle = self.init_pose_data[joint_name]
            pulse = self.angle2pulse(id_, angle)
            self.init_servo_data.append([id_, pulse])
        self.bus.set_servos_position(1000, self.init_servo_data)
        time.sleep(1)

        # ----- state flags (real-robot path of the ROS1 node) -----
        self.init_pose_finish = True
        self.walking_enable = True
        self.count_step = 0
        self.stop = False

        # ----- IK + walking engine, exactly as ainex_controller -----
        self.ik = LegIK()
        leg_data = self.ik.get_leg_length()
        self.leg_length = leg_data[0] + leg_data[1] + leg_data[2]

        self.walking_offset = load_yaml(walking_offset_path)

        self.present_joint_state = {}  # type: Dict[str, JointPosition]
        for joint_name in self.joint_index:
            self.present_joint_state[joint_name] = JointPosition()
            self.present_joint_state[joint_name].present_position = self.init_pose_data[joint_name]

        self.walking_param = load_yaml(walking_param_path)

        # DELIBERATE DIVERGENCE from the vendor code:
        # ainex_controller.set_app_walking_param_callback hardcodes
        # walking_param['init_x_offset'] = 0 on every app command, throwing
        # away the tuned forward-lean offset in walking_param.yaml (this robot
        # carries e.g. -0.012). We remember the yaml value here and re-apply it
        # in app_param() instead of zeroing it, so app-driven walking keeps the
        # calibrated lean. All other vendor-fixed values are kept as vendor.
        self.default_init_x_offset = self.walking_param['init_x_offset']

        self.init_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              math.radians(0.0), math.radians(-0.0)]
        self.walking_module = WalkingModule(self.ik, self.walking_param, self.joint_index,
                                            self.init_position, self.walking_param['trajectory_step_s'])

        self.last_position = None  # type: Optional[Dict[str, Any]]
        self.stop_stamp = self.walking_param['trajectory_step_s']
        self.servo_control_cycle = self.walking_param['servo_control_cycle']
        self.servo_max_move = 0
        self.max_stop_move = 0.24
        self.speed = 0.2 / math.radians(60)  # servo max speed 0.2s/60deg
        time.sleep(0.2)
        log('init finish')

    # ------------------------------------------------------------------
    # pulse <-> radian
    # ------------------------------------------------------------------
    def angle2pulse(self, id_, angle):
        # type: (int, float) -> int
        return self.joint_angles_convert_coef[id_][0] + int(round(angle * self.joint_angles_convert_coef[id_][1]))

    def pulse2angle(self, id_, pulse):
        # type: (int, float) -> float
        return (pulse - self.joint_angles_convert_coef[id_][0]) / self.joint_angles_convert_coef[id_][1]

    # ------------------------------------------------------------------
    # Tick loop — faithful port of Controller.run() (real-robot branch)
    # ------------------------------------------------------------------
    def run(self):
        # type: () -> None
        next_time = time.monotonic()
        while not self.shutdown_event.is_set():
            sleep_time = 0.0
            wrote_servos = False
            if self.init_pose_finish and self.walking_enable:
                with self.lock:
                    # Re-check under the lock: a socket command may have just
                    # dropped the gates (e.g. run_action in progress).
                    if not (self.init_pose_finish and self.walking_enable):
                        sleep_time = 0.001
                    else:
                        joint_max_move, times_stamp, joint_state = \
                            self.walking_module.run(self.present_joint_state)
                        if joint_max_move >= self.max_stop_move:
                            self.stop = False
                            self.servo_max_move = 0
                            data = []
                            for joint_name in self.present_joint_state:
                                if self.walking_param['arm_swing_gain'] != 0 or \
                                        (self.walking_param['arm_swing_gain'] == 0 and
                                         joint_name != 'r_sho_pitch' and joint_name != 'l_sho_pitch'):
                                    goal_position = joint_state[joint_name].goal_position
                                    id_ = self.joint_id[joint_name]
                                    pulse = self.angle2pulse(id_, goal_position)
                                    data.append([id_, pulse])
                                    self.present_joint_state[joint_name].present_position = goal_position
                                    if self.last_position is not None:
                                        d = abs(self.last_position[joint_name].goal_position - goal_position)
                                        if self.servo_max_move < d:
                                            self.servo_max_move = d
                            # duration 0 fast path, exactly like the ROS1 loop
                            self.bus.set_servos_position(0, data)
                            if self.sim:
                                self._sim_emit_tick(joint_state)
                            wrote_servos = True
                            curr_time = time.monotonic()
                            delta_sec = curr_time - next_time
                            delay_time = self.servo_max_move * self.speed - delta_sec
                            if delay_time > 0:
                                if self.servo_max_move * self.speed < self.servo_control_cycle:
                                    sleep_time = self.servo_control_cycle - delta_sec
                                else:
                                    sleep_time = delay_time
                            else:
                                if delta_sec < self.servo_control_cycle:
                                    sleep_time = self.servo_control_cycle - delta_sec
                            self.last_position = copy.deepcopy(joint_state)
                        elif self.walking_module.walk_finish():
                            self.servo_max_move = 0
                            sleep_time = 0.001
                            self.stop = True
                        else:
                            self.servo_max_move = 0
                            sleep_time = 0.001
                        # period_times: stop automatically after N gait cycles
                        if times_stamp >= self.walking_param['period_time'] / 1000.0 - self.walking_param['trajectory_step_s']:
                            if self.walking_param['period_times'] != 0:
                                self.count_step += 1
                                if self.walking_param['period_times'] == self.count_step:
                                    self.count_step = 0
                                    self.walking_param['period_times'] = 0
                                    self.walking_module.stop()
                            else:
                                self.count_step = 0
            else:
                sleep_time = 0.001

            # Sleeps happen outside the lock so socket commands are not starved.
            if sleep_time > 0:
                time.sleep(sleep_time)
            if wrote_servos:
                # ROS1 code re-stamps next_time right after its post-write sleep.
                next_time = time.monotonic()

    # ------------------------------------------------------------------
    # Sim mode: UDP joint stream (see README, "Sim mode")
    # ------------------------------------------------------------------
    def _sim_emit(self, joints):
        # type: (Dict[str, float]) -> None
        """Send one datagram {"t": monotonic float, "joints": {name: rad}}."""
        payload = json.dumps({'t': time.monotonic(), 'joints': joints})
        try:
            self.sim_sock.sendto(payload.encode('utf-8'), self.sim_udp_addr)
        except OSError as e:
            log('sim udp send error:', e)

    def _sim_emit_tick(self, joint_state):
        # type: (Dict[str, JointPosition]) -> None
        """Emit every goal_position (radians) the walking tick computed, plus
        the last head pan/tilt values written via {"op":"servos"}."""
        joints = {}
        for name in joint_state:
            joints[name] = joint_state[name].goal_position
        with self.sim_lock:
            joints.update(self.sim_head_radians)
        self._sim_emit(joints)

    def set_servos(self, duration_ms, positions):
        # type: (float, List[List[int]]) -> None
        """Raw servo write path used by {"op":"servos"}. In sim mode, head
        writes (ids 23/24) are converted back to radians with the yaml
        coefficients and streamed under their URDF joint names
        (head_pan / head_tilt)."""
        self.bus.set_servos_position(duration_ms, positions)
        if self.sim:
            head = {}
            for servo_id, pulse in positions:
                sid = int(servo_id)
                if sid in (23, 24) and sid in self.joint_angles_convert_coef:
                    head[self.joint_name[sid]] = self.pulse2angle(sid, int(pulse))
            if head:
                with self.sim_lock:
                    self.sim_head_radians.update(head)
                self._sim_emit(head)

    # ------------------------------------------------------------------
    # Helpers ported from the ROS1 callbacks
    # ------------------------------------------------------------------
    def _wait_walk_stopped(self):
        # type: () -> bool
        deadline = time.monotonic() + self.STOP_WAIT_TIMEOUT_S
        while not self.stop:
            if self.shutdown_event.is_set() or time.monotonic() > deadline:
                return False
            time.sleep(0.01)
        return True

    def do_command(self, command):
        # type: (str) -> None
        # Port of walking_command_callback. Commands other than the *_control
        # pair are silently ignored while init_pose_finish is False (vendor
        # behaviour: it still returned success).
        if command not in ('start', 'stop', 'enable', 'disable', 'enable_control', 'disable_control'):
            raise ValueError('unknown command: %s' % command)
        if self.init_pose_finish:
            if command == 'start':
                with self.lock:
                    self.walking_enable = True
                    self.walking_module.start()
            elif command == 'stop':
                with self.lock:
                    self.walking_module.stop()
                self._wait_walk_stopped()
            elif command == 'enable':
                with self.lock:
                    self.walking_enable = True
            elif command == 'disable':
                with self.lock:
                    self.walking_module.stop()
                self._wait_walk_stopped()
                with self.lock:
                    self.walking_enable = False
        if command == 'enable_control':
            self.init_pose_finish = True
        elif command == 'disable_control':
            self.init_pose_finish = False

    def app_param(self, speed, height, x, y, angle):
        # type: (int, float, float, float, float) -> None
        # Port of set_app_walking_param_callback (the phone-app mapping).
        with self.lock:
            wp = self.walking_param
            wo = self.walking_offset
            wp['period_times'] = 0
            # DELIBERATE DIVERGENCE (see __init__): the vendor hardcodes
            # wp['init_x_offset'] = 0 here, which discards the tuned
            # forward-lean offset from walking_param.yaml. We restore the
            # yaml-loaded value instead so the calibrated lean survives app
            # commands. Everything else below keeps the vendor's fixed values.
            wp['init_x_offset'] = self.default_init_x_offset
            wp['init_y_offset'] = 0
            wp['init_z_offset'] = height
            if self.body_height_range[0] > wp['init_z_offset']:
                wp['init_z_offset'] = self.body_height_range[0]
            elif self.body_height_range[1] < wp['init_z_offset']:
                wp['init_z_offset'] = self.body_height_range[1]

            wp['x_move_amplitude'] = x
            if self.x_amplitude_range[0] > wp['x_move_amplitude']:
                wp['x_move_amplitude'] = self.x_amplitude_range[0]
            elif self.x_amplitude_range[1] < wp['x_move_amplitude']:
                wp['x_move_amplitude'] = self.x_amplitude_range[1]
            wp['y_move_amplitude'] = y
            if self.y_amplitude_range[0] > wp['y_move_amplitude']:
                wp['y_move_amplitude'] = self.y_amplitude_range[0]
            elif self.y_amplitude_range[1] < wp['y_move_amplitude']:
                wp['y_move_amplitude'] = self.y_amplitude_range[1]

            wp['angle_move_amplitude'] = angle
            if self.angle_amplitude_range[0] > wp['angle_move_amplitude']:
                wp['angle_move_amplitude'] = self.angle_amplitude_range[0]
            elif self.angle_amplitude_range[1] < wp['angle_move_amplitude']:
                wp['angle_move_amplitude'] = self.angle_amplitude_range[1]

            wp['init_roll_offset'] = 0
            wp['init_pitch_offset'] = 0
            wp['init_yaw_offset'] = 0
            wp['hip_pitch_offset'] = 15
            wp['z_move_amplitude'] = 0.02
            wp['pelvis_offset'] = 5
            wp['move_aim_on'] = False
            wp['arm_swing_gain'] = 0.5

            if speed == 4:
                wp['period_time'] = 330
                wp['dsp_ratio'] = 0.2
                wp['init_y_offset'] = -0.008
                wp['step_fb_ratio'] = 0.028
                wp['y_swap_amplitude'] = 0.02
                wp['z_swap_amplitude'] = 0.006
                wp['pelvis_offset'] = 5
                wp['z_move_amplitude'] = 0.015

                if wp['x_move_amplitude'] != 0:
                    wp['x_move_amplitude'] = math.copysign(0.01, wp['x_move_amplitude'])
                if wp['y_move_amplitude'] != 0:
                    wp['y_move_amplitude'] = math.copysign(0.01, wp['y_move_amplitude'])
                if wp['angle_move_amplitude'] != 0:
                    wp['angle_move_amplitude'] = math.copysign(8, wp['angle_move_amplitude'])

                if wp['x_move_amplitude'] != 0:
                    wp['init_roll_offset'] = -3
                    if wp['y_move_amplitude'] == 0 and wp['angle_move_amplitude'] == 0:
                        wp['x_move_amplitude'] = math.copysign(0.012, wp['x_move_amplitude'])
                    if wp['x_move_amplitude'] > 0:
                        wp['angle_move_amplitude'] += wo['high_speed_forward_offset']
                    else:
                        wp['init_z_offset'] = 0.030
                        wp['angle_move_amplitude'] += wo['high_speed_backward_offset']

                if wp['y_move_amplitude'] != 0 and wp['x_move_amplitude'] == 0 and wp['angle_move_amplitude'] == 0:
                    wp['y_move_amplitude'] = math.copysign(0.015, wp['y_move_amplitude'])
                    if wp['y_move_amplitude'] > 0:
                        wp['angle_move_amplitude'] += wo['high_speed_move_left_offset']
                    else:
                        wp['angle_move_amplitude'] += wo['high_speed_move_right_offset']

                if wp['angle_move_amplitude'] != 0 and wp['y_move_amplitude'] == 0 and wp['x_move_amplitude'] == 0:
                    wp['init_roll_offset'] = -3
                    wp['angle_move_amplitude'] = math.copysign(10, wp['angle_move_amplitude'])

            elif speed == 3:
                wp['period_time'] = 400
                wp['dsp_ratio'] = 0.2
                wp['init_y_offset'] = -0.005
                wp['step_fb_ratio'] = 0.028
                wp['y_swap_amplitude'] = 0.023
                wp['z_swap_amplitude'] = 0.006

                if wp['x_move_amplitude'] != 0:
                    wp['x_move_amplitude'] = math.copysign(0.01, wp['x_move_amplitude'])
                if wp['y_move_amplitude'] != 0:
                    wp['y_move_amplitude'] = math.copysign(0.01, wp['y_move_amplitude'])
                if wp['angle_move_amplitude'] != 0:
                    wp['angle_move_amplitude'] = math.copysign(8, wp['angle_move_amplitude'])

                if wp['angle_move_amplitude'] != 0 and wp['y_move_amplitude'] == 0 and wp['x_move_amplitude'] == 0:
                    wp['y_swap_amplitude'] = 0.023
                    wp['angle_move_amplitude'] = math.copysign(8, wp['angle_move_amplitude'])
                elif wp['angle_move_amplitude'] != 0:
                    if wp['y_move_amplitude'] != 0:
                        wp['init_roll_offset'] = 3
                    elif wp['x_move_amplitude'] != 0:
                        wp['angle_move_amplitude'] = math.copysign(8, wp['angle_move_amplitude'])

                if wp['y_move_amplitude'] != 0 and wp['x_move_amplitude'] == 0 and wp['angle_move_amplitude'] == 0:
                    wp['init_y_offset'] = 0
                    wp['init_roll_offset'] = 3
                    wp['y_swap_amplitude'] = 0.023
                    wp['y_move_amplitude'] = math.copysign(0.012, wp['y_move_amplitude'])
                    if wp['y_move_amplitude'] > 0:
                        wp['angle_move_amplitude'] += wo['medium_speed_move_left_offset']
                    else:
                        wp['angle_move_amplitude'] += wo['medium_speed_move_right_offset']

                if wp['x_move_amplitude'] != 0 and wp['y_move_amplitude'] == 0 and wp['angle_move_amplitude'] == 0:
                    wp['x_move_amplitude'] = math.copysign(0.012, wp['x_move_amplitude'])
                    if wp['x_move_amplitude'] > 0:
                        wp['angle_move_amplitude'] += wo['medium_speed_forward_offset']
                    else:
                        wp['angle_move_amplitude'] += wo['medium_speed_backward_offset']

            elif speed == 2:
                wp['period_time'] = 500
                wp['dsp_ratio'] = 0.2
                wp['step_fb_ratio'] = 0.028
                wp['y_swap_amplitude'] = 0.02
                wp['z_swap_amplitude'] = 0.006
                wp['init_y_offset'] = -0.008

                if wp['x_move_amplitude'] != 0:
                    wp['x_move_amplitude'] = math.copysign(0.01, wp['x_move_amplitude'])
                if wp['y_move_amplitude'] != 0:
                    wp['y_move_amplitude'] = math.copysign(0.01, wp['y_move_amplitude'])
                if wp['angle_move_amplitude'] != 0:
                    wp['angle_move_amplitude'] = math.copysign(10, wp['angle_move_amplitude'])

                if wp['angle_move_amplitude'] != 0 and wp['y_move_amplitude'] == 0 and wp['x_move_amplitude'] == 0:
                    wp['init_roll_offset'] = 0
                    wp['init_y_offset'] = -0.005
                    wp['y_swap_amplitude'] = 0.022
                elif wp['angle_move_amplitude'] != 0 and wp['x_move_amplitude'] != 0:
                    wp['y_swap_amplitude'] = 0.022

                if wp['y_move_amplitude'] != 0 and wp['x_move_amplitude'] == 0 and wp['angle_move_amplitude'] == 0:
                    wp['init_y_offset'] = -0.005
                    wp['y_swap_amplitude'] = 0.028
                    wp['init_roll_offset'] = 3
                    if wp['y_move_amplitude'] > 0:
                        wp['angle_move_amplitude'] += wo['low_speed_move_left_offset']
                    else:
                        wp['angle_move_amplitude'] += wo['low_speed_move_right_offset']
                    wp['y_move_amplitude'] = math.copysign(0.015, wp['y_move_amplitude'])
                elif wp['y_move_amplitude'] != 0 and wp['angle_move_amplitude'] != 0:
                    wp['init_y_offset'] = -0.005
                    wp['y_swap_amplitude'] = 0.025
                    wp['init_roll_offset'] = 3

                if wp['x_move_amplitude'] != 0 and wp['y_move_amplitude'] == 0 and wp['angle_move_amplitude'] == 0:
                    wp['x_move_amplitude'] = math.copysign(0.015, wp['x_move_amplitude'])
                    if wp['x_move_amplitude'] > 0:
                        wp['angle_move_amplitude'] += wo['low_speed_forward_offset']
                    else:
                        wp['angle_move_amplitude'] += wo['low_speed_backward_offset']

            elif speed == 1:
                wp['period_time'] = 600
                wp['dsp_ratio'] = 0.2
                wp['step_fb_ratio'] = 0.028
                wp['y_swap_amplitude'] = 0.02
                wp['z_swap_amplitude'] = 0.006
                wp['init_y_offset'] = -0.008

                if wp['x_move_amplitude'] != 0:
                    wp['x_move_amplitude'] = math.copysign(0.01, wp['x_move_amplitude'])
                if wp['y_move_amplitude'] != 0:
                    wp['y_move_amplitude'] = math.copysign(0.01, wp['y_move_amplitude'])
                if wp['angle_move_amplitude'] != 0:
                    wp['angle_move_amplitude'] = math.copysign(10, wp['angle_move_amplitude'])

                if wp['angle_move_amplitude'] != 0 and wp['y_move_amplitude'] == 0 and wp['x_move_amplitude'] == 0:
                    wp['y_swap_amplitude'] = 0.025
                    wp['init_roll_offset'] = 3
                    wp['init_y_offset'] = 0
                elif wp['angle_move_amplitude'] != 0 and wp['x_move_amplitude'] != 0:
                    wp['y_swap_amplitude'] = 0.025
                    wp['init_roll_offset'] = 3
                    wp['init_y_offset'] = 0

                if wp['y_move_amplitude'] != 0 and wp['x_move_amplitude'] == 0 and wp['angle_move_amplitude'] == 0:
                    wp['init_y_offset'] = 0
                    wp['y_swap_amplitude'] = 0.03
                    wp['init_roll_offset'] = 5
                    wp['y_move_amplitude'] = math.copysign(0.012, wp['y_move_amplitude'])
                    if wp['y_move_amplitude'] > 0:
                        wp['angle_move_amplitude'] += 0
                    else:
                        wp['angle_move_amplitude'] += 0
                elif wp['y_move_amplitude'] != 0 and wp['angle_move_amplitude'] != 0:
                    wp['init_y_offset'] = -0.005
                    wp['y_swap_amplitude'] = 0.03
                    wp['init_roll_offset'] = 3

                if wp['x_move_amplitude'] != 0 and wp['y_move_amplitude'] == 0 and wp['angle_move_amplitude'] == 0:
                    wp['init_y_offset'] = 0
                    wp['init_roll_offset'] = 3
                    wp['x_move_amplitude'] = math.copysign(0.015, wp['x_move_amplitude'])
                    if wp['x_move_amplitude'] > 0:
                        wp['angle_move_amplitude'] += 0
                    else:
                        wp['angle_move_amplitude'] += 0

            self.walking_module.set_walking_param(wp)

    def set_param(self, params):
        # type: (Dict[str, Any]) -> None
        # Port of set_walking_param_callback, but accepting a partial dict
        # (the ROS1 msg always carried every field; JSON callers may send a
        # subset). The same clamps are applied to whatever is provided.
        if not isinstance(params, dict):
            raise ValueError('params must be an object')
        with self.lock:
            wp = self.walking_param
            passthrough_keys = ('period_times', 'init_x_offset', 'init_y_offset',
                                'init_roll_offset', 'init_pitch_offset', 'init_yaw_offset',
                                'hip_pitch_offset', 'period_time', 'dsp_ratio',
                                'step_fb_ratio', 'z_swap_amplitude', 'pelvis_offset',
                                'move_aim_on')
            for key in passthrough_keys:
                if key in params:
                    wp[key] = params[key]
            if 'init_z_offset' in params:
                wp['init_z_offset'] = params['init_z_offset']
                if self.body_height_range[0] > wp['init_z_offset']:
                    wp['init_z_offset'] = self.body_height_range[0]
                elif self.body_height_range[1] < wp['init_z_offset']:
                    wp['init_z_offset'] = self.body_height_range[1]
            if 'x_move_amplitude' in params:
                wp['x_move_amplitude'] = params['x_move_amplitude']
                if self.x_amplitude_range[0] > wp['x_move_amplitude']:
                    wp['x_move_amplitude'] = self.x_amplitude_range[0]
                elif self.x_amplitude_range[1] < wp['x_move_amplitude']:
                    wp['x_move_amplitude'] = self.x_amplitude_range[1]
            if 'y_move_amplitude' in params:
                wp['y_move_amplitude'] = params['y_move_amplitude']
                if self.y_amplitude_range[0] > wp['y_move_amplitude']:
                    wp['y_move_amplitude'] = self.y_amplitude_range[0]
                elif self.y_amplitude_range[1] < wp['y_move_amplitude']:
                    wp['y_move_amplitude'] = self.y_amplitude_range[1]
            if 'z_move_amplitude' in params:
                wp['z_move_amplitude'] = params['z_move_amplitude']
                if self.step_height_range[0] > wp['z_move_amplitude']:
                    wp['z_move_amplitude'] = self.step_height_range[0]
                elif self.step_height_range[1] < wp['z_move_amplitude']:
                    wp['z_move_amplitude'] = self.step_height_range[1]
            if 'angle_move_amplitude' in params:
                wp['angle_move_amplitude'] = params['angle_move_amplitude']
                if self.angle_amplitude_range[0] > wp['angle_move_amplitude']:
                    wp['angle_move_amplitude'] = self.angle_amplitude_range[0]
                elif self.angle_amplitude_range[1] < wp['angle_move_amplitude']:
                    wp['angle_move_amplitude'] = self.angle_amplitude_range[1]
            if 'y_swap_amplitude' in params:
                wp['y_swap_amplitude'] = params['y_swap_amplitude']
                if self.y_swap_range[0] > wp['y_swap_amplitude']:
                    wp['y_swap_amplitude'] = self.y_swap_range[0]
                elif self.y_swap_range[1] < wp['y_swap_amplitude']:
                    wp['y_swap_amplitude'] = self.y_swap_range[1]
            if 'arm_swing_gain' in params:
                wp['arm_swing_gain'] = params['arm_swing_gain']
                if self.arm_swap_range[0] > wp['arm_swing_gain']:
                    wp['arm_swing_gain'] = self.arm_swap_range[0]
                elif self.arm_swap_range[1] < wp['arm_swing_gain']:
                    wp['arm_swing_gain'] = self.arm_swap_range[1]
            self.walking_module.set_walking_param(wp)

    def get_param(self):
        # type: () -> Dict[str, Any]
        # Port of get_walking_param_callback: pull the live values from the
        # engine. The ROS1 node replaced self.walking_param wholesale with the
        # engine dict; we merge instead so daemon-only keys
        # (trajectory_step_s / servo_control_cycle) can never be lost.
        with self.lock:
            engine_param = self.walking_module.get_walking_param()
            # PORT-VERIFY: confirm get_walking_param() returns a plain dict
            # with the same keys walking_param.yaml carries.
            self.walking_param.update(engine_param)
            out = dict(self.walking_param)
            out['period_times'] = 0  # vendor reported 0 here
        result = {}
        for k, v in out.items():
            if isinstance(v, (bool, int, float, str)) or v is None:
                result[k] = v
            else:
                try:
                    result[k] = float(v)
                except (TypeError, ValueError):
                    result[k] = str(v)
        return result

    def move_to_init_pose(self):
        # type: () -> None
        # Port of Controller.move_to_init_pose: compare actual servo positions
        # with the recorded init pose; if the robot drifted too far, seed the
        # walking engine from reality (fresh WalkingModule) so the next gait
        # ramp starts from where the servos actually are.
        self.init_pose_finish = False
        data = self.bus.get_servos_position(list(range(1, 15)))

        # Vendor sanity-check of hip-yaw reads (ids 11/12 occasionally return
        # garbage): flag, re-read once, then clamp to 500. The ROS1 code also
        # wrote debug files here via a buggy 'str(i[1])' (i is an int loop
        # index — it would have crashed); we just log instead.
        read_error = False
        for i in range(len(data)):
            if data[i][0] == 11 and (data[i][1] < 385 or data[i][1] > 570):
                read_error = True
                log('suspicious read servo 11:', data[i][1])
            if data[i][0] == 12 and (data[i][1] < 430 or data[i][1] > 615):
                read_error = True
                log('suspicious read servo 12:', data[i][1])
        if read_error:
            data = self.bus.get_servos_position(list(range(1, 15)))
            for i in range(len(data)):
                if data[i][0] == 11 and (data[i][1] < 385 or data[i][1] > 570):
                    data[i][1] = 500
                if data[i][0] == 12 and (data[i][1] < 430 or data[i][1] > 615):
                    data[i][1] = 500

        d = 0.0
        for item in data:
            if self.joint_name[item[0]] in self.joint_index:
                d += abs(self.init_pose_data[self.joint_name[item[0]]] - self.pulse2angle(item[0], item[1]))
        if d > self.max_stop_move:
            with self.lock:
                for item in data:
                    if self.joint_name[item[0]] in self.joint_index:
                        jp = JointPosition()
                        jp.present_position = self.pulse2angle(item[0], item[1])
                        self.present_joint_state[self.joint_name[item[0]]] = jp
                # Vendor reloads the yaml here, discarding any runtime param
                # changes — kept faithful.
                self.walking_param = load_yaml(self.walking_param_path)
                self.walking_module = WalkingModule(self.ik, self.walking_param, self.joint_index,
                                                    self.init_position, self.walking_param['trajectory_step_s'])
                self.last_position = None

        # Arms/grippers (everything that is not a walking joint or the head)
        # go back to the recorded init pose; the walking engine restores the
        # legs itself once ticking resumes.
        init_servo_data = []
        for joint_name in self.init_pose_data:
            if joint_name not in self.joint_index and joint_name != 'head_pan' and joint_name != 'head_tilt':
                id_ = self.joint_id[joint_name]
                angle = self.init_pose_data[joint_name]
                pulse = self.angle2pulse(id_, angle)
                init_servo_data.append([id_, pulse])
        self.bus.set_servos_position(500, init_servo_data)
        time.sleep(0.5)
        self.init_pose_finish = True

    def run_action(self, name):
        # type: (str) -> None
        # Port of set_action_callback: blocking — stop walking, play the
        # group, then re-init pose and re-enable walking.
        if not self.action_lock.acquire(blocking=False):
            raise RuntimeError('an action is already running')
        try:
            with self.lock:
                self.walking_module.stop()
            self._wait_walk_stopped()
            self.walking_enable = False
            self.init_pose_finish = False
            ok = self.action_groups.run_action(name)
            self.move_to_init_pose()
            self.walking_enable = True
            if not ok:
                raise ValueError('action file not found: %s' % name)
        finally:
            self.action_lock.release()

    def state(self):
        # type: () -> Dict[str, bool]
        return {
            'walking': not self.stop,          # same meaning as ROS1 walking/is_walking
            'enabled': bool(self.walking_enable),
            'stopped': bool(self.stop),
        }

    # ------------------------------------------------------------------
    # IMU
    # ------------------------------------------------------------------
    def _imu_poll_loop(self):
        # type: () -> None
        # ~100 Hz poll of the SDK's IMU queue. Each poll is a non-blocking
        # queue read under the bus lock (no serial round-trip), so servo IO
        # is never starved. PORT-VERIFY: confirm the STM32 auto-reports IMU
        # frames once enable_reception(True) is set (the ROS1 node relied on
        # the same behaviour); if frames never arrive, 'imu' returns
        # 'no imu data'.
        while not self.shutdown_event.is_set():
            try:
                raw = self.bus.get_imu()
            except Exception as e:
                log('imu poll error:', e)
                raw = None
            if raw is not None:
                corrected = self.imu_corrector.correct(raw)
                if corrected is not None:
                    with self.imu_lock:
                        self.latest_imu = corrected
            time.sleep(0.01)

    def get_imu_corrected(self):
        # type: () -> Dict[str, Any]
        """Latest calibrated sample: accel in m/s^2 (upright gravity reads
        ~+9.8 on Y), gyro in rad/s — matching the ROS1 /imu_corrected topic."""
        with self.imu_lock:
            sample = self.latest_imu
        if sample is None:
            if not self.imu_corrector.gyro_calibrated:
                raise RuntimeError('no imu data: gyro bias calibration in progress '
                                   '(%d/%d samples) — keep the robot still'
                                   % (self.imu_corrector.gyro_sample_count,
                                      self.imu_corrector.GYRO_CALIB_SAMPLES))
            raise RuntimeError('no imu data received from the board')
        return sample

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def shutdown(self):
        # type: () -> None
        if self.shutdown_event.is_set():
            return
        log('shutting down')
        self.shutdown_event.set()
        self.action_groups.stop_action_group()
        try:
            with self.lock:
                self.walking_module.stop()
        except Exception as e:
            log('walking stop error:', e)
        time.sleep(0.1)
        self.bus.close()
        if self.sim:
            try:
                self.sim_sock.close()
            except OSError:
                pass


# ---------------------------------------------------------------------------
# Socket server: newline-delimited JSON over a Unix domain socket, multi-client
# ---------------------------------------------------------------------------
class SocketServer:
    def __init__(self, daemon, socket_path='/tmp/motiond.sock'):
        # type: (MotionDaemon, str) -> None
        self.daemon = daemon
        self.socket_path = socket_path
        self.server = None  # type: Optional[socket.socket]
        self.thread = None  # type: Optional[threading.Thread]

    def start(self):
        # type: () -> None
        try:
            os.unlink(self.socket_path)
        except OSError as e:
            if e.errno != errno.ENOENT:
                raise
        self.server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.server.bind(self.socket_path)
        os.chmod(self.socket_path, 0o666)
        self.server.listen(8)
        self.server.settimeout(1.0)
        self.thread = threading.Thread(target=self._accept_loop, daemon=True)
        self.thread.start()
        log('listening on', self.socket_path)

    def _accept_loop(self):
        # type: () -> None
        while not self.daemon.shutdown_event.is_set():
            try:
                conn, _ = self.server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            t = threading.Thread(target=self._client_loop, args=(conn,), daemon=True)
            t.start()

    def _client_loop(self, conn):
        # type: (socket.socket) -> None
        buf = b''
        try:
            conn.settimeout(None)
            while not self.daemon.shutdown_event.is_set():
                chunk = conn.recv(4096)
                if not chunk:
                    break
                buf += chunk
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    response = self._handle_line(line)
                    conn.sendall((json.dumps(response) + '\n').encode('utf-8'))
        except (OSError, BrokenPipeError):
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass

    def _handle_line(self, line):
        # type: (bytes) -> Dict[str, Any]
        try:
            request = json.loads(line.decode('utf-8'))
        except (ValueError, UnicodeDecodeError) as e:
            return {'ok': False, 'err': 'bad json: %s' % e}
        if not isinstance(request, dict):
            return {'ok': False, 'err': 'request must be a json object'}
        try:
            return self._dispatch(request)
        except Exception as e:  # keep the daemon alive whatever a client sends
            return {'ok': False, 'err': '%s: %s' % (type(e).__name__, e)}

    def _dispatch(self, request):
        # type: (Dict[str, Any]) -> Dict[str, Any]
        d = self.daemon
        op = request.get('op')
        if op == 'command':
            d.do_command(str(request.get('command')))
            return {'ok': True}
        elif op == 'app_param':
            d.app_param(int(request.get('speed', 3)),
                        float(request.get('height', 0.025)),
                        float(request.get('x', 0.0)),
                        float(request.get('y', 0.0)),
                        float(request.get('angle', 0.0)))
            return {'ok': True}
        elif op == 'set_param':
            d.set_param(request.get('params'))
            return {'ok': True}
        elif op == 'get_param':
            return {'ok': True, 'params': d.get_param()}
        elif op == 'servos':
            positions = request.get('positions')
            if not isinstance(positions, list) or not positions:
                return {'ok': False, 'err': 'positions must be a non-empty list of [id, pulse]'}
            clean = []
            for item in positions:
                if (not isinstance(item, (list, tuple))) or len(item) != 2:
                    return {'ok': False, 'err': 'positions entries must be [id, pulse]'}
                clean.append([int(item[0]), int(item[1])])
            duration_ms = float(request.get('duration_ms', 0))
            d.set_servos(duration_ms, clean)
            return {'ok': True}
        elif op == 'get_servos':
            ids = request.get('ids')
            if not isinstance(ids, list) or not ids:
                return {'ok': False, 'err': 'ids must be a non-empty list'}
            data = d.bus.get_servos_position([int(i) for i in ids])
            return {'ok': True, 'positions': data}
        elif op == 'run_action':
            name = request.get('name')
            if not name or not isinstance(name, str):
                return {'ok': False, 'err': 'name must be a non-empty string'}
            d.run_action(name)
            return {'ok': True}
        elif op == 'imu':
            if d.sim:
                # A later phase may loop back a simulated IMU from Gazebo.
                return {'ok': False, 'err': 'sim mode'}
            sample = d.get_imu_corrected()
            return {'ok': True, 'accel': sample['accel'], 'gyro': sample['gyro']}
        elif op == 'servo_temp':
            ids = request.get('ids')
            if not isinstance(ids, list) or not ids:
                return {'ok': False, 'err': 'ids must be a non-empty list'}
            temps = d.bus.get_servos_temp([int(i) for i in ids])
            return {'ok': True, 'temps': temps}
        elif op == 'servo_state':
            ids = request.get('ids')
            if not isinstance(ids, list) or not ids:
                return {'ok': False, 'err': 'ids must be a non-empty list'}
            temps = dict(d.bus.get_servos_temp([int(i) for i in ids]))
            try:
                vins = dict(d.bus.get_servos_vin([int(i) for i in ids]))
            except Exception:
                vins = {}
            return {'ok': True,
                    'state': [[i, temps.get(i), vins.get(i)] for i in [int(x) for x in ids]]}
        elif op == 'battery':
            mv = d.bus.get_battery()
            if mv is None:
                return {'ok': False, 'err': 'battery unavailable'}
            return {'ok': True, 'battery_mv': int(mv)}
        elif op == 'state':
            result = {'ok': True}
            result.update(d.state())
            return result
        else:
            return {'ok': False, 'err': 'unknown op: %s' % op}

    def close(self):
        # type: () -> None
        if self.server is not None:
            try:
                self.server.close()
            except OSError:
                pass
        try:
            os.unlink(self.socket_path)
        except OSError:
            pass


def main():
    parser = argparse.ArgumentParser(description='AiNex standalone motion daemon (no ROS)')
    parser.add_argument('--walking-param',
                        default='/home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/config/walking_param.yaml')
    parser.add_argument('--walking-offset',
                        default='/home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/config/walking_offset.yaml')
    parser.add_argument('--init-pose',
                        default='/home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/config/init_pose.yaml')
    parser.add_argument('--servo-controller',
                        default='/home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/config/servo_controller.yaml')
    parser.add_argument('--action-path', default='/home/ubuntu/software/ainex_controller/ActionGroups')
    parser.add_argument('--imu-calib',
                        default='/home/ubuntu/ros_ws/src/ainex_calibration/config/imu_calib.yaml')
    parser.add_argument('--serial-device', default='/dev/rrc')
    parser.add_argument('--socket', default='/tmp/motiond.sock')
    parser.add_argument('--sim', action='store_true',
                        help='simulation mode: no serial bus; joint goals are '
                             'streamed as JSON UDP datagrams for the Gazebo '
                             'digital twin (see README)')
    parser.add_argument('--sim-udp-port', type=int, default=9910,
                        help='UDP port on 127.0.0.1 for --sim joint datagrams')
    args = parser.parse_args()

    daemon = MotionDaemon(walking_param_path=args.walking_param,
                          walking_offset_path=args.walking_offset,
                          init_pose_path=args.init_pose,
                          servo_controller_path=args.servo_controller,
                          action_path=args.action_path,
                          imu_calib_path=args.imu_calib,
                          serial_device=args.serial_device,
                          socket_path=args.socket,
                          sim=args.sim,
                          sim_udp_port=args.sim_udp_port)

    server = SocketServer(daemon, args.socket)
    server.start()

    def handle_signal(signum, frame):
        daemon.shutdown_event.set()

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    try:
        daemon.run()  # tick loop on the main thread; blocks until shutdown
    finally:
        daemon.shutdown()
        server.close()
        log('bye')


if __name__ == '__main__':
    main()
