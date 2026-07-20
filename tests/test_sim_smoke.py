#!/usr/bin/env python3
"""Pure-python smoke test for the sim pipeline. Run with a bare system python3
(no ROS, no robot .so files, no pyserial needed):

    python3 ros2_port/tests/test_sim_smoke.py

Covers:
  1. sim_adapter datagram->ordered-array conversion (parse_datagram /
     merge_joint_positions), imported standalone (no rclpy).
  2. motiond sim-mode emitter path: SimServoBus semantics, the tick datagram
     schema over a real UDP socket, and the {"op":"servos"} head 23/24
     pulse->radian loopback, with the vendor .so / board SDK stubbed out.
  3. End-to-end: motiond datagrams fed through the adapter conversion.
"""
import importlib.util
import json
import math
import os
import socket
import sys
import threading
import types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)  # .../ros2_port
SIM_ADAPTER_PATH = os.path.join(
    ROOT, 'ros2_ws', 'src', 'ainex_bridge', 'ainex_bridge', 'sim_adapter.py')
MOTIOND_PATH = os.path.join(ROOT, 'motiond', 'motiond.py')

failures = []


def check(name, cond, detail=''):
    status = 'ok' if cond else 'FAIL'
    print('  [%s] %s%s' % (status, name, (' — ' + detail) if detail and not cond else ''))
    if not cond:
        failures.append(name)


def import_from_path(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------------------
# 1. sim_adapter conversion functions, standalone (no rclpy installed needed)
# ---------------------------------------------------------------------------
print('== sim_adapter conversion (standalone import, no rclpy) ==')
sim_adapter = import_from_path('sim_adapter_under_test', SIM_ADAPTER_PATH)

order = ['r_hip_yaw', 'l_knee', 'head_pan']
seed = [0.0, 0.0, 0.0]

dg = json.dumps({'t': 1.25, 'joints': {'l_knee': 0.5, 'r_hip_yaw': -0.1,
                                       'not_a_joint': 9.9}}).encode('utf-8')
joints = sim_adapter.parse_datagram(dg)
check('parse_datagram parses joints', joints == {'l_knee': 0.5, 'r_hip_yaw': -0.1,
                                                 'not_a_joint': 9.9}, repr(joints))
check('parse_datagram rejects garbage', sim_adapter.parse_datagram(b'\xff\x00{') is None)
check('parse_datagram rejects non-object', sim_adapter.parse_datagram(b'[1,2]') is None)
check('parse_datagram rejects missing joints',
      sim_adapter.parse_datagram(b'{"t": 1.0}') is None)
check('parse_datagram drops non-numeric values',
      sim_adapter.parse_datagram(
          b'{"t":1,"joints":{"a":"x","b":true,"c":2}}') == {'c': 2.0})

vals1 = sim_adapter.merge_joint_positions(joints, order, seed)
check('merge: fixed order + unknown ignored', vals1 == [-0.1, 0.5, 0.0], repr(vals1))
check('merge: input untouched', seed == [0.0, 0.0, 0.0])

vals2 = sim_adapter.merge_joint_positions({'head_pan': 0.3}, order, vals1)
check('merge: missing joints hold last value', vals2 == [-0.1, 0.5, 0.3], repr(vals2))

check('DEFAULT_JOINT_ORDER has 16 joints and head last',
      len(sim_adapter.DEFAULT_JOINT_ORDER) == 16 and
      sim_adapter.DEFAULT_JOINT_ORDER[-2:] == ['head_pan', 'head_tilt'])

# ---------------------------------------------------------------------------
# 2. motiond sim emitter path (vendor .so + board SDK stubbed)
# ---------------------------------------------------------------------------
print('== motiond sim mode (stubbed engine) ==')


def stub_module(name, **attrs):
    mod = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(mod, k, v)
    sys.modules.setdefault(name, mod)


class _Stub(object):
    def __init__(self, *a, **k):
        pass


stub_module('kinematics', LegIK=_Stub)
stub_module('walking_module', WalkingModule=_Stub)
stub_module('ros_robot_controller_sdk', Board=_Stub)
try:
    import yaml  # noqa: F401
except ImportError:
    stub_module('yaml', safe_load=lambda f: {})

motiond = import_from_path('motiond_under_test', MOTIOND_PATH)

# SimServoBus semantics
bus = motiond.SimServoBus()
bus.set_servos_position(1000, [[1, 400], [23, 500]])
check('SimServoBus records writes',
      bus.get_servos_position([1, 23]) == [[1, 400], [23, 500]])
try:
    bus.get_servos_position([2])
    check('SimServoBus unknown id raises IOError', False)
except IOError:
    check('SimServoBus unknown id raises IOError', True)
check('SimServoBus fixed 35C temps', bus.get_servos_temp([5, 6]) == [[5, 35], [6, 35]])
check('SimServoBus no imu', bus.get_imu() is None)

# Minimal sim daemon (skip __init__: no configs/engine needed for the emitter)
rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx.bind(('127.0.0.1', 0))
rx.settimeout(2.0)
port = rx.getsockname()[1]

d = motiond.MotionDaemon.__new__(motiond.MotionDaemon)
d.sim = True
d.sim_lock = threading.Lock()
d.sim_head_radians = {}
d.sim_udp_addr = ('127.0.0.1', port)
d.sim_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
d.bus = motiond.SimServoBus()
ETPR = motiond.MotionDaemon.ENCODER_TICKS_PER_RADIAN
# head servos: init=500, not flipped (vendor servo_controller.yaml)
d.joint_angles_convert_coef = {23: [500, ETPR], 24: [500, ETPR]}


def recv_dg():
    data, _ = rx.recvfrom(65536)
    return json.loads(data.decode('utf-8'))


# a) {"op":"servos"} head write -> pulse->radian loopback datagram
server = motiond.SocketServer(d, os.path.join(HERE, 'unused.sock'))
resp = server._dispatch({'op': 'servos', 'duration_ms': 200,
                         'positions': [[23, 700], [24, 500], [1, 512]]})
check('servos op ok in sim', resp == {'ok': True}, repr(resp))
msg = recv_dg()
expected_pan = (700 - 500) / ETPR
check('head datagram has t float', isinstance(msg.get('t'), float))
check('head datagram names match URDF (head_pan/head_tilt)',
      sorted(msg['joints'].keys()) == ['head_pan', 'head_tilt'], repr(msg))
check('head pan pulse->radian correct',
      abs(msg['joints']['head_pan'] - expected_pan) < 1e-9 and
      msg['joints']['head_tilt'] == 0.0,
      repr(msg['joints']))
check('non-head id recorded but not streamed',
      d.bus.servo_position.get(1) == 512 and '1' not in msg['joints'])

# b) tick emit: every computed goal_position + last head values merged in
joint_state = {}
tick_truth = {}
for i, name in enumerate(sorted(motiond.MotionDaemon.joint_index)):
    jp = motiond.JointPosition()
    jp.goal_position = 0.01 * i - 0.05
    joint_state[name] = jp
    tick_truth[name] = jp.goal_position
d._sim_emit_tick(joint_state)
msg = recv_dg()
ok_names = set(msg['joints']) == set(tick_truth) | {'head_pan', 'head_tilt'}
check('tick datagram carries all 14 computed joints + head', ok_names,
      repr(sorted(msg['joints'])))
ok_vals = all(abs(msg['joints'][n] - v) < 1e-12 for n, v in tick_truth.items())
check('tick datagram values are the goal radians', ok_vals)
check('tick datagram holds last head values',
      abs(msg['joints']['head_pan'] - expected_pan) < 1e-9)

# c) imu / servo_temp / get_servos protocol answers in sim
check("imu answers {'ok':False,'err':'sim mode'}",
      server._dispatch({'op': 'imu'}) == {'ok': False, 'err': 'sim mode'})
check('servo_temp answers fixed 35C',
      server._dispatch({'op': 'servo_temp', 'ids': [3, 4]}) ==
      {'ok': True, 'temps': [[3, 35], [4, 35]]})
check('get_servos answers from recorded dict',
      server._dispatch({'op': 'get_servos', 'ids': [23, 1]}) ==
      {'ok': True, 'positions': [[23, 700], [1, 512]]})

# ---------------------------------------------------------------------------
# 3. end-to-end: motiond datagrams through the adapter conversion
# ---------------------------------------------------------------------------
print('== end-to-end datagram -> ordered array ==')
d._sim_emit_tick(joint_state)
data, _ = rx.recvfrom(65536)
parsed = sim_adapter.parse_datagram(data)
check('adapter parses motiond datagram', parsed is not None)
full_order = sim_adapter.DEFAULT_JOINT_ORDER
arr = sim_adapter.merge_joint_positions(parsed, full_order, [0.0] * len(full_order))
check('array length matches joint_order', len(arr) == len(full_order))
check('array follows the fixed order',
      abs(arr[full_order.index('r_knee')] - tick_truth['r_knee']) < 1e-12 and
      abs(arr[full_order.index('head_pan')] - expected_pan) < 1e-9)

rx.close()
d.sim_sock.close()

print()
if failures:
    print('FAILED: %d check(s): %s' % (len(failures), ', '.join(failures)))
    sys.exit(1)
print('all checks passed')
