#!/usr/bin/env python3
# encoding: utf-8
"""motiond_client -- client library for the local motion daemon (motiond).

Talks newline-delimited JSON over a Unix domain socket (default
/tmp/motiond.sock).  This is the ROS-version-agnostic pathway: mission code
uses this (via compat.py) instead of rospy publishers/services.

Protocol (fixed contract):
  {"op":"command","command":"start|stop|enable|disable|enable_control|disable_control"}
                                                  -> {"ok":true}
  {"op":"app_param","speed":3,"height":0.025,"x":0.0,"y":0.0,"angle":0.0}
                                                  -> {"ok":true}
  {"op":"set_param","params":{...}}               -> {"ok":true}
  {"op":"get_param"}                              -> {"ok":true,"params":{...}}
  {"op":"servos","duration_ms":200,"positions":[[23,500],[24,500]]}
                                                  -> {"ok":true}
  {"op":"get_servos","ids":[1,2]}                 -> {"ok":true,"positions":[[1,512],[2,498]]}
  {"op":"run_action","name":"stand"}              -> {"ok":true}
  {"op":"state"}                                  -> {"ok":true,"walking":bool,"enabled":bool,"stopped":bool}

PROPOSED ADDITION (required by imu_source.py, not yet in the daemon):
  {"op":"imu"} -> {"ok":true,"accel":[x,y,z],"gyro":[x,y,z]}
    accel in m/s^2 and gyro in rad/s, calibrated the same way ROS1's
    /imu_corrected was (raw board g * 9.80665, deg/s -> rad/s, then the
    imu_calib.yaml scale/bias correction).  The board IMU lives on the same
    STM32 serial link as the servos (Board.get_imu() in
    ros_robot_controller_sdk.py), which motiond already owns -- so only the
    daemon can serve it.

Python 3.8 compatible.  Thread-safe (one in-flight request at a time),
reconnects automatically, short timeouts, raises MotiondError on any failure.
"""
import json
import socket
import threading

import os
SOCK_PATH = os.environ.get('MOTIOND_SOCK', '/tmp/motiond.sock')
DEFAULT_TIMEOUT = 2.0


class MotiondError(RuntimeError):
    """Raised when motiond is unreachable, times out, or replies ok:false."""
    pass


class MotiondClient:
    """Reconnecting, thread-safe motiond client.

    One socket, one request/response in flight at a time (guarded by a lock,
    which is all the newline-JSON framing needs).  A dead socket is dropped
    and reopened transparently on the next call; ``retries`` extra attempts
    are made per request before giving up with MotiondError.
    """

    def __init__(self, sock_path=SOCK_PATH, timeout=DEFAULT_TIMEOUT, retries=1):
        self.sock_path = sock_path
        self.timeout = float(timeout)
        self.retries = int(retries)
        self._lock = threading.RLock()
        self._sock = None
        self._buf = b''

    # ---- transport -----------------------------------------------------
    def _connect(self):
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(self.timeout)
        s.connect(self.sock_path)
        self._sock = s
        self._buf = b''

    def _drop(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None
        self._buf = b''

    def close(self):
        with self._lock:
            self._drop()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def _read_line(self):
        while b'\n' not in self._buf:
            chunk = self._sock.recv(4096)
            if not chunk:
                raise OSError('motiond closed the connection')
            self._buf += chunk
        line, self._buf = self._buf.split(b'\n', 1)
        return line

    def _request(self, obj):
        """Send one op, return the decoded reply dict (reply['ok'] is True)."""
        payload = (json.dumps(obj) + '\n').encode('utf-8')
        last_err = None
        with self._lock:
            for _attempt in range(self.retries + 1):
                try:
                    if self._sock is None:
                        self._connect()
                    self._sock.sendall(payload)
                    line = self._read_line()
                except (OSError, socket.timeout) as e:
                    last_err = e
                    self._drop()
                    continue
                try:
                    resp = json.loads(line.decode('utf-8'))
                except ValueError as e:
                    self._drop()
                    raise MotiondError('motiond sent unparseable reply for %r: %s'
                                       % (obj.get('op'), e))
                if not isinstance(resp, dict) or not resp.get('ok'):
                    err = resp.get('error') if isinstance(resp, dict) else resp
                    raise MotiondError('motiond refused %r: %s' % (obj.get('op'), err))
                return resp
        raise MotiondError('motiond unreachable at %s (%s: %s)'
                           % (self.sock_path, type(last_err).__name__, last_err))

    # ---- protocol ops --------------------------------------------------
    def command(self, cmd):
        """Walking-engine command: start|stop|enable|disable|enable_control|disable_control.

        Same strings the ROS1 SetWalkingCommand('/walking/command') service took.
        """
        self._request({'op': 'command', 'command': str(cmd)})

    def app_param(self, speed=3, height=0.025, x=0.0, y=0.0, angle=0.0):
        """Joystick-pathway walking params (was AppWalkingParam on /app/set_walking_param).

        x fwd m, y strafe m, angle deg (+ = turn LEFT, HW-verified), speed tier,
        height = body height (walking init_z_offset).
        """
        self._request({'op': 'app_param', 'speed': int(speed), 'height': float(height),
                       'x': float(x), 'y': float(y), 'angle': float(angle)})

    def set_param(self, params):
        """Full walking-param update (was WalkingParam on walking/set_param)."""
        if not isinstance(params, dict):
            raise TypeError('set_param wants a dict of WalkingParam fields')
        self._request({'op': 'set_param', 'params': params})

    def get_param(self):
        """Current walking params as a dict (was walking/get_param service)."""
        return self._request({'op': 'get_param'})['params']

    def set_servos_position(self, duration_ms, positions):
        """Move bus servos: positions = [[id, pos], ...], duration in ms."""
        self._request({'op': 'servos', 'duration_ms': int(duration_ms),
                       'positions': [[int(i), int(p)] for i, p in positions]})

    def get_servos_position(self, ids):
        """Read bus servo positions -> [[id, pos], ...]."""
        resp = self._request({'op': 'get_servos', 'ids': [int(i) for i in ids]})
        return [[int(i), int(p)] for i, p in resp['positions']]

    def run_action(self, name):
        """Run a .d6a action group by name (e.g. 'stand', 'recline_to_stand')."""
        self._request({'op': 'run_action', 'name': str(name)})

    def state(self):
        """Walking-engine state -> {'walking': bool, 'enabled': bool, 'stopped': bool}."""
        resp = self._request({'op': 'state'})
        return {'walking': bool(resp.get('walking')),
                'enabled': bool(resp.get('enabled')),
                'stopped': bool(resp.get('stopped'))}

    # ---- proposed extension (see module docstring) ---------------------
    def imu(self):
        """PROPOSED op: latest IMU sample -> (accel_xyz, gyro_xyz).

        accel in m/s^2 (upright gravity ~ +9.8 on Y), gyro in rad/s --
        the same units and axes /imu_corrected carried in ROS1.
        Raises MotiondError until the daemon grows the 'imu' op.
        """
        resp = self._request({'op': 'imu'})
        accel = [float(v) for v in resp['accel']]
        gyro = [float(v) for v in resp['gyro']]
        return accel, gyro
