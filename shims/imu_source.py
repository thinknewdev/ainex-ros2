#!/usr/bin/env python3
# encoding: utf-8
"""imu_source -- ROS-free IMU feed for the mission code.

Where the IMU actually lives (traced through ros1_src):
  * The IMU is on the STM32 servo-control board, read over the SAME serial
    link the servos use: Board.get_imu() in
    ros_robot_controller/src/ros_robot_controller/ros_robot_controller_sdk.py
    (raw units: g and deg/s).
  * ros_robot_controller_node.py polled it at ~100 Hz and published
    /ros_robot_controller/imu_raw (accel * 9.80665 -> m/s^2,
    gyro deg -> rad/s).
  * ainex_peripherals/launch/imu.launch ran imu_calib/apply_calib over
    imu_raw to produce /imu_corrected -- the topic the mission code subscribes.

It is NOT an I2C/sysfs device, so there is no direct ROS-free read path from
a second process: the daemon owns the board serial port.  Therefore this
module needs ONE daemon protocol addition (documented in MIGRATION.md):

  {"op":"imu"} -> {"ok":true,"accel":[x,y,z],"gyro":[x,y,z]}

  accel in m/s^2 (upright gravity ~ +9.8 on Y), gyro in rad/s, with the
  imu_calib.yaml correction applied daemon-side (i.e. serve what
  /imu_corrected carried, not raw board counts).

ImuFeed matches the mission code's usage semantics:
  the mission code                      here
  ----------                      ----
  _accel  (list [ax, ay, az])     feed.accel
  _yaw['deg'] (integrated yaw     feed.gyro_deg          (read)
     about the UP axis, Y)        feed.gyro_deg = 0.0    (reset, like
                                    the mission code does before a measured turn)
  subscriber callback rate        background poll thread (default 50 Hz)

Python 3.8 compatible.
"""
import math
import threading
import time

try:
    from .motiond_client import MotiondClient, MotiondError, SOCK_PATH
except ImportError:                      # not imported as a package
    from motiond_client import MotiondClient, MotiondError, SOCK_PATH

__all__ = ['ImuFeed']

_UP_AXIS_INDEX = {'x': 0, 'y': 1, 'z': 2}


class ImuFeed:
    """Background-threaded IMU reader with a gyro yaw integrator.

    .accel     latest [ax, ay, az] in m/s^2 (upright gravity ~ +9.8 on Y;
               starts at the upright default so tilt/fall math is sane
               before the first sample, same as the mission code's initial _accel).
    .gyro      latest [gx, gy, gz] in rad/s.
    .gyro_deg  integrated rotation (degrees) about the UP axis (Y), exactly
               like the mission code's _yaw['deg']: deg += degrees(gyro_y) * dt.
               Assignable: `feed.gyro_deg = 0.0` re-zeroes before a turn.
    .age       seconds since the last successful sample (inf if never).
    .ok        True when a sample landed within the last second.
    """

    def __init__(self, client=None, poll_hz=50.0, up_axis='y', autostart=True):
        self._client = client or MotiondClient(SOCK_PATH)
        self._period = 1.0 / float(poll_hz)
        self._up = _UP_AXIS_INDEX[up_axis.lower()]
        self._lock = threading.Lock()
        self.accel = [0.0, 9.8, 0.0]          # upright default, like the mission code
        self.gyro = [0.0, 0.0, 0.0]
        self._yaw_deg = 0.0
        self._yaw_t = None                    # last integration timestamp
        self._sample_t = 0.0                  # last successful sample wall time
        self._run = False
        self._thread = None
        if autostart:
            self.start()

    # -- lifecycle -------------------------------------------------------
    def start(self):
        if self._run:
            return self
        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._run = False

    # -- the mission code-compatible surface ------------------------------------
    @property
    def gyro_deg(self):
        with self._lock:
            return self._yaw_deg

    @gyro_deg.setter
    def gyro_deg(self, value):
        with self._lock:
            self._yaw_deg = float(value)

    def reset_yaw(self):
        self.gyro_deg = 0.0

    @property
    def age(self):
        return (time.time() - self._sample_t) if self._sample_t else float('inf')

    @property
    def ok(self):
        return self.age < 1.0

    def wait_ready(self, timeout=2.0):
        """Block until the first sample arrives (the mission code slept 0.3s and
        hoped; this actually checks). Returns True if data is flowing."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self._sample_t:
                return True
            time.sleep(0.02)
        return False

    # -- convenience mirrors of the mission code's gravity math -----------------
    def tilt_deg(self):
        """Body angle from vertical, from the gravity vector (the mission code's tilt_deg)."""
        ax, ay, az = self.accel
        g = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
        up = self.accel[self._up]
        return math.degrees(math.acos(max(-1.0, min(1.0, up / g))))

    # -- internals --------------------------------------------------------
    def _loop(self):
        while self._run:
            t_next = time.time() + self._period
            try:
                accel, gyro = self._client.imu()
            except MotiondError:
                # daemon down / op missing: mark stale, break the integrator so
                # the gap does not integrate garbage when samples resume
                with self._lock:
                    self._yaw_t = None
                time.sleep(0.25)
                continue
            now = time.time()
            with self._lock:
                self.accel = accel
                self.gyro = gyro
                if self._yaw_t is not None:
                    dt = now - self._yaw_t
                    if 0.0 < dt < 0.5:        # sane gap only, like a live topic
                        self._yaw_deg += math.degrees(gyro[self._up]) * dt
                self._yaw_t = now
                self._sample_t = now
            delay = t_next - time.time()
            if delay > 0:
                time.sleep(delay)


if __name__ == '__main__':
    # quick check: prints accel / yaw at 2 Hz (requires the daemon 'imu' op)
    feed = ImuFeed()
    if not feed.wait_ready(3.0):
        print('no IMU samples -- is motiond running with the "imu" op?')
    while True:
        a = feed.accel
        print('accel=(%.2f,%.2f,%.2f) tilt=%.1f yaw=%+.1f age=%.2fs'
              % (a[0], a[1], a[2], feed.tilt_deg(), feed.gyro_deg, feed.age))
        time.sleep(0.5)
