#!/usr/bin/env python3
# encoding: utf-8
"""compat -- drop-in replacements for the mission code's ROS touchpoints.

Backed by motiond (see motiond_client.py) instead of rospy, so the mission code /
vision_service.py run identically under ROS1 or ROS2 hosts: the daemon owns the
robot, the mission code just talks JSON over /tmp/motiond.sock.

Replaces:
  ainex_kinematics.motion_manager.MotionManager   -> MotionManager (this file)
  ainex_kinematics.gait_manager.GaitManager       -> GaitManagerCompat
  rospy.Publisher('/app/set_walking_param', ...)  -> app_walk()
  rospy.ServiceProxy('/walking/command', ...)     -> app_cmd()

Signatures mirror the vendor classes exactly (including the *positions varargs
quirk of set_servos_position), so mission code migrates by swapping imports.

Python 3.8 compatible.
"""
import math
import time

try:
    from .motiond_client import MotiondClient, MotiondError, SOCK_PATH
except ImportError:                      # not imported as a package
    from motiond_client import MotiondClient, MotiondError, SOCK_PATH

__all__ = ['MotionManager', 'GaitManagerCompat', 'app_walk', 'app_cmd',
           'get_client', 'MotiondError']

# Defaults matching the mission code's app pathway constants
APP_SPEED = 3        # the joystick's proven tier
APP_HEIGHT = 0.025   # body height = walking init_z_offset baseline

_client = None


def get_client():
    """Shared MotiondClient (one socket for the whole process)."""
    global _client
    if _client is None:
        _client = MotiondClient(SOCK_PATH)
    return _client


# ======================================================================
# MotionManager -- same surface as ainex_kinematics.motion_manager
# ======================================================================
class MotionManager:
    """Drop-in for the vendor MotionManager (servo moves + action groups).

    The vendor published SetBusServosPosition on a topic and played .d6a files
    locally; here every call is one motiond op.  action_path is accepted for
    signature compatibility -- the daemon resolves action names itself.
    """

    def __init__(self, action_path=None, client=None):
        self.action_path = action_path            # kept for signature parity
        self._client = client or get_client()
        self.servo_position = {}                  # last commanded, like vendor

    def set_servos_position(self, duration, *positions):
        """Control multiple servos: duration ms, positions [[id, pos], ...].

        Vendor quirk preserved: the list arrives as the first vararg
        (mm.set_servos_position(400, [[23,500],[24,500]])).
        """
        pos_list = positions[0]
        for i in pos_list:
            self.servo_position[str(i[0])] = i[1]
        self._client.set_servos_position(int(duration), pos_list)

    def get_servos_position(self, *args, fake=False):
        """Read servo positions -> [[id, pos], ...] (ids as varargs, like vendor)."""
        if fake:
            return [[int(i), p] for i, p in self.servo_position.items()]
        return self._client.get_servos_position(list(args))

    def run_action(self, actNum):
        """Run an action group by name.

        NOTE: the vendor call BLOCKED until the .d6a finished playing; mission
        code depends on that for get-up sequences.  The daemon's run_action must
        do the same (reply only when the action completes) -- flagged in
        MIGRATION.md as a required daemon semantic.
        """
        if actNum is None:
            return
        self._client.run_action(actNum)

    def stop_action_group(self):
        """Vendor parity stub: the daemon has no action-stop op (unused by the
        mission code -- vendor run_action can't be interrupted either)."""
        pass


# ======================================================================
# GaitManagerCompat -- same surface as ainex_kinematics.gait_manager
# ======================================================================
class _WalkingParam:
    """Attribute-style stand-in for the WalkingParam ROS message, so mission
    lines like `gait.walking_param.init_x_offset = 0.0` work unchanged."""

    def __init__(self, d=None):
        for k, v in (d or {}).items():
            setattr(self, k, v)

    def to_dict(self):
        return {k: v for k, v in self.__dict__.items() if not k.startswith('_')}


class _ParamPub:
    """Stand-in for the vendor's `param_pub` rospy.Publisher: mission code calls
    `gait.param_pub.publish(gait.walking_param)`."""

    def __init__(self, client):
        self._client = client

    def publish(self, param):
        d = param.to_dict() if hasattr(param, 'to_dict') else dict(param)
        self._client.set_param(d)


class GaitManagerCompat:
    """Drop-in for the vendor GaitManager, mapped onto motiond ops.

    Derived from gait_manager.py ground truth:
      enable()  -> ServiceProxy('walking/command')('enable')   => command('enable')
      disable() -> ServiceProxy('walking/command')('disable')  => command('disable')
      stop()    -> ServiceProxy('walking/command')('stop')     => command('stop')
      param_pub.publish(WalkingParam)                          => set_param
      ServiceProxy('walking/get_param')()                      => get_param
      is_walking (from walking/is_walking topic)               => state()['walking']
    """

    # same tuning tables as the vendor (used by move()/set_step callers)
    dsp_ratio = [[300, 0.2, 0.02],
                 [400, 0.2, 0.02],
                 [500, 0.2, 0.02],
                 [600, 0.1, 0.04]]
    body_height_range = [0.015, 0.06]
    x_amplitude_range = [0.0, 0.02]
    y_amplitude_range = [0.0, 0.02]
    step_height_range = [0.01, 0.04]
    rotation_angle_range = [0, 10]
    arm_swap_range = [0, 60]
    y_swap_range = [0, 0.05]
    dsp_ratio_range = [0, 1]

    def __init__(self, client=None):
        self._client = client or get_client()
        self.state = 'enable'
        self.err = 1e-8
        # vendor fetched current params at construction (walking/get_param)
        self.walking_param = _WalkingParam(self._client.get_param())
        self.param_pub = _ParamPub(self._client)

    # -- live engine state ------------------------------------------------
    @property
    def is_walking(self):
        """Vendor kept this fresh via the walking/is_walking subscriber; here
        it is a live daemon read (False if the daemon is unreachable)."""
        try:
            return self._client.state()['walking']
        except MotiondError:
            return False

    @is_walking.setter
    def is_walking(self, value):     # vendor code never sets it, but be safe
        pass

    # -- the three commands the task maps ---------------------------------
    def enable(self):
        self._client.command('enable')
        self.state = 'enable'

    def disable(self):
        self._client.command('disable')
        self.state = 'disable'

    def stop(self):
        self._client.command('stop')
        self.state = 'stop'

    # -- vendor param plumbing, faithfully reproduced ---------------------
    def get_gait_param(self):
        """Same field mapping as the vendor's get_gait_param()."""
        wp = self.walking_param
        return {
            'init_x_offset': wp.init_x_offset,
            'init_y_offset': wp.init_y_offset,
            'body_height': wp.init_z_offset,
            'init_roll_offset': wp.init_roll_offset,
            'init_pitch_offset': wp.init_pitch_offset,
            'init_yaw_offset': wp.init_yaw_offset,
            'hip_pitch_offset': wp.hip_pitch_offset,
            'step_fb_ratio': wp.step_fb_ratio,
            'step_height': wp.z_move_amplitude,
            'angle_move_amplitude': wp.angle_move_amplitude,
            'z_swap_amplitude': wp.z_swap_amplitude,
            'pelvis_offset': wp.pelvis_offset,
            'move_aim_on': wp.move_aim_on,
        }

    def _apply_gait_param(self, walking_param):
        wp = self.walking_param
        wp.init_x_offset = walking_param['init_x_offset']
        wp.init_y_offset = walking_param['init_y_offset']
        wp.init_z_offset = walking_param['body_height']
        wp.init_roll_offset = walking_param['init_roll_offset']
        wp.init_pitch_offset = walking_param['init_pitch_offset']
        wp.init_yaw_offset = walking_param['init_yaw_offset']
        wp.hip_pitch_offset = walking_param['hip_pitch_offset']
        wp.step_fb_ratio = walking_param['step_fb_ratio']
        wp.z_move_amplitude = walking_param['step_height']
        wp.angle_move_amplitude = walking_param['angle_move_amplitude']
        wp.z_swap_amplitude = walking_param['z_swap_amplitude']
        wp.pelvis_offset = walking_param['pelvis_offset']
        wp.move_aim_on = walking_param['move_aim_on']

    def update_pose(self, walking_param):
        if self.state == 'disable':
            self.enable()
        self._check_pose(walking_param)
        self._apply_gait_param(walking_param)
        wp = self.walking_param
        wp.x_move_amplitude = 0
        wp.y_move_amplitude = 0
        wp.angle_move_amplitude = 0
        self._client.set_param(wp.to_dict())

    def _check_pose(self, walking_param):
        if walking_param['body_height'] - self.body_height_range[1] > self.err \
                or walking_param['body_height'] - self.body_height_range[0] < -self.err:
            raise Exception('body_height %s out of range(0.015~0.06)'
                            % walking_param['body_height'])
        if walking_param['step_height'] - self.step_height_range[1] > self.err \
                or walking_param['step_height'] - self.step_height_range[0] < -self.err:
            raise Exception('step_height %s out of range(0.01~0.04)'
                            % walking_param['step_height'])

    def update_param(self, step_velocity, x_amplitude, y_amplitude, rotation_angle,
                     walking_param=None, arm_swap=30, step_num=0):
        """Same validation + field mapping as the vendor, one set_param at the end."""
        if step_velocity[0] < 0:
            raise Exception('period_time cannot be negative')
        if step_velocity[1] > self.dsp_ratio_range[1] or step_velocity[1] < self.dsp_ratio_range[0]:
            raise Exception('dsp_ratio %s out of range(0~1)' % step_velocity[1])
        if step_velocity[2] - self.y_swap_range[1] > self.err or step_velocity[2] < self.y_swap_range[0]:
            raise Exception('y_swap %s out of range(0~0.05)' % step_velocity[2])
        if abs(x_amplitude) - self.x_amplitude_range[1] > self.err:
            raise Exception('x_amplitude %s out of range(-0.02~0.02)' % x_amplitude)
        if abs(y_amplitude) - self.y_amplitude_range[1] > self.err:
            raise Exception('y_amplitude %s out of range(-0.02~0.02)' % y_amplitude)
        if abs(rotation_angle) - self.rotation_angle_range[1] > self.err:
            raise Exception('rotation_angle %s out of range(-10~10)' % rotation_angle)
        if abs(arm_swap) - self.arm_swap_range[1] > self.err or arm_swap < self.arm_swap_range[0]:
            raise Exception('arm_swap %s out of range(0~60)' % arm_swap)
        if step_num < 0:
            raise Exception('step_num cannot be negative')

        if walking_param is not None:
            self._check_pose(walking_param)
            self._apply_gait_param(walking_param)

        wp = self.walking_param
        wp.period_time = step_velocity[0]
        wp.dsp_ratio = step_velocity[1]
        wp.y_swap_amplitude = step_velocity[2]
        wp.x_move_amplitude = x_amplitude
        wp.y_move_amplitude = y_amplitude
        wp.angle_move_amplitude = rotation_angle
        wp.arm_swing_gain = math.radians(arm_swap)
        wp.period_times = step_num
        self._client.set_param(wp.to_dict())

    def set_body_height(self, body_height, use_time):
        if self.state == 'disable':
            self.enable()
        wp = self.walking_param
        times = int(abs(body_height - wp.init_z_offset) / 0.005)
        for _ in range(times):
            wp.init_z_offset += math.copysign(0.005, body_height - wp.init_z_offset)
            self._client.set_param(wp.to_dict())
            time.sleep(use_time / times)

    def set_step(self, step_velocity, x_amplitude, y_amplitude, rotation_angle,
                 walking_param=None, arm_swap=30, step_num=0):
        """Vendor-faithful set_step: params first, then 'start'; with step_num,
        block until the engine walks and finishes (vendor watched
        walking/is_walking -- here we poll state())."""
        try:
            self.update_param(step_velocity, x_amplitude, y_amplitude, rotation_angle,
                              walking_param, arm_swap, step_num)
            if step_num != 0:
                self.walking_param = _WalkingParam(self._client.get_param())
                self._client.command('start')
                t0 = time.time()
                while not self.is_walking and time.time() - t0 < 5.0:
                    time.sleep(0.01)
                while self.is_walking:
                    time.sleep(0.01)
                self.state = 'step_walking'
            else:
                if self.state != 'walking':
                    self.state = 'walking'
                    self.walking_param = _WalkingParam(self._client.get_param())
                    self._client.command('start')
        except BaseException as e:
            print(e)
            return

    def move(self, step_velocity, x_amplitude, y_amplitude, rotation_angle,
             arm_swap=30, step_num=0):
        if 0 < step_velocity < 5:
            self.set_step(self.dsp_ratio[step_velocity - 1], x_amplitude, y_amplitude,
                          rotation_angle, self.get_gait_param(), arm_swap, step_num)


# ======================================================================
# app pathway helpers -- mirror the mission code's app_cmd / app_walk
# ======================================================================
def app_cmd(c, client=None):
    """Was: rospy.ServiceProxy('/walking/command', SetWalkingCommand)(c).
    Same never-raise contract as the mission code's helper (prints on failure)."""
    try:
        (client or get_client()).command(c)
    except Exception as e:
        print('  app_cmd(%s) failed: %s' % (c, e))


def app_walk(x, angle, y=0.0, speed=APP_SPEED, height=APP_HEIGHT, client=None):
    """Was: publish AppWalkingParam on /app/set_walking_param while walking.
    x fwd m, angle deg (POSITIVE = LEFT, HW-verified), y strafe m.

    Raises MotiondError on failure -- same as the original, where a dead
    publisher raised out of app_walk (callers wrap it in try/except).
    NOTE: the original also POSTed a /hint yaw to the vision service; that is
    HTTP, not ROS, so it stays in the mission code (see MIGRATION.md).
    """
    (client or get_client()).app_param(speed=speed, height=height,
                                       x=float(x), y=float(y), angle=float(angle))
