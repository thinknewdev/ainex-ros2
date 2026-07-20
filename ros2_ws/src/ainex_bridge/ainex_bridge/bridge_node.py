#!/usr/bin/env python3
"""ainex_bridge: ROS2 facade for the AiNex motion daemon (motiond).

Replicates the vendor ROS1 ainex_controller topic/service surface and forwards
every call over a Unix socket to motiond, which owns the walking engine and
servo bus. No motion logic lives here.
"""
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from ainex_interfaces.msg import AppWalkingParam, HeadState, WalkingParam
from ainex_interfaces.srv import (GetWalkingParam, GetWalkingState,
                                  SetWalkingCommand, SetWalkingParam)
from ainex_bridge_interfaces.msg import BusServoPosition, SetBusServosPosition
from ainex_bridge_interfaces.srv import GetBusServosPosition

from ainex_bridge.motiond_client import MotiondClient

# Servo pulse <-> radian conversion, from the vendor servo_controller.yaml:
# head servos have init=500, min=0, max=1000 (not flipped).
ENCODER_TICKS_PER_RADIAN = 180.0 / 3.1415926 / 240.0 * 1000.0
HEAD_PAN_ID = 23
HEAD_TILT_ID = 24
HEAD_INIT_PULSE = 500
PULSE_RANGE = (0, 1000)

# WalkingParam msg fields == vendor walking_param.yaml keys (see the ROS1
# set_walking_param_callback / get_walking_param_callback field mapping).
WALKING_PARAM_KEYS = [
    'init_x_offset', 'init_y_offset', 'init_z_offset',
    'init_roll_offset', 'init_pitch_offset', 'init_yaw_offset',
    'hip_pitch_offset',
    'period_time', 'dsp_ratio', 'step_fb_ratio',
    'x_move_amplitude', 'y_move_amplitude', 'z_move_amplitude',
    'angle_move_amplitude',
    'y_swap_amplitude', 'z_swap_amplitude', 'pelvis_offset',
    'arm_swing_gain',
]


def angle_to_pulse(angle_rad):
    pulse = HEAD_INIT_PULSE + int(round(angle_rad * ENCODER_TICKS_PER_RADIAN))
    return max(PULSE_RANGE[0], min(PULSE_RANGE[1], pulse))


class BridgeNode(Node):
    def __init__(self):
        super().__init__('ainex_bridge')
        self.declare_parameter('socket_path', '/tmp/motiond.sock')
        self.declare_parameter('request_timeout', 2.0)
        self.declare_parameter('state_poll_hz', 5.0)

        socket_path = self.get_parameter('socket_path').value
        timeout = float(self.get_parameter('request_timeout').value)
        poll_hz = float(self.get_parameter('state_poll_hz').value)

        self.client = MotiondClient(socket_path, timeout, self.get_logger())
        cbg = ReentrantCallbackGroup()

        # --- services (vendor names) ---
        self.create_service(SetWalkingCommand, 'walking/command',
                            self.walking_command_cb, callback_group=cbg)
        self.create_service(SetWalkingParam, 'walking/set_param',
                            self.set_walking_param_cb, callback_group=cbg)
        self.create_service(GetWalkingParam, 'walking/get_param',
                            self.get_walking_param_cb, callback_group=cbg)
        self.create_service(GetWalkingState, 'walking/is_walking',
                            self.is_walking_cb, callback_group=cbg)
        self.create_service(GetBusServosPosition,
                            'ros_robot_controller/bus_servo/get_position',
                            self.get_servos_cb, callback_group=cbg)

        # --- subscriptions (vendor names) ---
        self.create_subscription(AppWalkingParam, 'app/set_walking_param',
                                 self.app_walking_param_cb, 1,
                                 callback_group=cbg)
        self.create_subscription(String, 'app/set_action',
                                 self.set_action_cb, 1, callback_group=cbg)
        self.create_subscription(HeadState, 'head_pan_controller/command',
                                 self.head_pan_cb, 5, callback_group=cbg)
        self.create_subscription(HeadState, 'head_tilt_controller/command',
                                 self.head_tilt_cb, 5, callback_group=cbg)
        self.create_subscription(SetBusServosPosition,
                                 'ros_robot_controller/bus_servo/set_position',
                                 self.set_servos_cb, 5, callback_group=cbg)

        # --- walking state publisher (vendor publishes Bool on
        # 'walking/is_walking' at state transitions; mirrored by polling) ---
        self.walk_state_pub = self.create_publisher(Bool,
                                                    'walking/is_walking', 1)
        self._last_walking = None
        if poll_hz > 0:
            self.create_timer(1.0 / poll_hz, self.poll_state,
                              callback_group=cbg)

        self.get_logger().info(
            'ainex_bridge up, motiond socket: {}'.format(socket_path))

    # ------------------------------------------------------------------
    # walking services
    # ------------------------------------------------------------------
    def walking_command_cb(self, request, response):
        ok, _ = self.client.request_ok(
            {'op': 'command', 'command': request.command})
        response.result = ok
        return response

    def set_walking_param_cb(self, request, response):
        p = request.parameters
        params = {key: float(getattr(p, key)) for key in WALKING_PARAM_KEYS}
        params['period_times'] = int(p.period_times)
        params['move_aim_on'] = bool(p.move_aim_on)
        ok, _ = self.client.request_ok({'op': 'set_param', 'params': params})
        response.result = ok
        return response

    def get_walking_param_cb(self, request, response):
        ok, reply = self.client.request_ok({'op': 'get_param'})
        if ok:
            params = reply.get('params', {})
            p = WalkingParam()
            for key in WALKING_PARAM_KEYS:
                setattr(p, key, float(params.get(key, 0.0)))
            p.move_aim_on = bool(params.get('move_aim_on', False))
            p.period_times = 0  # vendor always returns 0 here
            response.parameters = p
        return response

    def is_walking_cb(self, request, response):
        ok, reply = self.client.request_ok({'op': 'state'})
        response.state = bool(reply.get('walking', False)) if ok else False
        response.message = 'is_walking'
        return response

    # ------------------------------------------------------------------
    # walking topics
    # ------------------------------------------------------------------
    def app_walking_param_cb(self, msg):
        self.client.request_ok({
            'op': 'app_param',
            'speed': int(msg.speed),
            'height': float(msg.height),
            'x': float(msg.x),
            'y': float(msg.y),
            'angle': float(msg.angle),
        })

    def set_action_cb(self, msg):
        self.client.request_ok({'op': 'run_action', 'name': msg.data})

    # ------------------------------------------------------------------
    # head / bus servos
    # ------------------------------------------------------------------
    def _head_cb(self, msg, servo_id):
        # Vendor: set_servos_position(int(duration*1000),
        #   [[id, angle2pulse(id, position)]]) with position in radians.
        self.client.request_ok({
            'op': 'servos',
            'duration_ms': int(msg.duration * 1000),
            'positions': [[servo_id, angle_to_pulse(msg.position)]],
        })

    def head_pan_cb(self, msg):
        self._head_cb(msg, HEAD_PAN_ID)

    def head_tilt_cb(self, msg):
        self._head_cb(msg, HEAD_TILT_ID)

    def set_servos_cb(self, msg):
        positions = [[int(p.id), int(p.position)] for p in msg.position]
        if not positions:
            return
        self.client.request_ok({
            'op': 'servos',
            'duration_ms': int(msg.duration * 1000),
            'positions': positions,
        })

    def get_servos_cb(self, request, response):
        ok, reply = self.client.request_ok(
            {'op': 'get_servos', 'ids': [int(i) for i in request.ids]})
        response.success = ok
        if ok:
            response.positions = [
                BusServoPosition(id=int(sid), position=int(pos))
                for sid, pos in reply.get('positions', [])
            ]
        return response

    # ------------------------------------------------------------------
    # state polling
    # ------------------------------------------------------------------
    def poll_state(self):
        try:
            reply = self.client.request({'op': 'state'})
        except Exception:
            return  # daemon down; try again next tick
        if not reply.get('ok', False):
            return
        walking = bool(reply.get('walking', False))
        if walking != self._last_walking:
            self._last_walking = walking
            self.walk_state_pub.publish(Bool(data=walking))


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.client.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
