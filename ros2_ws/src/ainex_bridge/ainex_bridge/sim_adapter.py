#!/usr/bin/env python3
"""sim_adapter: motiond --sim UDP joint stream -> Gazebo position controller.

Listens for the datagrams motiond emits in --sim mode (one JSON object per
datagram: {"t": <monotonic float>, "joints": {"<joint_name>": <radians>, ...}})
and republishes them as std_msgs/Float64MultiArray on
/joint_group_position_controller/commands, in the FIXED joint order given by
the 'joint_order' string-array parameter.

CONTRACT with ainex_gazebo: 'joint_order' MUST be exactly the joints list (same
names, same order) configured for the ros2_control joint_group_position_controller
that this topic feeds. Supply it at launch (sim_adapter.launch.py carries the
proposed default order); a mismatch silently commands the wrong joints.

Behaviour:
- unknown joint names in a datagram are ignored;
- joints missing from a datagram hold their last value (seeded to all zeros);
- publishes at most 'publish_rate' Hz (default 50), coalescing faster input;
- stops publishing when no datagram has arrived for 'staleness_timeout' s.

The datagram->ordered-array conversion lives in the plain functions
parse_datagram() / merge_joint_positions() so it can be unit-tested with a
bare python3 (this module imports without rclpy installed).
"""
import json
import socket
import threading
import time

try:  # keep the module importable without ROS for the pure-python unit tests
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
except ImportError:  # pragma: no cover - exercised only in the no-ROS tests
    rclpy = None
    Node = object

# Proposed ainex_gazebo convention: the motiond walking joints in vendor
# joint_index order, then the head. Keep in sync with the gazebo controller
# config (see module docstring).
DEFAULT_JOINT_ORDER = [
    'r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee', 'r_ank_pitch', 'r_ank_roll',
    'l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 'l_knee', 'l_ank_pitch', 'l_ank_roll',
    'r_sho_pitch', 'l_sho_pitch',
    'head_pan', 'head_tilt',
]


def parse_datagram(data):
    """Decode one motiond sim datagram (bytes) -> {joint_name: radians}.

    Returns None if the datagram is not a JSON object with a 'joints' object.
    Non-string keys and non-numeric (or boolean) values are dropped.
    """
    try:
        msg = json.loads(data.decode('utf-8'))
    except (ValueError, UnicodeDecodeError):
        return None
    if not isinstance(msg, dict):
        return None
    joints = msg.get('joints')
    if not isinstance(joints, dict):
        return None
    out = {}
    for name, value in joints.items():
        if isinstance(name, str) and isinstance(value, (int, float)) \
                and not isinstance(value, bool):
            out[name] = float(value)
    return out


def merge_joint_positions(joints, joint_order, last_values):
    """Merge a parsed datagram into an ordered command array.

    joints:      {joint_name: radians} from parse_datagram()
    joint_order: fixed list of joint names (the controller's order)
    last_values: previous ordered array (len == len(joint_order))

    Returns a NEW list in joint_order: joints present in the datagram take
    their new value, missing joints hold last_values, unknown datagram joints
    (not in joint_order) are ignored.
    """
    out = list(last_values)
    for i, name in enumerate(joint_order):
        if name in joints:
            out[i] = joints[name]
    return out


class SimAdapter(Node):
    def __init__(self):
        super().__init__('sim_adapter')
        self.declare_parameter('udp_port', 9910)
        self.declare_parameter('udp_bind', '127.0.0.1')
        self.declare_parameter('joint_order', DEFAULT_JOINT_ORDER)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('staleness_timeout', 1.0)

        self.joint_order = [str(j) for j in self.get_parameter('joint_order').value]
        if not self.joint_order:
            raise ValueError("'joint_order' must be a non-empty string array "
                             "matching the ainex_gazebo controller joints list")
        port = int(self.get_parameter('udp_port').value)
        bind = str(self.get_parameter('udp_bind').value)
        rate = float(self.get_parameter('publish_rate').value)
        self.staleness_timeout = float(self.get_parameter('staleness_timeout').value)

        self._lock = threading.Lock()
        self._values = [0.0] * len(self.joint_order)  # seed: all zeros
        self._last_rx = None   # monotonic stamp of the last good datagram
        self._stale_logged = False

        self.pub = self.create_publisher(
            Float64MultiArray, 'joint_group_position_controller/commands', 10)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((bind, port))
        self._sock.settimeout(0.5)
        self._shutdown = threading.Event()
        self._rx_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._rx_thread.start()

        # The timer is the rate limiter: datagrams only update state; the
        # newest merged array is published at most `rate` Hz (coalescing).
        self.create_timer(1.0 / max(rate, 1e-3), self._publish_tick)
        self.get_logger().info(
            'sim_adapter: udp %s:%d -> joint_group_position_controller/commands '
            '(%d joints, <=%.0f Hz, stale after %.1fs)'
            % (bind, port, len(self.joint_order), rate, self.staleness_timeout))

    def _recv_loop(self):
        while not self._shutdown.is_set():
            try:
                data, _ = self._sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            joints = parse_datagram(data)
            if joints is None:
                continue
            with self._lock:
                self._values = merge_joint_positions(
                    joints, self.joint_order, self._values)
                self._last_rx = time.monotonic()

    def _publish_tick(self):
        with self._lock:
            last_rx = self._last_rx
            values = list(self._values)
        if last_rx is None:
            return  # never received anything: publish nothing
        if time.monotonic() - last_rx > self.staleness_timeout:
            if not self._stale_logged:
                self._stale_logged = True
                self.get_logger().warning(
                    'sim joint stream stale (> %.1fs with no datagrams); '
                    'pausing publishes' % self.staleness_timeout)
            return
        if self._stale_logged:
            self._stale_logged = False
            self.get_logger().info('sim joint stream resumed')
        msg = Float64MultiArray()
        msg.data = values
        self.pub.publish(msg)

    def shutdown(self):
        self._shutdown.set()
        try:
            self._sock.close()
        except OSError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SimAdapter()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
