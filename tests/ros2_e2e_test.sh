#!/bin/bash
# 1. start the bridge in a persistent humble container (host net shares the UDS + UDP with ainex container)
sudo docker rm -f humble_bridge 2>/dev/null
sudo docker run -d --name humble_bridge --network host \
  -v /home/pi/ros2_port/ros2_ws:/ws -v /home/pi/docker/src:/share ros:humble \
  bash -c "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 run ainex_bridge bridge --ros-args -p socket_path:=/share/motiond.sock"
sleep 12
# 2. UDP listener in background on host
timeout 25 python3 - > /tmp/udp_capture.txt 2>&1 <<'PY' &
import socket, json
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("127.0.0.1", 9910))
s.settimeout(20)
n = 0
try:
    while n < 60:
        d, _ = s.recvfrom(65535)
        j = json.loads(d)
        n += 1
        if n in (1, 30, 60):
            k = sorted(j["joints"].keys())[:4]
            print("datagram", n, "joints:", len(j["joints"]), "sample:", {x: round(j["joints"][x], 4) for x in k})
except socket.timeout:
    print("TIMEOUT after", n, "datagrams")
print("TOTAL:", n)
PY
UDP_PID=$!
# 3. drive it from ROS2
EX="sudo docker exec humble_bridge bash -c"
$EX "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 service call /walking/command ainex_interfaces/srv/SetWalkingCommand '{command: enable_control}'" | tail -1
$EX "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 topic pub -1 /app/set_walking_param ainex_interfaces/msg/AppWalkingParam '{speed: 3, height: 0.025, x: 0.008, y: 0.0, angle: 0.0}'" >/dev/null
$EX "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 service call /walking/command ainex_interfaces/srv/SetWalkingCommand '{command: start}'" | tail -1
sleep 6
$EX "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 service call /walking/is_walking ainex_interfaces/srv/GetWalkingState '{}'" | tail -1
$EX "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 service call /walking/command ainex_interfaces/srv/SetWalkingCommand '{command: stop}'" | tail -1
$EX "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 service call /walking/is_walking ainex_interfaces/srv/GetWalkingState '{}'" | tail -1
wait $UDP_PID
echo "=== UDP capture:"
cat /tmp/udp_capture.txt
sudo docker rm -f humble_bridge >/dev/null
