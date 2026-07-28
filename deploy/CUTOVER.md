# ROS1 -> ROS2 cutover runbook

How to move an AiNex from the shipped ROS1 Noetic stack to running purely on the
ROS2 port. This is the exact sequence used for the first on-robot cutover.

## Starting point (hybrid)

A stock AiNex runs everything from one launch, `roslaunch ainex_bringup
bringup.launch` (started at boot by `start_node.service`), inside the ROS1
`ainex` container. That launch owns:

- the head camera pipeline: `usb_cam` -> `image_proc/rectify` -> `web_video_server` on `:8080`
- vendor demo nodes (`/app`, `/color_detection`, `/face_detect`, vendor `/joystick`)
- the rosbridge websocket / rosapi

The motion side is already ROS-independent once `motiond` (this repo) is the
serial-bus owner: the web API, mission code, and joystick talk to it over the
UDS socket, not over ROS. So the **only** thing the live application stack needs
from ROS1 is the `:8080` camera feed. Verify this on your own robot before
cutting over:

- the active vision service reads frames from
  `http://127.0.0.1:8080/snapshot?topic=/camera/image_raw` (no `rospy`), and
- nothing references rosbridge (`:9090`).

## 1. Build the ROS2 camera runtime

The base `ros:humble` image lacks the camera deps. Build the runtime image and
the `ainex_camera` package:

```bash
docker build -f deploy/Dockerfile.camera -t ainex_ros2:cam .

docker run -d --name ainex_ros2 --privileged --net host --restart unless-stopped \
    -v <host_shared_src>:/share \
    -v <host_ros2_ws>:/ws \
    ainex_ros2:cam sleep infinity

docker exec ainex_ros2 bash -lc \
    'source /opt/ros/humble/setup.bash && cd /ws && colcon build --packages-select ainex_camera'
```

`--privileged` (or `--device /dev/video0`) is what lets the container open the
head camera. Host networking is required so `web_video_server` on `:8080` is
reachable by the vision service running in the other container.

## 2. Swap the camera (camera-only, no motion -> no fall risk)

Only one process can hold `/dev/video0`, so ROS1's camera must stop before the
ROS2 camera starts. Do it as one step with automatic rollback:

1. Stop the ROS1 launch: `docker exec <ainex_container> pkill -INT -f 'roslaunch ainex_bringup'` (then force-kill any camera stragglers).
2. Start the ROS2 camera: `docker exec -d ainex_ros2 bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; ros2 launch ainex_camera camera.launch.py'`.
3. Poll `http://127.0.0.1:8080/snapshot?topic=/camera/image_raw` for a real JPEG (magic `ff d8`, > 2 KB, size varies between requests = live frames).
4. If frames don't appear within ~45 s, kill the ROS2 camera and restart the ROS1 launch — you are back to the working hybrid.

`ainex_camera` serves the same `:8080/snapshot?topic=...` endpoint the ROS1
`web_video_server` did, so the vision service needs no change; confirm it still
reports detections after the swap.

## 3. Make it durable

```bash
# stop ROS1 from coming back at boot
sudo systemctl disable --now start_node

# run the ROS2 camera as a managed service (see deploy/ainex_ros2_camera.service)
sudo cp deploy/ainex_ros2_camera.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now ainex_ros2_camera
```

Optionally `docker commit ainex_ros2 ainex_ros2:cam` after the first
`colcon build` so the installed deps survive a container rebuild.

## 4. Verify pure ROS2

- ROS1 gone: no `rosmaster` / `usb_cam` / `roslaunch ainex_bringup` processes.
- Camera: live JPEG frames on `:8080`.
- Vision: detections still reported by the vision service.
- Motion (unaffected — `motiond` never used ROS): telemetry (`/motion/status`)
  reports IMU + battery, and a small head nudge (`POST /motion/head?pan=..`)
  returns 200 and physically moves the head.

## Rollback (any time)

```bash
sudo systemctl disable --now ainex_ros2_camera
sudo systemctl enable --now start_node   # restores the ROS1 bringup at boot
# or immediately: docker exec -u <user> <ainex_container> \
#   bash -lc 'source <ros_setup>; roslaunch ainex_bringup bringup.launch'
```
