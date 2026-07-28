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

## 3b. ROS2 motion bridge (optional but recommended)

`motiond` is reachable directly over its UDS socket, but to expose the full ROS2
motion topic/service surface (`/walking/command`, `/head_pan_controller/command`,
`/ros_robot_controller/bus_servo/set_position`, ...) run `ainex_bridge`.

The bridge runs in the Humble container (which mounts the shared dir at `/share`)
and must reach `motiond`'s socket. `motiond` binds a container-local socket by
default (`/tmp/motiond.sock`), which the bridge's container cannot see. Fix: have
`motiond` bind the socket on the **shared** path and symlink the legacy `/tmp`
path to it, so every existing consumer keeps working unchanged (`connect()`,
`-S`, and `os.path.exists` all follow the symlink):

```sh
# in motiond's launch script, before exec'ing motiond:
SOCK=<shared_dir>/motiond.sock          # e.g. /home/ubuntu/share/src/motiond.sock
rm -f "$SOCK" /tmp/motiond.sock
ln -sf "$SOCK" /tmp/motiond.sock
exec python3 -u .../motiond.py --socket "$SOCK"
```

Then run the bridge against the shared path and install its service:

```bash
docker exec -d ainex_ros2 bash -lc \
  'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; \
   ros2 run ainex_bridge bridge --ros-args -p socket_path:=/share/motiond.sock'
# durable: see deploy/ainex_ros2_bridge.service
sudo cp deploy/ainex_ros2_bridge.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now ainex_ros2_bridge
```

Bridge health: its log prints `ainex_bridge up, motiond socket: /share/motiond.sock`
(no "connection refused"), and `ros2 node list` shows exactly one `/ainex_bridge`.

## 3c. Sever residual ROS1 ties (for a fully ROS1-free boot)

`disable` is not enough if anything can still pull ROS1 back in:

- **Mask** the ROS1 launcher so nothing (boot, dependency, manual) can start it:
  `sudo systemctl mask start_node` (if a real unit file blocks masking, move it
  aside first, then mask).
- **Strip `start_node` from other units.** On a stock robot several services carry
  `After=start_node.service` / `Requires=start_node.service`; the `Requires=` ones
  (e.g. the web api, joystick) will *pull ROS1 back up* when restarted, and will
  *fail to start* once start_node is masked. Replace those lines with
  `After=docker.service` / `Requires=docker.service`.
- **App code that calls `rospy.init_node` blocks forever without a master.** Any
  service whose app is a ROS1 node (the vendor web api, joystick) must either be
  ported/guarded to run master-optional (detect `rosgraph.is_master_online()`
  first; fall back to the motiond socket for all data) or be disabled. A pure-ROS1
  node with no ROS2 equivalent (e.g. a `/joy`-driven joystick) should be disabled
  until ported.

## 4. Verify pure ROS2

- ROS1 gone: no `rosmaster` / `usb_cam` / `roslaunch ainex_bringup` processes.
- Camera: live JPEG frames on `:8080`.
- Vision: detections still reported by the vision service.
- Motion (unaffected — `motiond` never used ROS): telemetry (`/motion/status`)
  reports IMU + battery, and a small head nudge (`POST /motion/head?pan=..`)
  returns 200 and physically moves the head.
- Bridge: `ros2 node list` shows `/ainex_bridge` and the motion topics/services.

Best final check is a **cold power cycle**: after boot, `rosmaster`/`usb_cam`
count is 0, the camera serves frames, vision reports detections, `/ainex_bridge`
is present, and `/motion/status` + `/motion/servos` respond — all with `start_node`
`masked` and no ROS1 anywhere.

## Rollback (any time)

```bash
sudo systemctl disable --now ainex_ros2_camera
sudo systemctl enable --now start_node   # restores the ROS1 bringup at boot
# or immediately: docker exec -u <user> <ainex_container> \
#   bash -lc 'source <ros_setup>; roslaunch ainex_bringup bringup.launch'
```
