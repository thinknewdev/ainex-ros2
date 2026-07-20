# ainex_camera — ROS2 Humble camera bringup for the AiNex head camera

Port of the ROS1 camera stack:

| ROS1 (ainex_peripherals / ainex_bringup)          | ROS2 (this package)                        |
|---------------------------------------------------|--------------------------------------------|
| `usb_cam` node, `/dev/usb_cam`, 640x480 `yuyv`    | `v4l2_camera_node`, `/dev/video0`, 640x480 `YUYV` |
| `/camera/image_raw` + `/camera/camera_info`       | same topics                                |
| `image_proc/rectify` nodelet → `/camera/image_rect_color` | `image_proc rectify_node` → same topic |
| `web_video_server` on `:8080`                     | `web_video_server` on `:8080`, same URLs   |

ROS1 sources this was ported from:
- `ros1_src/ainex_peripherals/launch/usb_cam.launch` (device, 640x480 yuyv, calib URL)
- `ros1_src/ainex_peripherals/launch/image_calib.launch` (rectify nodelet, topic names)
- `ros1_src/ainex_bringup/launch/bringup.launch` (web_video_server, respawn behavior)

## apt packages (robot's Humble arm64 container)

```bash
sudo apt install \
  ros-humble-v4l2-camera \
  ros-humble-image-proc \
  ros-humble-web-video-server \
  ros-humble-image-transport-plugins   # optional but recommended (compressed transports)
```

`v4l2-ctl` (package `v4l-utils`) should already be present — the vision service uses it.

## Build & launch

```bash
cd <ros2_ws>
colcon build --packages-select ainex_camera --symlink-install
source install/setup.bash
ros2 launch ainex_camera camera.launch.py
# args: video_device:=/dev/video0 camera_name:=camera port:=8080 \
#       camera_info_url:=file:///home/ubuntu/.ros/camera_info/head_camera.yaml
```

## Snapshot URL compatibility — VERDICT: compatible, no shim needed

The robot's vision service depends on:

```
http://127.0.0.1:8080/snapshot?topic=/camera/image_rect_color&quality=90
```

Verified against the Humble-released web_video_server (**3.1.0**, what
`ros-humble-web-video-server` installs; checked `src/web_video_server.cpp` and
`src/streamers/jpeg_streamers.cpp` at tag 3.1.0):

- The server still registers the `/snapshot` path (plus `/`, `/stream`,
  `/stream_viewer`, `/shutdown`) — identical to ROS1.
- The snapshot handler still takes `topic=` and the JPEG snapshot streamer
  still reads `quality` from the query string
  (`request.get_query_param_value_or_default<int>("quality", 95)`).
- Default port is still 8080 (`declare_parameter("port", 8080)`).

So the old URL works **verbatim**; no `snapshot_shim.py` was written because
nothing needs shimming. MJPEG streaming URLs
(`/stream?topic=...&type=mjpeg...`) are likewise unchanged.

## Calibration file

No vendor calibration yaml exists in the source tree. The ROS1 launch pointed
at a file that lives **on the robot only**:

```
/home/ubuntu/.ros/camera_info/head_camera.yaml
```

The launch defaults `camera_info_url` to that same path
(`file:///home/ubuntu/.ros/camera_info/head_camera.yaml`). Keep/copy that file
into the Humble container at the same path (or pass a different
`camera_info_url:=`). If the yaml's `camera_name` doesn't match the node's
camera name you'll get a harmless warning; calibration still loads. Without
the file, `/camera/camera_info` is published with zeroed calibration and
`rectify_node` will not produce a valid `/camera/image_rect_color`.

## v4l2 controls vs. the vision service (important)

The vision service sets controls directly with `v4l2-ctl`:
manual exposure 240, gamma 120, brightness 15, white balance 4600,
`auto_exposure=1`.

Humble's `v4l2_camera` (0.6.2) exposes every v4l2 control as a ROS parameter
**defaulted to the driver default and written to the device at startup** —
i.e. left unconfigured it would reset the vision service's settings every time
the camera node (re)starts. `camera.launch.py` therefore pre-loads the vision
service's exact values as parameter overrides (both old and new kernel control
spellings; the nonexistent spelling is ignored), so the node applies the same
numbers instead of driver defaults.

Belt and suspenders: keep the vision service applying its `v4l2-ctl` calls
after camera startup as it does today — ordering of auto-exposure vs. absolute
exposure application inside v4l2_camera is not guaranteed on older kernel
control names, and a late `v4l2-ctl` pass fixes any control the node's startup
pass couldn't set. Check real control names with
`v4l2-ctl -d /dev/video0 -l` and adjust `v4l2_control_overrides` in
`launch/camera.launch.py` if they differ.

## Hardware-verify checklist (run on the robot)

1. `v4l2-ctl -d /dev/video0 --list-formats-ext` — confirm 640x480 YUYV is
   offered (it's the camera's only mode).
2. `v4l2-ctl -d /dev/video0 -l` — note exact control names; reconcile with
   `v4l2_control_overrides` in the launch file.
3. Confirm `/dev/video0` is the right node inside the container (ROS1 used a
   udev symlink `/dev/usb_cam`; if present, `video_device:=/dev/usb_cam`
   works too) and that the container has the device mapped.
4. Confirm `/home/ubuntu/.ros/camera_info/head_camera.yaml` exists in the
   Humble container; copy it from the ROS1 container/backup if not.
5. `ros2 launch ainex_camera camera.launch.py`, then:
   - `ros2 topic hz /camera/image_raw` — expect ~30 fps, 640x480.
   - `ros2 topic echo /camera/camera_info --once` — nonzero K/D matrices.
   - `ros2 topic hz /camera/image_rect_color` — rectified stream alive.
6. `curl -o /tmp/snap.jpg "http://127.0.0.1:8080/snapshot?topic=/camera/image_rect_color&quality=90"`
   — returns a valid JPEG (the vision service's exact URL).
7. Browser check: `http://<robot>:8080/` lists topics;
   `/stream?topic=/camera/image_rect_color` shows live MJPEG.
8. While the camera node is running, run the vision service's `v4l2-ctl`
   settings pass, then `v4l2-ctl -d /dev/video0 -C exposure_time_absolute,gamma,brightness`
   (or old names) — confirm values stick at 240/120/15 and the image does not
   pulse (i.e. nothing is fighting the controls).
9. Restart the camera node (`respawn` will also do this on crash) and re-check
   step 8 — confirms the launch-file control overrides survive node restarts.
10. CPU check under load (`top`): v4l2_camera + rectify + one MJPEG client;
    confirm acceptable on the robot's CPU.
