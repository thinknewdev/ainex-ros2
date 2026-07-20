"""AiNex head-camera bringup for ROS2 Humble.

Ports the ROS1 stack from ainex_peripherals/launch/usb_cam_with_calib.launch
plus the web_video_server node from ainex_bringup/launch/bringup.launch:

  ROS1                                   ROS2 (this file)
  ----                                   ----------------
  usb_cam usb_cam_node                   v4l2_camera v4l2_camera_node
    /dev/usb_cam 640x480 yuyv              /dev/video0 640x480 YUYV
    -> /camera/image_raw                   -> /camera/image_raw (rgb8)
    -> /camera/camera_info                 -> /camera/camera_info
  image_proc/rectify nodelet             image_proc rectify_node
    -> /camera/image_rect_color            -> /camera/image_rect_color
  web_video_server on :8080              web_video_server on :8080
    /snapshot?topic=...&quality=...        same endpoint, unchanged

v4l2 control notes
------------------
The vision service sets controls directly with v4l2-ctl (manual exposure 240,
gamma 120, brightness 15, white balance 4600, auto_exposure=1).  The Humble
v4l2_camera node (0.6.2) exposes every v4l2 control as a ROS parameter whose
default is the DRIVER default, and it writes those values to the device at
startup.  Left alone, that would clobber the vision service's settings.  The
parameter overrides below pre-load the vision service's values so the node
applies the same numbers at startup instead of driver defaults.

Control parameter names come from the kernel's control names (lowercased,
spaces -> underscores, commas removed) and were renamed between kernel
generations, so both spellings are provided; the spelling that does not exist
on the device is silently ignored (never declared, so the override is unused).
Verify actual names on the robot with `v4l2-ctl -d /dev/video0 -l`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_name = LaunchConfiguration('camera_name')
    video_device = LaunchConfiguration('video_device')
    camera_info_url = LaunchConfiguration('camera_info_url')
    port = LaunchConfiguration('port')

    # Values the vision service also applies via v4l2-ctl -- keep in sync.
    v4l2_control_overrides = {
        'brightness': 15,
        'gamma': 120,
        # Newer kernel control names (>= 5.9-ish UVC naming).
        'auto_exposure': 1,                    # 1 = manual mode on UVC
        'exposure_time_absolute': 240,
        'white_balance_automatic': False,
        # Older kernel spellings of the same controls.
        'exposure_auto': 1,
        'exposure_absolute': 240,
        'white_balance_temperature_auto': False,
        'white_balance_temperature': 4600,
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name', default_value='camera',
            description='Namespace / frame id, matches ROS1 camera_name'),
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='V4L2 device (ROS1 used udev symlink /dev/usb_cam)'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='file:///home/ubuntu/.ros/camera_info/head_camera.yaml',
            description='Calibration yaml, same file the ROS1 usb_cam used'),
        DeclareLaunchArgument(
            'port', default_value='8080',
            description='web_video_server HTTP port'),

        # 1. Camera driver: /camera/image_raw + /camera/camera_info
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            namespace=camera_name,
            output='screen',
            respawn=True,
            respawn_delay=5.0,
            parameters=[{
                'video_device': video_device,
                'image_size': [640, 480],
                'pixel_format': 'YUYV',
                'output_encoding': 'rgb8',
                'camera_frame_id': camera_name,
                'camera_info_url': camera_info_url,
                **v4l2_control_overrides,
            }],
        ),

        # 2. Rectification: /camera/image_raw -> /camera/image_rect_color
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_color',
            namespace=camera_name,
            output='screen',
            respawn=True,
            respawn_delay=5.0,
            parameters=[{'queue_size': 10}],
            remappings=[
                ('image', 'image_raw'),
                ('image_rect', 'image_rect_color'),
            ],
        ),

        # 3. HTTP video server; keeps the legacy snapshot URL working:
        #    http://127.0.0.1:8080/snapshot?topic=/camera/image_rect_color&quality=90
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                # web_video_server declares port as int; coerce the launch arg.
                'port': ParameterValue(port, value_type=int),
                'address': '0.0.0.0',
                'default_stream_type': 'mjpeg',
            }],
        ),
    ])
