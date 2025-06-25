#!/usr/bin/python3

# Copyright (c) 2022, www.guyuehome.com

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory, get_package_prefix


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hobot_usb_cam',
            executable='hobot_usb_cam',
            name='hobot_usb_cam',
            parameters=[
                {"camera_calibration_file_path": "/opt/tros/lib/hobot_usb_cam/config/usb_camera_calibration.yaml"},
                {"frame_id": "default_usb_cam"},
                {"framerate": 30},
                {'image_width': 960},
                {'image_height': 544},
                {"io_method": "mmap"},
                {"pixel_format": "mjpeg"},
                {"video_device": "/dev/video8"},
                # {"zero_copy": True}
                {"zero_copy": False}
            ],
            remappings=[
            # ("/image", "/image_raw","image_model")
              ("/image", "/image_raw")
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            output='screen',
            parameters=[
                    {"channel": 1},
                    {"in_mode": "ros"},
                    {"in_format": "jpeg"},
                    {"out_mode": "ros"},
                    {"out_format": "bgr8"},
                    {"sub_topic": "/image_raw"},
                    {"pub_topic": "/image"},
                    # -------------------------------new
                    # {"pub_topic": "/image_model"},
                    #------------------------------
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_codec') + '/launch/hobot_codec_decode.launch.py'),
                                               launch_arguments={'codec_in_mode': 'ros', 'codec_out_mode': 'shared_mem',
                                                                 'codec_sub_topic': '/image_raw', 'codec_pub_topic': '/hbmem_img'}.items()),
    ])