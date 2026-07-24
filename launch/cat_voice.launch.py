# =============================================================================
# cat_voice.launch.py — cat detection + per-cat voice playback (toggleable)
# =============================================================================
#
# Standalone launch file, run ALONGSIDE the patrol stack -- it does not touch
# the patrol brains directly (it only publishes /cat_detector_cpp/detections,
# which the brains subscribe to). Three independently-toggleable pieces:
#
#   start_detector (default true)  cat_detector_cpp -- YOLO + per-cat (white/
#                                  brown) classifier. Publishes identity on
#                                  /cat_detector_cpp/detections. Required for
#                                  both voice playback AND the cat-recognized
#                                  email in the patrol brain.
#   start_voice    (default true)  voice_node -- subscribes to that topic and
#                                  plays a random ~/cats/voices/{jente,arik}-*.mp3
#                                  line on recognition.
#   start_bt       (default true)  connect_bt_speaker.sh -- pairs/connects the
#                                  BT speaker and sets it as the PulseAudio
#                                  default sink (the sink voice_node plays to).
#
# The three are orthogonal so you can, e.g., run the detector for
# cat-recognized emails WITHOUT any audio:
#   ros2 launch cat_patrol_robot cat_voice.launch.py start_voice:=false start_bt:=false
#
# USAGE:
#   ros2 launch cat_patrol_robot cat_voice.launch.py            # all three
#   ros2 launch cat_patrol_robot cat_voice.launch.py start_bt:=false
# =============================================================================
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_cat = get_package_share_directory('cat_patrol_robot')
    pkg_detector = get_package_share_directory('cat_detector_cpp')

    bt_connect_script = os.path.join(
        get_package_prefix('cat_patrol_robot'), 'lib', 'cat_patrol_robot',
        'connect_bt_speaker.sh')

    start_bt_arg = DeclareLaunchArgument(
        'start_bt', default_value='true', choices=['true', 'false'],
        description='Connect the BT speaker and set it as the PulseAudio default sink.')
    start_detector_arg = DeclareLaunchArgument(
        'start_detector', default_value='true', choices=['true', 'false'],
        description='Start cat_detector_cpp (YOLO + white/brown classifier).')
    start_voice_arg = DeclareLaunchArgument(
        'start_voice', default_value='true', choices=['true', 'false'],
        description='Start voice_node (plays per-cat mp3 on recognition).')

    bt_connect = ExecuteProcess(
        cmd=['bash', bt_connect_script],
        name='connect_bt_speaker',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_bt')),
    )

    detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_detector, 'launch', 'cat_detector_cpp.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_detector')),
    )

    voice_node = Node(
        package='cat_patrol_robot',
        executable='voice_node',
        name='cat_voice',
        output='screen',
        parameters=[os.path.join(pkg_cat, 'config', 'cat_voice_params.yaml')],
        condition=IfCondition(LaunchConfiguration('start_voice')),
    )

    return LaunchDescription([
        start_bt_arg,
        start_detector_arg,
        start_voice_arg,
        bt_connect,
        detector,
        voice_node,
    ])
