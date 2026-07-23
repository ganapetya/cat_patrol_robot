# =============================================================================
# cat_voice.launch.py — cat identification + per-cat voice playback
# =============================================================================
#
# Standalone launch file, run ALONGSIDE cat_patrol.launch.py during a patrol
# round -- it does not touch patrol_node or cat_patrol.launch.py at all.
#
# Starts:
#   1. connect_bt_speaker.sh   -- pairs/connects the BT speaker and sets it
#                                 as the PulseAudio default sink.
#   2. cat_detector_cpp        -- YOLO + per-cat classifier (not started by
#                                 cat_patrol.launch.py today), publishes
#                                 identity on /cat_detector_cpp/detections.
#   3. voice_node              -- subscribes to that topic and plays a
#                                 random ~/cats/voices/{jente,arik}-*.mp3
#                                 line on recognition, replacing the
#                                 detector's onboard buzzer beep.
#
# USAGE:
#   ros2 launch cat_patrol_robot cat_voice.launch.py
# =============================================================================
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_cat = get_package_share_directory('cat_patrol_robot')
    pkg_detector = get_package_share_directory('cat_detector_cpp')

    bt_connect_script = os.path.join(
        get_package_prefix('cat_patrol_robot'), 'lib', 'cat_patrol_robot',
        'connect_bt_speaker.sh')

    bt_connect = ExecuteProcess(
        cmd=['bash', bt_connect_script],
        name='connect_bt_speaker',
        output='screen',
    )

    detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_detector, 'launch', 'cat_detector_cpp.launch.py')),
    )

    voice_node = Node(
        package='cat_patrol_robot',
        executable='voice_node',
        name='cat_voice',
        output='screen',
        parameters=[os.path.join(pkg_cat, 'config', 'cat_voice_params.yaml')],
    )

    return LaunchDescription([
        bt_connect,
        detector,
        voice_node,
    ])
