# =============================================================================
# patrol_system.launch.py — THE whole cat-patrol stack in one launch file
# =============================================================================
#
# Brings up every piece needed for an autonomous Nav2 waypoint patrol, in the
# right order, so you never have to run N launch files by hand again:
#
#   1. Sensors + bringup  (cat_patrol.launch.py, start_patrol_node:=false)
#        Astra camera + Yahboom base/EKF/URDF + RPLidar + mail_node.
#        The odom patrol_node is intentionally OFF — the Nav2 patrol_manager is
#        the brain here, and two brains must not both drive /cmd_vel.
#   2. Localization       (localization.launch.py)
#        map_server + AMCL. AMCL seeds itself at HOME (set_initial_pose in
#        amcl_params.yaml), so NO RViz "2D Pose Estimate" is needed. Attach RViz
#        anytime (use_rviz:=true) to override the pose or watch the run.
#   3. Navigation         (navigation.launch.py)  — the Nav2 stack.
#   4. Patrol manager     (patrol_manager.launch.py) — waypoint cycle brain.
#   5. Detector + voice   (cat_voice.launch.py) — cat_detector_cpp (use_detector)
#                          and voice_node + BT speaker (use_voice), toggled
#                          independently. The detector feeds BOTH voice playback
#                          and the brain's cat-recognized email, so you can run
#                          it silently (use_voice:=false, use_detector:=true).
#
# Steps 2-5 are started on short delays so the base driver has published
# odom/TF and the camera is streaming before localization / detection begin.
# (patrol_manager also polls for the Nav2 action server, so timing is soft.)
#
# USAGE (normally via the `patrol` CLI, but works standalone too):
#   ros2 launch cat_patrol_robot patrol_system.launch.py
#   ros2 launch cat_patrol_robot patrol_system.launch.py use_rviz:=true
#   ros2 launch cat_patrol_robot patrol_system.launch.py use_voice:=false
#   ros2 launch cat_patrol_robot patrol_system.launch.py use_voice:=false use_detector:=true
#   ros2 launch cat_patrol_robot patrol_system.launch.py use_detector:=false
# =============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(pkg, rel_launch, launch_arguments=None):
    path = os.path.join(get_package_share_directory(pkg), 'launch', rel_launch)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=launch_arguments or [],
    )


def compose(context):
    pkg_cat = get_package_share_directory('cat_patrol_robot')
    pkg_pm = get_package_share_directory('patrol_manager')

    robot = LaunchConfiguration('robot').perform(context)
    start_lidar = LaunchConfiguration('start_lidar').perform(context)
    use_detector = LaunchConfiguration('use_detector').perform(context).strip().lower() == 'true'
    use_voice = LaunchConfiguration('use_voice').perform(context).strip().lower() == 'true'
    use_rviz = LaunchConfiguration('use_rviz').perform(context).strip().lower() == 'true'

    entities = [
        LogInfo(msg='=========== PATROL SYSTEM (Nav2 brain) ==========='),
        LogInfo(msg=f'  robot={robot} start_lidar={start_lidar} '
                    f'use_detector={use_detector} use_voice={use_voice} use_rviz={use_rviz}'),
        LogInfo(msg='=================================================='),
    ]

    # 1) Sensors + bringup + mail, WITHOUT the odom patrol_node.
    entities.append(_include(
        'cat_patrol_robot', 'cat_patrol.launch.py',
        launch_arguments=[
            ('robot', robot),
            ('start_lidar', start_lidar),
            ('start_patrol_node', 'false'),
            ('start_mail', 'true'),
            # Stable MCU alias (avoids the zero-IMU/free-fall failure of the
            # ambiguous by-id; matches myscripts2/t1.sh). bringup gets
            # use_joystick:=false already, so nothing fights Nav2 for /cmd_vel.
            ('chassis_serial_port', '/dev/myserial'),
        ],
    ))

    # 2 + 3) Localization and Nav2, once odom/TF should be up (~6 s after base).
    # params_file MUST be passed explicitly: localization/navigation/patrol_manager
    # all declare a launch arg literally named "params_file", so when included
    # together the FIRST default wins for ALL of them (AMCL's) unless overridden.
    # That collision previously fed nav2 + patrol_manager the AMCL yaml → Nav2
    # lifecycle manager aborted (-6) and patrol_manager loaded 0 waypoints.
    entities.append(TimerAction(period=6.0, actions=[
        LogInfo(msg='[patrol_system] Starting localization (AMCL@home) + Nav2'),
        _include('cat_patrol_robot', 'localization.launch.py', launch_arguments=[
            ('params_file', os.path.join(pkg_cat, 'config', 'amcl_params.yaml')),
        ]),
        _include('cat_patrol_robot', 'navigation.launch.py', launch_arguments=[
            ('params_file', os.path.join(pkg_cat, 'config', 'nav2_params.yaml')),
        ]),
    ]))

    # 4 + 5) Brain + detector/voice, once camera + Nav2 have had time to come up.
    # detector and voice/BT are independent: the detector feeds both voice
    # playback AND the patrol brain's cat-recognized email, so you can keep it
    # on while silencing audio (use_voice:=false).
    later = [
        LogInfo(msg='[patrol_system] Starting patrol_manager (waypoint brain)'),
        _include('patrol_manager', 'patrol_manager.launch.py', launch_arguments=[
            ('params_file', os.path.join(pkg_pm, 'config', 'patrol_manager_params.yaml')),
        ]),
        _include('cat_patrol_robot', 'cat_voice.launch.py', launch_arguments=[
            ('start_detector', 'true' if use_detector else 'false'),
            ('start_voice', 'true' if use_voice else 'false'),
            ('start_bt', 'true' if use_voice else 'false'),
        ]),
    ]
    if use_voice and not use_detector:
        later.append(LogInfo(
            msg='[patrol_system] WARNING: use_voice=true but use_detector=false — '
                'voice_node will receive no detections and stay silent.'))
    if not use_detector:
        later.append(LogInfo(
            msg='[patrol_system] use_detector=false — cat-recognized email disabled '
                '(no /cat_detector_cpp/detections). Waypoint/loop emails still send.'))
    entities.append(TimerAction(period=12.0, actions=later))

    # Optional RViz (pose override / live monitoring).
    if use_rviz:
        rviz_cfg = os.path.join(pkg_cat, 'config', 'b.rviz')
        entities.append(TimerAction(period=8.0, actions=[Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=(['-d', rviz_cfg] if os.path.isfile(rviz_cfg) else []),
        )]))

    return entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='x3', choices=['x3', 'r2', 'none']),
        DeclareLaunchArgument('start_lidar', default_value='auto',
                              choices=['true', 'false', 'auto']),
        DeclareLaunchArgument('use_detector', default_value='true', choices=['true', 'false'],
                              description='Start cat_detector_cpp (feeds cat-recognized email + voice).'),
        DeclareLaunchArgument('use_voice', default_value='true', choices=['true', 'false'],
                              description='Start voice_node + BT speaker (audio output).'),
        DeclareLaunchArgument('use_rviz', default_value='false', choices=['true', 'false'],
                              description='Also start RViz (for pose override / monitoring).'),
        OpaqueFunction(function=compose),
    ])
