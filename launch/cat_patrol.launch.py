"""Bring up Yahboom base + patrol + mail.

Robot variant: ``robot`` = x3 / r2 / none (drivers skipped for ``none``).
With ``robot:=none``, ``patrol_node`` stays idle unless you publish odom /
start patrol manually — no TF spam from a phantom robot.
Camera is not launched here — start your camera separately if needed.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_cat = get_package_share_directory('cat_patrol_robot')
    pkg_bringup = get_package_share_directory('yahboomcar_bringup')
    params_file = os.path.join(pkg_cat, 'config', 'cat_patrol_params.yaml')

    robot_arg = DeclareLaunchArgument(
        'robot', default_value='x3',
        description='Yahboom chassis: x3, r2, or none (skip driver launch)',
        choices=['x3', 'r2', 'none'],
    )

    use_sim_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    chassis_serial_arg = DeclareLaunchArgument(
        'chassis_serial_port',
        default_value='/dev/ttyUSB0',
        description='X3 only: USB device for Rosmaster (see yahboomcar_bringup_X3_launch)',
    )

    def compose(context):  # noqa: D401
        """OpaqueFunction: resolved robot string → correct bringup + patrol params."""

        entities = []

        robot = LaunchConfiguration('robot').perform(context)
        robot = robot.strip().lower()

        if robot == 'x3':
            entities.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_bringup, 'launch', 'yahboomcar_bringup_X3_launch.py')),
                    launch_arguments=[
                        ('chassis_serial_port', LaunchConfiguration('chassis_serial_port')),
                        ('use_joystick', 'false'),
                    ],
                ))
        elif robot == 'r2':
            entities.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_bringup, 'launch', 'yahboomcar_bringup_R2_launch.py')),
                ))
        elif robot != 'none':
            raise RuntimeError(
                'Invalid robot=%r — use robot:=x3, robot:=r2, or robot:=none' % (robot, ))

        start_patrol = robot != 'none'

        patrol_node = Node(
            package='cat_patrol_robot',
            executable='patrol_node',
            name='patrol_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'start_patrol_on_boot': start_patrol,
                },
            ],
        )
        entities.append(patrol_node)

        entities.append(
            Node(
                package='cat_patrol_robot',
                executable='mail_node',
                name='mail_node',
                output='screen',
                parameters=[{'mail_request_topic': '/cat_patrol/mail_request'}],
            ))

        return entities

    return LaunchDescription([
        robot_arg,
        use_sim_arg,
        chassis_serial_arg,
        OpaqueFunction(function=compose),
    ])
