# =============================================================================
# cat_patrol.launch.py — Bring up the entire cat patrol robot stack
# =============================================================================
#
# ROS 2 CONCEPT: LAUNCH FILES
#   A launch file starts MULTIPLE nodes at once and configures them.
#   Without a launch file, you'd need to open many terminals and run
#   each node manually.  Launch files automate this:
#     ros2 launch cat_patrol_robot cat_patrol.launch.py
#
#   ROS 2 launch files are Python scripts (not XML like ROS 1).
#   The function generate_launch_description() returns a LaunchDescription
#   — a list of actions (nodes to start, parameters to set, etc.).
#
# WHAT THIS LAUNCH FILE STARTS:
#   1. Astra camera node (color + depth streams)
#   2. Yahboom chassis driver (motor control, odometry, TF)
#   3. patrol_node (C++ — the robot's brain)
#   4. mail_node (Python — sends email with photos)
#
# USAGE:
#   ros2 launch cat_patrol_robot cat_patrol.launch.py
#   ros2 launch cat_patrol_robot cat_patrol.launch.py robot:=none
#   ros2 launch cat_patrol_robot cat_patrol.launch.py robot:=r2
#
# ARGUMENTS:
#   robot              — x3 (default), r2, or none (skip chassis driver)
#   use_sim_time       — false (default); set true for Gazebo simulation
#   chassis_serial_port — /dev/ttyUSB0 (X3 only, the Rosmaster board USB)
#   uvc_product_id     — 0x050f (Astra camera USB product ID)
# =============================================================================

"""Bring up Yahboom base + Astra camera + patrol + mail.

Robot variant: ``robot`` = x3 / r2 / none (drivers skipped for ``none``).
With ``robot:=none``, ``patrol_node`` stays idle unless you publish odom /
start patrol manually — no TF spam from a phantom robot.
Camera (astra_camera) is launched automatically with depth + color enabled.
"""
import os
import yaml

# ROS 2 LAUNCH CONCEPT: get_package_share_directory
#   Returns the INSTALLED share directory for a package.  This is where
#   colcon puts config files, launch files, etc. after building.
#   Example: get_package_share_directory('cat_patrol_robot')
#     → /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/cat_patrol_robot/share/cat_patrol_robot
from ament_index_python.packages import get_package_share_directory

# ROS 2 LAUNCH CONCEPT: LaunchDescription
#   The "container" for all launch actions.  You add nodes, arguments,
#   includes, etc. to it.  ROS 2 processes them in order.
from launch import LaunchDescription

# ROS 2 LAUNCH CONCEPT: LAUNCH ARGUMENTS (DeclareLaunchArgument)
#   Launch arguments are like command-line flags.  Users set them with:
#     ros2 launch package launch_file.py arg_name:=value
#   Inside the launch file, LaunchConfiguration('arg_name') retrieves the value.
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction

# ROS 2 LAUNCH CONCEPT: INCLUDING OTHER LAUNCH FILES
#   IncludeLaunchDescription lets you nest launch files.  The camera driver
#   and chassis driver each have their OWN launch files — we just include them.
#   PythonLaunchDescriptionSource — for .py launch files
#   AnyLaunchDescriptionSource    — auto-detects .py, .xml, .yaml formats
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource

# ROS 2 LAUNCH CONCEPT: LaunchConfiguration
#   A lazy reference to a launch argument's value.  It's NOT resolved when
#   the Python code runs — it's resolved LATER when the launch system
#   actually starts nodes.  This is why you can't just write:
#     if LaunchConfiguration('robot') == 'x3':  # WON'T WORK — it's not a string yet
#   Instead, use OpaqueFunction (see below) to access the resolved value.
from launch.substitutions import LaunchConfiguration

# ROS 2 LAUNCH CONCEPT: Node action
#   Tells the launch system to start a ROS 2 node.  You specify:
#   - package: which ROS package contains the executable
#   - executable: the program to run (defined in CMakeLists.txt or setup.py)
#   - name: the ROS node name (shows in `ros2 node list`)
#   - parameters: list of YAML files and/or dicts with parameter values
#   - output: 'screen' to see stdout/stderr in the terminal
from launch_ros.actions import Node
from launch.actions import LogInfo


def _read_odom_angular_scale(params_file: str, fallback: str = '0.5') -> str:
    """Read odom_angular_scale from YAML so launch defaults are config-driven."""
    try:
        with open(params_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        params = data.get('/**', {}).get('ros__parameters', {})
        value = params.get('odom_angular_scale', fallback)
        return str(value)
    except Exception:
        return fallback


def generate_launch_description():
    # Locate installed share directories for each package we depend on
    pkg_cat = get_package_share_directory('cat_patrol_robot')
    pkg_bringup = get_package_share_directory('yahboomcar_bringup')
    pkg_astra = get_package_share_directory('astra_camera')

    # Path to our YAML parameter file
    params_file = os.path.join(pkg_cat, 'config', 'cat_patrol_params.yaml')
    odom_angular_scale_default = _read_odom_angular_scale(params_file, '0.5')

    # -----------------------------------------------------------------------
    # Declare launch arguments (these become command-line options)
    # -----------------------------------------------------------------------
    # ROS 2 LAUNCH CONCEPT: DeclareLaunchArgument
    #   Registers an argument so it can be set from the command line:
    #     ros2 launch cat_patrol_robot cat_patrol.launch.py robot:=r2
    #   'choices' restricts valid values (optional but helps catch typos).
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

    odom_angular_scale_arg = DeclareLaunchArgument(
        'odom_angular_scale',
        default_value=odom_angular_scale_default,
        description='base_node_X3 angular_scale for odometry yaw calibration',
    )

    uvc_product_id_arg = DeclareLaunchArgument(
        'uvc_product_id',
        default_value='0x050f',
        description='Astra camera UVC product ID',
    )

    # -----------------------------------------------------------------------
    # OpaqueFunction: resolve arguments → build node list
    # -----------------------------------------------------------------------
    # ROS 2 LAUNCH CONCEPT: OpaqueFunction
    #   Launch arguments (LaunchConfiguration) are "lazy" — they're not
    #   Python strings until the launch system runs.  But sometimes you need
    #   to make DECISIONS based on argument values (if robot == 'x3': ...).
    #
    #   OpaqueFunction solves this: it wraps a regular Python function that
    #   receives a "context" object.  Inside this function, you can call
    #     LaunchConfiguration('robot').perform(context)
    #   to get the ACTUAL string value, and then use normal if/else logic.
    #
    #   The function must return a list of launch actions (nodes, includes, etc.).
    def compose(context):  # noqa: D401
        """OpaqueFunction: resolved robot string → correct bringup + patrol params."""

        entities = []  # Collect all launch actions here

        # Loud banner so the operator can verify exactly which YAML and which
        # capture_completion_extra_deg / odom_angular_scale are about to be used.
        extra_deg = 'unknown'
        try:
            with open(params_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            params = data.get('/**', {}).get('ros__parameters', {})
            extra_deg = params.get('capture_completion_extra_deg', 'unset')
        except Exception as ex:
            extra_deg = f'read-error: {ex}'

        entities.append(LogInfo(msg='=========== CAT PATROL LAUNCH RESOLVED ==========='))
        entities.append(LogInfo(msg=f'  pkg_share        : {pkg_cat}'))
        entities.append(LogInfo(msg=f'  params_file      : {params_file}'))
        entities.append(LogInfo(msg=f'  odom_angular_scale (default from yaml) : {odom_angular_scale_default}'))
        entities.append(LogInfo(msg=f'  capture_completion_extra_deg (yaml)    : {extra_deg}'))
        entities.append(LogInfo(msg='=================================================='))

        # Resolve the 'robot' argument to an actual string
        robot = LaunchConfiguration('robot').perform(context)
        robot = robot.strip().lower()

        # --- Astra camera (always launched regardless of robot type) ---
        # This starts the camera driver which publishes:
        #   /camera/color/image_raw  (color frames)
        #   /camera/depth/image_raw  (depth frames)
        entities.append(
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(pkg_astra, 'launch', 'astra_pro.launch.xml')),
                launch_arguments=[
                    ('uvc_product_id', LaunchConfiguration('uvc_product_id')),
                ],
            ))

        # --- Chassis driver (conditional on robot type) ---
        # The bringup launch files start the motor driver, publish /cmd_vel
        # listener, and broadcast TF transforms (odom → base_footprint).
        if robot == 'x3':
            entities.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_bringup, 'launch', 'yahboomcar_bringup_X3_launch.py')),
                    launch_arguments=[
                        ('chassis_serial_port', LaunchConfiguration('chassis_serial_port')),
                        ('angular_scale', LaunchConfiguration('odom_angular_scale')),
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

        # robot:=none → don't start patrolling (no TF, no motors)
        start_patrol = robot != 'none'

        # --- Patrol node (C++, the robot's brain) ---
        # ROS 2 CONCEPT: PARAMETER MERGING
        #   The 'parameters' list can contain YAML files AND Python dicts.
        #   Values from later entries OVERRIDE earlier ones.  So params_file
        #   provides all defaults, and the dict overrides use_sim_time and
        #   start_patrol_on_boot specifically.
        patrol_node = Node(
            package='cat_patrol_robot',
            executable='patrol_node',
            name='patrol_node',
            output='screen',
            parameters=[
                params_file,                    # Load all defaults from YAML
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'start_patrol_on_boot': start_patrol,
                },
            ],
        )
        entities.append(patrol_node)

        # --- Mail node (Python, sends email with photos) ---
        # This node sits idle until patrol_node publishes a mail request.
        # SMTP credentials must be set as environment variables.
        entities.append(
            Node(
                package='cat_patrol_robot',
                executable='mail_node',
                name='mail_node',
                output='screen',
                parameters=[{'mail_request_topic': '/cat_patrol/mail_request'}],
            ))

        return entities

    # -----------------------------------------------------------------------
    # Assemble the LaunchDescription
    # -----------------------------------------------------------------------
    # ROS 2 LAUNCH CONCEPT: ACTION ORDER
    #   Actions in the list are processed in order:
    #     1. Declare all arguments first (so they're available to later actions)
    #     2. Then the OpaqueFunction which creates and returns nodes
    return LaunchDescription([
        robot_arg,
        use_sim_arg,
        chassis_serial_arg,
        odom_angular_scale_arg,
        uvc_product_id_arg,
        OpaqueFunction(function=compose),
    ])
