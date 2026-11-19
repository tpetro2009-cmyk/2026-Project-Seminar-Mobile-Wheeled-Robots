
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


PACKAGE = 'webots_amcl_localization'


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)

    world = os.path.join(share, 'worlds', 'home.wbt')
    robot_urdf = os.path.join(share, 'urdf', 'turtlebot_webots.urdf')
    supervisor_urdf = os.path.join(
        share, 'urdf', 'ground_truth_supervisor.urdf'
    )
    control_config = os.path.join(
        share, 'config', 'ros2_control.yaml'
    )
    amcl_config = os.path.join(
        share, 'config', 'amcl.yaml'
    )
    map_file = os.path.join(
        share, 'maps', 'home.yaml'
    )

    min_particles = LaunchConfiguration('min_particles')
    max_particles = LaunchConfiguration('max_particles')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')
    # Webots
    webots = WebotsLauncher(
        world=world,
        mode='realtime',
        ros2_supervisor=False,
    )

    # Supervisor: публикует Ground Truth и /clock
    supervisor = WebotsController(
        robot_name='ground_truth_supervisor',
        parameters=[{
            'robot_description': supervisor_urdf,
            'use_sim_time': True,
        }],
        respawn=True,
    )

    # TurtleBot3
    with open(robot_urdf, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    robot_driver = WebotsController(
        robot_name='TurtleBot3Burger',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True,
                'set_robot_state_publisher': False
            },
            control_config
        ],
        remappings=[
            ('/diffdrive_controller/cmd_vel', '/cmd_vel'),
            ('/diffdrive_controller/odom', '/odom')
        ],
        respawn=True
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ]
    )
    # Контроллеры колёс
    diff_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diffdrive_controller',
            '--controller-manager-timeout', '50',
        ],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout', '50',
        ],
        output='screen',
    )

    wait_for_controllers = WaitForControllerConnection(
        target_driver=robot_driver,
        nodes_to_start=[
            diff_drive,
            joint_state_broadcaster,
        ],
    )

    # Статическое преобразование base_link -> LDS-01
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '-0.03',
            '--y', '0',
            '--z', '0.153',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'LDS-01',
        ],
        output='screen',
    )

    # Карта
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_file,
        }],
    )

    # AMCL
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
            {
                'use_sim_time': True,
                'min_particles': min_particles,
                'max_particles': max_particles,
                'set_initial_pose': True,
                'initial_pose.x': initial_x,
                'initial_pose.y': initial_y,
                'initial_pose.yaw': initial_yaw,
            },
        ],
    )
   # Сбор данных для эксперимента
    experiment_recorder = Node(
        package=PACKAGE,
        executable='experiment_recorder',
        name='experiment_recorder',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'output': '/tmp/experiment.csv',
            'position_threshold': 0.15,
            'yaw_threshold': 0.15,
            'confirmation_duration': 3.0,
        }],
    )


    # Управление жизненным циклом Nav2
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
            ],
        }],
    )

    trajectory_publisher = Node(
        package=PACKAGE,
        executable='trajectory_publisher',
        name='trajectory_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'linear_speed': 0.12,
            'angular_speed': 0.08,
            'radius': 1.5,
            'center_x': 0.0,
            'center_y': 0.5,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'min_particles',
            default_value='100',
            description='Minimum number of AMCL particles',
        ),
        DeclareLaunchArgument(
            'max_particles',
            default_value='500',
            description='Maximum number of AMCL particles',
        ),
        DeclareLaunchArgument(
            'initial_x',
            default_value='1.5',
            description='Initial robot X in map frame',
        ),
        DeclareLaunchArgument(
            'initial_y',
            default_value='0.5',
            description='Initial robot Y in map frame',
        ),
        DeclareLaunchArgument(
            'initial_yaw',
            default_value='1.5708',
            description='Initial robot yaw in radians',
        ),

        webots,
        supervisor,
        robot_driver,
        robot_state_publisher,
        wait_for_controllers,
        lidar_tf,
        map_server,
        amcl,
        lifecycle_manager,
        experiment_recorder,
        trajectory_publisher,

        RegisterEventHandler(
            OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),
    ])
    