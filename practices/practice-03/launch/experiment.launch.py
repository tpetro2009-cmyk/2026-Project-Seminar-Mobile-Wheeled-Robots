import math
import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher

PKG = 'odometry_study'

START = dict(x='0', y='0', z='0', yaw='0')


def setup(context, *args, **kwargs):
    share = get_package_share_directory(PKG)
    noise = LaunchConfiguration('noise').perform(context)
    duration = float(LaunchConfiguration('duration').perform(context))
    results_dir = LaunchConfiguration('results_dir').perform(context)
    sep_mult = LaunchConfiguration('wheel_separation_multiplier').perform(context)
    name = 'noise_' + noise.replace('.', '_')

    
    with open(os.path.join(share, 'worlds', 'arena.wbt')) as f:
        world = f.read()
    world, n = re.subn(r'(encoderNoise\s+)[-+0-9.eE]+', r'\g<1>' + noise, world)
    assert n == 1, 'encoderNoise field not found in arena.wbt'
    world = world.replace('"../protos/', '"' + os.path.join(share, 'protos') + '/')
    tmp = os.path.join(tempfile.mkdtemp(), 'arena_run.wbt')
    with open(tmp, 'w') as f:
        f.write(world)

    webots = WebotsLauncher(world=tmp, mode='realtime', ros2_supervisor=False)

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': '<robot name=""><link name=""/></robot>'}])
    footprint_publisher = Node(
        package='tf2_ros', executable='static_transform_publisher', output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'])
    map_publisher = Node(
        package='tf2_ros', executable='static_transform_publisher', output='screen',
        arguments=['--x', START['x'], '--y', START['y'], '--z', START['z'],
                   '--yaw', START['yaw'], '--pitch', '0', '--roll', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'])

    supervisor = WebotsController(
        robot_name='ground_truth_supervisor',  
        parameters=[{'robot_description':
                     os.path.join(share, 'resource', 'ground_truth_supervisor.urdf'),
                     'use_sim_time': True}],
        respawn=True)

    robot_driver = WebotsController(
        robot_name='TurtleBot3Burger',
        parameters=[{'robot_description':
                     os.path.join(share, 'resource', 'turtlebot_webots.urdf'),
                     'use_sim_time': True,
                     'set_robot_state_publisher': True},
                    os.path.join(share, 'resource', 'ros2_control.yaml')],
        remappings=[('/diffdrive_controller/cmd_vel', '/cmd_vel'),
                    ('/diffdrive_controller/odom', '/odom')],
        respawn=True)

    timeout = ['--controller-manager-timeout', '50']
    diff = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['diffdrive_controller',
                   '--controller-ros-args=-p diffdrive_controller:'
                   f'wheel_separation_multiplier:={sep_mult}'] + timeout)
    jsb = Node(package='controller_manager', executable='spawner', output='screen',
               arguments=['joint_state_broadcaster'] + timeout)
    waiting = WaitForControllerConnection(target_driver=robot_driver,
                                          nodes_to_start=[diff, jsb])

    cmd = Node(package=PKG, executable='cmd_publisher',
               parameters=[{'use_sim_time': True, 'linear_x': 0.1, 'angular_z': 0.2}])
    recorder = Node(package=PKG, executable='experiment_recorder',
                    parameters=[{'use_sim_time': True, 'duration': duration,
                                 'output_dir': results_dir, 'output_name': name,
                                 'noise': float(noise)}])

    return [
        webots, robot_state_publisher, footprint_publisher, map_publisher,
        supervisor, robot_driver, waiting, cmd, recorder,
        RegisterEventHandler(OnProcessExit(target_action=recorder,
                                           on_exit=[EmitEvent(event=Shutdown())])),
        RegisterEventHandler(OnProcessExit(target_action=webots,
                                           on_exit=[EmitEvent(event=Shutdown())])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('noise', default_value='0'),
        DeclareLaunchArgument('duration', default_value='130.0',
                              description='experiment length, sim seconds'),
        DeclareLaunchArgument('results_dir', default_value='results'),
        DeclareLaunchArgument('wheel_separation_multiplier', default_value='1.11'),
        OpaqueFunction(function=setup),
    ])
