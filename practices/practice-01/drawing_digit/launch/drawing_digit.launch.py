import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():

    turtlesim_node = Node(
    	package="turtlesim", executable="turtlesim_node", name="turtlesim"
    )

    kill_turtle1 = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/kill",
            "turtlesim/srv/Kill",
            "{name: turtle1}",
        ],
        output="screen",
    )

    spawn_turtle_first = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/spawn",
            "turtlesim/srv/Spawn",
            "{x: 5, y: 5, theta: 0, name: 'turtle2'}",
        ],
        output="screen",
    )

    spawn_turtle_second = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/spawn",
            "turtlesim/srv/Spawn",
            "{x: 7, y: 5, theta: 0, name: 'turtle3'}",
        ],
        output="screen",
    )

    drawer_first = Node(
        package="drawing_digit",
        executable="drawing_digit",
        name="drawing_digit",
        parameters=[
        	{'turtle_name': 'turtle2'},
        	{'digit': 1}
        ],
        output="screen",
    )
    
    drawer_second = Node(
        package="drawing_digit",
        executable="drawing_digit",
        name="drawing_digit",
        parameters=[
        	{'turtle_name': 'turtle3'},
        	{'digit': 1}
        ],
        output="screen",
    )

 


   
    on_turtlesim_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=turtlesim_node,
            on_start=[kill_turtle1]
        )
    )

    on_kill_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=kill_turtle1,
            on_exit=[spawn_turtle_first]
        )
    )
    test_action = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_turtle_first,
            on_exit=[drawer_first]
        )
    )

    
    on_spawn_first_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_turtle_first,
            on_exit=[spawn_turtle_second]
        )
    )

    
    on_spawn_second_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_turtle_second,
            on_exit=[drawer_first, drawer_second]
        )
    )

    return LaunchDescription([
        turtlesim_node,
        on_turtlesim_start,
        on_kill_done,
        #test_action,
        on_spawn_first_done,
        on_spawn_second_done
    ])
