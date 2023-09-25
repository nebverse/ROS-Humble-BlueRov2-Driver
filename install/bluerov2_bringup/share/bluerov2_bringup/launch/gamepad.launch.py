#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    controller_node = Node(
        package="bluerov2_controller",
        executable="controller",
    )

    input_node = Node(
        package="bluerov2_controller",
        executable="input_controller",
    )

    depth_node = Node(
        package="bluerov2_controller",
        executable="depth_controller",
    )

    pitch_node = Node(
        package="bluerov2_controller",
        executable="pitch_controller",
    )

    roll_node = Node(
        package="bluerov2_controller",
        executable="roll_controller",
    )

    ld.add_action(controller_node)
    ld.add_action(input_node)
    ld.add_action(depth_node)
    ld.add_action(pitch_node)
    ld.add_action(roll_node)

    return ld