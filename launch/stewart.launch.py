from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    actuator_node = Node(
        package="stewart_platform_pkg",
        executable="actuator_node"
    )

    kinematics_node = Node(
        package="stewart_platform_pkg",
        executable="kinematics_node.py"
    )

    hmi_node = Node(
        package="stewart_platform_pkg",
        executable="hmi_node"
    )

    ld.add_action(actuator_node)
    ld.add_action(kinematics_node)
    ld.add_action(hmi_node)

    return ld