from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    robot_names = ["C3PO", "Luke Skywalker", "R2D2", "Obi Wan", "BB-8"]

    for name in robot_names:
        node = Node(
            package="my_py_pkg",
            executable="robot_news_station",
            name="robot_news_station_" + name.lower().replace(" ", "_").replace("-", "_"),
            parameters=[
                {"robot_name": name}
            ]
        )
        ld.add_action(node)

    smartphone_node = Node(
        package="my_cpp_pkg",
        executable="smartphone",
    )
    ld.add_action(smartphone_node)

    return ld