from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
            Node(
                package="bounding-box",
                executable="bounding_box",
                name="h1",
                parameters=[
                    {"title": "Human_1"},
                    {"motion": 1},
                    {"size_x": 0.5},
                    {"size_y": 0.5},
                    {"size_z": 2.0},
                ],
            ),
            Node(
                package="bounding-box",
                executable="bounding_box",
                name="h2",
                parameters=[
                    {"title": "Car_1"},
                    {"motion": 2},
                    {"size_x": 2.0},
                    {"size_y": 5.0},
                    {"size_z": 1.5},
                ],
            ),
            Node(
                package="bounding-box",
                executable="closest_distance",
                name="h3",
                parameters=[
                    {"source": "Human_1"},
                    {"target_1": "Car_1"},
                ],
            ),
        ])
