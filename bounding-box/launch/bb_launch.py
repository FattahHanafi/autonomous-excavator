from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
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
                executable="bounding_box",
                name="h3",
                parameters=[
                    {"title": "Excavator"},
                    {"motion": 0},
                    {"frame_id": "boom"},
                    {"size_x": 3.0},
                    {"size_y": 2.0},
                    {"size_z": 2.0},
                ],
            ),
            Node(
                package="bounding-box",
                executable="closest_distance",
                name="h4",
                parameters=[
                    {"source": "Excavator"},
                    {"target_1": "Human_1"},
                ],
            ),
            Node(
                package="bounding-box",
                executable="closest_distance",
                name="h5",
                parameters=[
                    {"source": "Excavator"},
                    {"target_1": "Car_1"},
                ],
            ),
        ]
    )
