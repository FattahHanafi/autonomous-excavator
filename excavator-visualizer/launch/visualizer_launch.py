import os
import os.path
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def to_urdf(xacro_path, parameters=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * parameters -- to be used when xacro file is parsed.
    """
    urdf_path = tempfile.mktemp(prefix="%s_" % os.path.basename(xacro_path))
    print("Hi2")
    # open and process file
    doc = xacro.process_file(xacro_path, mappings=parameters)
    print("Hi3")
    # open the output file
    out = xacro.open_output(urdf_path)
    print("Hi4")
    out.write(doc.toprettyxml(indent="  "))
    print("Hi5")

    return urdf_path


urdf_file_name = "electrical_excavator.urdf.xacro"
rviz_file_name = "machine_visualizer.rviz"

xacro_path = os.path.join(
    get_package_share_directory("excavator-visualizer"),
    "urdf",
    "electrical_excavator.urdf.xacro",
)
print("Hi")
urdf = to_urdf(xacro_path, {"use_nominal_extrinsics": "true", "add_plug": "true"})
rviz = os.path.join(
    get_package_share_directory("excavator-visualizer"), "config", rviz_file_name
)

with open(urdf, "r") as infp:
    robot_desc = infp.read()


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="excavator-visualizer",
                executable="joint_publisher",
            ),
            Node(
                package="excavator-visualizer",
                executable="stroke_publisher",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_desc}],
                arguments=[urdf],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz],
            ),
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
            Node(
                package="marching-cubes",
                executable="marching_cubes",
            ),
        ]
    )
