from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    robot_description = ParameterValue(
        Command([
            "cat ",
            PathJoinSubstitution([
                FindPackageShare("humanoid_description"),
                "urdf",
                "humanoid.urdf"
            ])
        ]),
        value_type=str
    )

    gazebo = ExecuteProcess(
        cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so"],
        output="screen"
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen"
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "humanoid",
            "-topic", "robot_description",
            "-z", "0.5"
        ],
        output="screen"
    )

    return LaunchDescription([gazebo, rsp, spawn])
