from math import radians

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument("namespace", default_value="pmocha")
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution(
            [FindPackageShare("pmocha_experiments"), "worlds", "corner_occlusion.world"]
        ),
    )
    urdf_arg = DeclareLaunchArgument(
        "urdf",
        default_value=PathJoinSubstitution(
            [FindPackageShare("pmocha_experiments"), "urdf", "pmocha_dynus_ground_robot.urdf.xacro"]
        ),
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    z_arg = DeclareLaunchArgument("z", default_value="0.05")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0")
    gui_arg = DeclareLaunchArgument("gui", default_value="true")

    def launch_setup(context, *args, **kwargs):
        namespace = LaunchConfiguration("namespace").perform(context)
        world = LaunchConfiguration("world").perform(context)
        urdf = LaunchConfiguration("urdf").perform(context)
        x = LaunchConfiguration("x").perform(context)
        y = LaunchConfiguration("y").perform(context)
        z = LaunchConfiguration("z").perform(context)
        yaw = str(radians(float(LaunchConfiguration("yaw").perform(context))))

        robot_description = ParameterValue(
            Command(["xacro ", urdf, " namespace:=", namespace]),
            value_type=str,
        )

        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
            output="screen",
        )

        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
            ),
            launch_arguments={
                "world": world,
                "gui": LaunchConfiguration("gui"),
                "verbose": "false",
            }.items(),
        )

        spawn_robot = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-topic",
                f"/{namespace}/robot_description",
                "-entity",
                namespace,
                "-x",
                x,
                "-y",
                y,
                "-z",
                z,
                "-Y",
                yaw,
            ],
            output="screen",
        )

        return [gazebo, robot_state_publisher, spawn_robot]

    return LaunchDescription(
        [
            namespace_arg,
            world_arg,
            urdf_arg,
            x_arg,
            y_arg,
            z_arg,
            yaw_arg,
            gui_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
