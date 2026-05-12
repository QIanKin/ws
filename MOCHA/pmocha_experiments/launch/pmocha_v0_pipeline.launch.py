from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("pmocha_experiments")

    gui_arg = DeclareLaunchArgument("gui", default_value="false")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")
    grid_frame_arg = DeclareLaunchArgument("grid_frame", default_value="odom")
    grid_resolution_arg = DeclareLaunchArgument(
        "grid_resolution", default_value="0.1"
    )
    grid_width_arg = DeclareLaunchArgument("grid_width_m", default_value="24.0")
    grid_height_arg = DeclareLaunchArgument("grid_height_m", default_value="24.0")
    grid_ox_arg = DeclareLaunchArgument("grid_origin_x", default_value="-12.0")
    grid_oy_arg = DeclareLaunchArgument("grid_origin_y", default_value="-12.0")
    z_min_arg = DeclareLaunchArgument("z_min", default_value="0.15")
    z_max_arg = DeclareLaunchArgument("z_max", default_value="1.50")
    max_range_arg = DeclareLaunchArgument("max_range", default_value="6.0")
    publish_period_arg = DeclareLaunchArgument(
        "publish_period", default_value="0.5"
    )
    ray_carving_arg = DeclareLaunchArgument(
        "enable_ray_carving", default_value="true"
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "pmocha_sensor_sim.launch.py"])
        ),
        launch_arguments={"gui": LaunchConfiguration("gui")}.items(),
    )

    perception = Node(
        package="pmocha_experiments",
        executable="perception_grid_node",
        name="perception_grid_node",
        output="screen",
        parameters=[
            {
                "grid_frame": LaunchConfiguration("grid_frame"),
                "sensor_topic": "/pmocha/d435/depth/color/points",
                "map_topic": "/pmocha/perception/occupancy_grid",
                "grid_resolution": LaunchConfiguration("grid_resolution"),
                "grid_width_m": LaunchConfiguration("grid_width_m"),
                "grid_height_m": LaunchConfiguration("grid_height_m"),
                "grid_origin_x": LaunchConfiguration("grid_origin_x"),
                "grid_origin_y": LaunchConfiguration("grid_origin_y"),
                "z_min": LaunchConfiguration("z_min"),
                "z_max": LaunchConfiguration("z_max"),
                "max_range": LaunchConfiguration("max_range"),
                "publish_period": LaunchConfiguration("publish_period"),
                "enable_ray_carving": LaunchConfiguration("enable_ray_carving"),
                "use_sim_time": True,
            }
        ],
    )

    planner = Node(
        package="pmocha_experiments",
        executable="planner_node",
        name="planner_node",
        output="screen",
        parameters=[
            {
                "map_topic": "/pmocha/perception/occupancy_grid",
                "odom_topic": "/pmocha/odom",
                "goal_topic": "/goal_pose",
                "planner_frame": LaunchConfiguration("grid_frame"),
                "astar_allow_unknown": True,
                "astar_allow_diagonal": True,
                "corridor_max_half_width": 0.6,
                "corridor_min_half_width": 0.15,
                "corridor_time_per_segment": 0.5,
                "corridor_unknown_is_obstacle": True,
                "esdf_clamp": 3.0,
                "esdf_z": 0.05,
                "use_sim_time": True,
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="pmocha_v0_rviz",
        arguments=[
            "-d",
            PathJoinSubstitution([pkg, "rviz", "pmocha_v0.rviz"]),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        grid_frame_arg,
        grid_resolution_arg,
        grid_width_arg,
        grid_height_arg,
        grid_ox_arg,
        grid_oy_arg,
        z_min_arg,
        z_max_arg,
        max_range_arg,
        publish_period_arg,
        ray_carving_arg,
        sim,
        perception,
        planner,
        rviz,
    ])
