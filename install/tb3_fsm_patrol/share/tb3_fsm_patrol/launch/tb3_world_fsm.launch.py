import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- launch arguments (bisa dioverride saat ros2 launch) ---
    model = LaunchConfiguration("model")
    master_uri = LaunchConfiguration("gazebo_master_uri")
    verbose = LaunchConfiguration("verbose")

    lin_speed = LaunchConfiguration("lin_speed")
    ang_speed = LaunchConfiguration("ang_speed")
    obstacle_front = LaunchConfiguration("obstacle_front")
    front_fov_deg = LaunchConfiguration("front_fov_deg")
    avoid_turn_s = LaunchConfiguration("avoid_turn_s")
    avoid_fwd_s = LaunchConfiguration("avoid_fwd_s")
    goal_tol = LaunchConfiguration("goal_tol")

    # --- paths turtlebot3_gazebo ---
    tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    tb3_world_launch = os.path.join(tb3_gazebo_share, "launch", "turtlebot3_world.launch.py")
    tb3_models = os.path.join(tb3_gazebo_share, "models")
    tb3_worlds = os.path.join(tb3_gazebo_share, "worlds")

    # --- gazebo classic system paths (untuk shader/resource) ---
    gz_share = "/usr/share/gazebo-11" if os.path.exists("/usr/share/gazebo-11") else "/usr/share/gazebo"
    gz_models = os.path.join(gz_share, "models")
    gz_media = os.path.join(gz_share, "media")

    # Set env (dibuat "bersih", tidak ikut-ikutan env lama yang bikin warning)
    env_actions = [
        SetEnvironmentVariable("TURTLEBOT3_MODEL", model),
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),  # biar gak download online
        SetEnvironmentVariable("GAZEBO_MASTER_URI", master_uri),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", f"{gz_models}:{tb3_models}"),
        SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", f"{gz_share}:{gz_media}:{tb3_worlds}"),
        # opsional, kalau kamu sering masalah Wayland:
        # SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_world_launch),
        launch_arguments={
            "force_system": "false",
            "verbose": verbose,
        }.items(),
    )

    fsm = Node(
        package="tb3_fsm_patrol",
        executable="fsm_patrol",
        output="screen",
        parameters=[{
            "lin_speed": lin_speed,
            "ang_speed": ang_speed,
            "obstacle_front": obstacle_front,
            "front_fov_deg": front_fov_deg,
            "avoid_turn_s": avoid_turn_s,
            "avoid_fwd_s": avoid_fwd_s,
            "goal_tol": goal_tol,
            # kalau mau waypoint custom, bisa tambahkan di sini juga nanti
        }],
    )

    # Delay sedikit supaya robot sudah spawn dulu (lebih rapi)
    fsm_delayed = TimerAction(period=2.0, actions=[fsm])

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="burger"),
        DeclareLaunchArgument("gazebo_master_uri", default_value="http://127.0.0.1:11346"),
        DeclareLaunchArgument("verbose", default_value="true"),

        # default parameter yang sudah kamu bilang "lumayan baik"
        DeclareLaunchArgument("lin_speed", default_value="0.10"),
        DeclareLaunchArgument("ang_speed", default_value="0.45"),
        DeclareLaunchArgument("obstacle_front", default_value="0.55"),
        DeclareLaunchArgument("front_fov_deg", default_value="40.0"),
        DeclareLaunchArgument("avoid_turn_s", default_value="1.2"),
        DeclareLaunchArgument("avoid_fwd_s", default_value="0.5"),
        DeclareLaunchArgument("goal_tol", default_value="0.25"),

        *env_actions,
        gazebo,
        fsm_delayed,
    ])
