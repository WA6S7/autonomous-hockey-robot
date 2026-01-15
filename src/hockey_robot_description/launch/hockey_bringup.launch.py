from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_share = FindPackageShare("hockey_robot_description")

    spawn_launch = PathJoinSubstitution([pkg_share, "launch", "spawn_robots.launch.py"])
    bridge_launch = PathJoinSubstitution([pkg_share, "launch", "bridge.launch.py"])

    # Perception + control nodes (two instances each)
    detector_r1 = Node(
        package="hockey_perception",
        executable="lidar_puck_detector",
        name="lidar_puck_detector_r1",
        output="screen",
        parameters=[{
            "scan_topic": "/robot1/scan",
            "puck_pose_topic": "/robot1/puck_pose_lidar",
            "use_sim_time": use_sim_time,
        }],
    )

    detector_r2 = Node(
        package="hockey_perception",
        executable="lidar_puck_detector",
        name="lidar_puck_detector_r2",
        output="screen",
        parameters=[{
            "scan_topic": "/robot2/scan",
            "puck_pose_topic": "/robot2/puck_pose_lidar",
            "use_sim_time": use_sim_time,
        }],
    )

    chaser_r1 = Node(
        package="hockey_perception",
        executable="puck_chaser",
        name="puck_chaser_r1",
        output="screen",
        parameters=[{
            "target_topic": "/robot1/puck_pose_lidar",
            "cmd_vel_topic": "/robot1/cmd_vel",
            "use_sim_time": use_sim_time,
        }],
    )

    chaser_r2 = Node(
        package="hockey_perception",
        executable="puck_chaser",
        name="puck_chaser_r2",
        output="screen",
        parameters=[{
            "target_topic": "/robot2/puck_pose_lidar",
            "cmd_vel_topic": "/robot2/cmd_vel",
            "use_sim_time": use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        # Start Gazebo + spawn robots
        IncludeLaunchDescription(PythonLaunchDescriptionSource(spawn_launch)),

        # Start bridges
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bridge_launch)),

        # Small delay helps avoid “topic not ready yet” races at startup
        TimerAction(period=2.0, actions=[
            detector_r1,
            detector_r2,
            chaser_r1,
            chaser_r2,
        ]),
    ])
