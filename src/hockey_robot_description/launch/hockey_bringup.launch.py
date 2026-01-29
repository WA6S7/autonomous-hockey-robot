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

    # Perception and control nodes (two instances each)
    detector_r1 = Node(
        package="hockey_perception",
        executable="lidar_puck_detector",
        name="lidar_puck_detector_r1",
        output="screen",
        parameters=[{
            "scan_topic": "/robot1/scan",
            "puck_pose_topic": "/robot1/puck_pose_lidar",
            "use_sim_time": True,
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
            "use_sim_time": True,
        }],
    )



    # Puck kicker node
    puck_kicker = Node(
        package="hockey_control",
        executable="puck_kicker",
        name="puck_kicker",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        # Start Gazebo and spawn robots
        IncludeLaunchDescription(PythonLaunchDescriptionSource(spawn_launch)),

        # Start bridges
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bridge_launch)),


        # Start FSM for robot1
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("hockey_control"), "launch", "fsm.launch.py"])
            ),
            launch_arguments={
                'robot_name': 'robot1',
                'scan_topic': 'scan',
                'puck_topic': 'puck_pose_lidar',
                'goal_y': '-4.5'
            }.items()
        ),

        # Start FSM for robot2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("hockey_control"), "launch", "fsm.launch.py"])
            ),
            launch_arguments={
                'robot_name': 'robot2',
                'scan_topic': 'scan',
                'puck_topic': 'puck_pose_lidar',
                'goal_y': '4.5'
            }.items()
        ),

        # Small delay helps avoid “topic not ready yet” races at startup
        TimerAction(period=2.0, actions=[
            detector_r1,
            detector_r2,
            puck_kicker,
        ]),
    ])
