from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable

def generate_launch_description():

    pkg_share = FindPackageShare("hockey_robot_description")
    world_pkg = FindPackageShare("hockey_world")

    world = PathJoinSubstitution(
        [world_pkg, "worlds", "hockey_arena.sdf"]
    )

    xacro = PathJoinSubstitution(
        [pkg_share, "urdf", "hockey_robot.urdf.xacro"]
    )

    xacro_exec = FindExecutable(name="xacro")

    robot1_desc = Command([
        xacro_exec, " ",
        xacro,
        " prefix:=robot1_ ns:=robot1"
    ])

    robot2_desc = Command([
        xacro_exec, " ",
        xacro,
        " prefix:=robot2_ ns:=robot2"
    ])


    return LaunchDescription([

        # Start Gazebo
        ExecuteProcess(
            cmd=["gz", "sim", "-r", world],
            output="screen"
        ),

        # ---------------- ROBOT 1 ----------------
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="robot1",
            name="robot_state_publisher_1",
            parameters=[{
                "robot_description": robot1_desc,
                "use_sim_time": True
            }],
        ),

        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", "robot1",
                "-topic", "/robot1/robot_description",
                "-x", "0", "-y", "3.5", "-z", "0.05",
                "-R", "0", "-P", "0", "-Y", "-1.5708",
            ],
        ),

        # ---------------- ROBOT 2 ----------------
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="robot2",
            name="robot_state_publisher_2",
            parameters=[{
                "robot_description": robot2_desc,
                "use_sim_time": True
            }],
        ),

        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", "robot2",
                "-topic", "/robot2/robot_description",
                "-x", "0", "-y", "-3.5", "-z", "0.05",
                "-R", "0", "-P", "0", "-Y", "1.5708"
            ],
        ),
    ])
