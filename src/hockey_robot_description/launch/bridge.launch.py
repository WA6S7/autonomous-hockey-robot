from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # ==========================
        # ROBOT 1 BRIDGES
        # ==========================
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/robot1/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
            parameters=[{"use_sim_time": True}],
            output="screen"
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/robot1/imu@sensor_msgs/msg/Imu@gz.msgs.IMU"],
            parameters=[{"use_sim_time": True}],
            output="screen"
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/robot1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"],
            parameters=[{"use_sim_time": True}],
            output="screen"
        ),

        # ==========================
        # ROBOT 2 BRIDGES
        # ==========================
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/robot2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
            parameters=[{"use_sim_time": True}],
            output="screen"
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/robot2/imu@sensor_msgs/msg/Imu@gz.msgs.IMU"],
            parameters=[{"use_sim_time": True}],
            output="screen"
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/robot2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"],
            parameters=[{"use_sim_time": True}],
            output="screen"
        ),

        # ==========================
        # PUCK DEBUG BRIDGE
        # ==========================
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/puck/pose@geometry_msgs/msg/Pose@gz.msgs.Pose"],
            parameters=[{"use_sim_time": True}],
            output="screen"
        ),
    ])
