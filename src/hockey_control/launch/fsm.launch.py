from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='robot1')
    scan_topic_arg = DeclareLaunchArgument('scan_topic', default_value='scan')
    puck_topic_arg = DeclareLaunchArgument('puck_topic', default_value='puck_pose_lidar')
    goal_y_arg = DeclareLaunchArgument('goal_y', default_value='0.0')
    
    return LaunchDescription([
        robot_name_arg,
        scan_topic_arg,
        puck_topic_arg,
        goal_y_arg,
        Node(
            package='hockey_control',
            executable='simple_fsm',
            name='simple_fsm',
            namespace=LaunchConfiguration('robot_name'),
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name'),
                'puck_topic': LaunchConfiguration('puck_topic'),
                'goal_y': LaunchConfiguration('goal_y')
            }],
            remappings=[
                ('scan', LaunchConfiguration('scan_topic')),
                ('cmd_vel', 'cmd_vel')
            ],
            output='screen'
        )
    ])
