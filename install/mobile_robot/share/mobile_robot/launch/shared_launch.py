import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch.actions import TimerAction  # add this import

from launch_ros.actions import Node

# launch/camera_processor.launch.py
##########robot dimensions###########
pi_const = 3.14159265
a = 0.25
b = 0.2
c = 0.2


r = 0.05
d = 0.02

s1 = b/2+r
s2 = 2*r
s3 = 2*r
s4 = c/2+d/2

separation = 2 * s4
######################################


def generate_launch_description():
    # Get package share directory
    mobile_robot_share = get_package_share_directory('mobile_robot')
    controller_params_file = os.path.join(
        mobile_robot_share,
        'parameters',
        'controller_params.yaml'
    )

    # Declare overridable arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera_raw',
        description='Input raw camera topic'
    )

    processed_topic_arg = DeclareLaunchArgument(
        'processed_topic',
        default_value='/camera_processed',
        description='Output processed image topic'
    )

    camera_processor_node = Node(
        package='mobile_robot',
        executable='image_processor',
        name='image_processor',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'processed_topic': LaunchConfiguration('processed_topic'),
        }],
        remappings=[
            ('/camera_raw',       LaunchConfiguration('camera_topic')),
            ('/camera_processed', LaunchConfiguration('processed_topic')),
        ]
    )
    # controller_manager_node = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[robot_description, controller_params_file],
    #     remappings=[
    #     ('robot_description', '/robot_description'),  # explicit global topic
    # ],
    # )

    twist_controller_node = Node(
        package='mobile_robot',
        executable='twist_controller',
        name='twist_controller',
        remappings=[
        ('/cmd_vel', '/diff_cont/cmd_vel'),
        ],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
    )
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
    )

    return LaunchDescription([
        camera_topic_arg,
        processed_topic_arg,
        camera_processor_node,
        #controller_manager_node,
        #twist_controller_node,
        TimerAction(period=6.0, actions=[diff_drive_spawner]),
        TimerAction(period=6.0, actions=[joint_broad_spawner]),
    ])