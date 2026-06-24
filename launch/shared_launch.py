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
    print(f"Mobile Robot share directory: {mobile_robot_share}")
    controller_params_file = os.path.join(
        mobile_robot_share,
        'parameters',
        'controller_params.yaml'
    )

    # Update the default to the compressed topic
    # camera_topic_arg = DeclareLaunchArgument(
    #     'camera_topic',
    #     default_value='/camera/image_raw/compressed',
    #     description='Input compressed camera topic'
    # )
    # processed_topic_arg = DeclareLaunchArgument(
    #     'processed_topic',
    #     default_value='/camera_processed',
    #     description='Output processed image topic'
    # )
    # distortion_tuner_node = Node(
    #     package='robot_vision',
    #     executable='distortion_tuner',
    #     name='distortion_tuner',
    #     output='screen',
    # )
    camera_processor_node = Node(
        package='mobile_robot',
        executable='image_processor',
        name='image_processor',
        output='screen',
        # parameters=[{
        #     'camera_topic': LaunchConfiguration('camera_topic'),
        #     'processed_topic': LaunchConfiguration('processed_topic'),
        # }],
        # remappings=[
        #     ('camera/image_raw/compressed', LaunchConfiguration('camera_topic')),
        #     ('/camera_processed',           LaunchConfiguration('processed_topic')),
        # ]
    )
    # HSV_tuner_node = Node(
    #     package='mobile_robot',
    #     executable='hsv_tuner_node',
    #     name='hsv_tuner_node',
    #     output='screen',
    # )

    # controller_manager_node = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[robot_description, controller_params_file],
    #     remappings=[
    #     ('robot_description', '/robot_description'),  # explicit global topic
    # ],
    # )

    # diff_drive_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['diff_cont'],
    # )
    # joint_broad_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_broad'],
    # )

    IPM_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_vision'),
                'launch',
                'demonstrate_track_outlines.launch.py'
            )
        )
    )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen'
    # )

    # rqt_image_view_node = Node(
    #     package='rqt_image_view',
    #     executable='rqt_image_view',
    #     name='rqt_image_view',
    #     output='screen',
    #     arguments=['--ros-args', '--remap', '/image:=/debug/mask_image']
    # )

    return LaunchDescription([
        #camera_topic_arg,
        #processed_topic_arg,
        #distortion_tuner_node,
        camera_processor_node,
        #TimerAction(period=4.0, actions=[HSV_tuner_node]),
        
        IPM_launch,
        #rviz_node,
        #rqt_image_view_node,
        #controller_manager_node,
        #TimerAction(period=4.0, actions=[diff_drive_spawner]),
        # TimerAction(period=4.0, actions=[joint_broad_spawner]),
        
    ])