#############ros2 and gazebo launch file for differential drive robot###################

import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro


def generate_launch_description():

    #name of the package
    namePackage = 'mobile_robot'


    # Include shared launch file
    sharedLaunchPackage = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(namePackage), 'launch', 'shared_launch.py'))
    sharedLaunch = IncludeLaunchDescription(sharedLaunchPackage)

    #create an empty launch description object
    launchDescriptionObject = LaunchDescription()

    # twist_controller_node = Node(
    #     package='mobile_robot',
    #     executable='twist_controller',
    #     name='twist_controller',
    #     output='screen',
    # )
    
    pure_pursuit_node = Node(
        package='mobile_robot',
        executable='pure_pursuit',
        name='pure_pursuit',
        output='screen',
        remappings=[
         #('/cmd_vel', '/diff_cont/cmd_vel'),
         ('/odom', '/diffbot_base_controller/odom'),
        ],
    )
    #add shared launch
    launchDescriptionObject.add_action(sharedLaunch)
    #launchDescriptionObject.add_action(TimerAction(period=4.0, actions=[twist_controller_node]))
    launchDescriptionObject.add_action(TimerAction(period=4.0, actions=[pure_pursuit_node]))

    return launchDescriptionObject