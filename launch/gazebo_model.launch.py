#############ros2 and gazebo launch file for differential drive robot###################

import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro


def generate_launch_description():

    #must match robot name in xacro file
    robotXacroName = 'differential_drive_robot'

    #name of the package
    namePackage = 'mobile_robot'

    #relative path to xacro file
    modelFileRelativePath = 'model/robot.xacro'

    #absolute path to model
    pathModelFile = os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)

    #get description from xacro file
    robotDescription = xacro.process_file(pathModelFile).toxml()

    #launch file from gazebo_ros package
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py'))

    ########launch description#########


    #if using our own world model
    #worldFileRelativePath = 'model/my_world.world'
    #pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)
    #gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-v', '-v4', '-u', pathWorldFile], 'on_exit_shutdown': 'true'}.items())

    #if using empty world model
    #can change -u to -r to start immeiately
    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-r -v -v4 empty.sdf'], 'on_exit_shutdown': 'true'}.items())

    # Gazebo node
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName,
            '-topic', 'robot_description',
        ],
        output='screen',
    )

    # Robot State Publisher Node
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )

    #important for robot control via ROS2
    bridge_params = os.path.join(
        get_package_share_directory(namePackage),
        'parameters/'
        'bridge_parameters.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    #create an empty launch description object
    launchDescriptionObject = LaunchDescription()

    #add gazeboLaunch
    launchDescriptionObject.add_action(gazeboLaunch)

    #add the Nodes
    launchDescriptionObject.add_action(spawnModelNodeGazebo)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)

    return launchDescriptionObject