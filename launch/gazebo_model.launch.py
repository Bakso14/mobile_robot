import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():

    #definisi nama, harus sama dengan nama yang ada di xacro
    robotXacroName = 'differential_drive_robot'
    #nama package sama dengan yang package yang dibuat dengan ros2 pkg create
    namePackage = 'mobile_robot'
    #file utama robot
    modelFileRelativePath = 'model/robot.xacro'
    #path lengkap ke file xacro
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    #konversi xacro ke xml
    robotDescription = xacro.process_file(pathModelFile).toxml()
    #menjalankan gazebo
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    )

    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments={
            'gz_args': ['-r -v -v4 empty.sdf'],
            'on_exit_shutdown': 'true'
            }.items()
        )
    
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName,
            '-topic', 'robot_description',
            '-z', '0.1',
        ],
        output='screen',
    )

    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription,
            'use_sim_time': True}]
    )

    bridge_params = os.path.join(
        get_package_share_directory(namePackage),
        'parameters',
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


    LaunchDescriptionObject = LaunchDescription()

    LaunchDescriptionObject.add_action(gazeboLaunch)
    LaunchDescriptionObject.add_action(spawnModelNodeGazebo)
    LaunchDescriptionObject.add_action(nodeRobotStatePublisher)
    LaunchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)

    return LaunchDescriptionObject