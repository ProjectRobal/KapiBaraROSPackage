import os
import logging
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

from launch.actions import SetEnvironmentVariable

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'kapibara'
    file_subpath = 'description/kapibara.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file,mappings={'sim_mode' : 'true'}).toxml()

    gazebo_env = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_prefix("kapibara"), "share"))

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace = 'KapiBara',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}], # add other parameters here if required
        remappings=[
            ('/tf','/KapiBara/tf'),
            ('/tf_static','/KapiBara/tf_static')
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py',)]),
            launch_arguments={
                'world': '/app/src/rviz/playground.sdf',
                'params_file': os.path.join(get_package_share_directory(pkg_name),"config/gazebo.yaml"),
                }.items()
        )
    
    state_publisher = Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
                    arguments=[],
                    output='screen')
    
    quaterion_to_euler = Node(
        package="kapibara",
        executable="quanterion_decoder.py",
        namespace = 'KapiBara',
        arguments=[]
    )

    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=["-topic","/KapiBara/robot_description","-entity","kapibara","-timeout","240","-z","1"],
                    namespace="KapiBara",
                    output='screen')
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="KapiBara",
        arguments=["motors",'--controller-manager-timeout','240'],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="KapiBara",
        arguments=["joint_broad",'--controller-manager-timeout','240'],
    )
    
    ears_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="KapiBara",
        arguments=["ears_controller",'--controller-manager-timeout','240'],
    )

    emotions = Node(
        package="emotion_estimer",
        executable="estimator.py",
        namespace="KapiBara",
        parameters=[]
    )
    
    mind = Node(
        package="kapibara_mind",
        executable="mind",
        namespace="KapiBara",
        parameters=[]
    )
    
    mqtt_bridge = Node(
        package="mqtt_bridge",
        executable="mqtt_bridge",
        namespace=""
    )
    
    # Run the node
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        #rviz,
        state_publisher,
        spawn,
        emotions,
        diff_drive_spawner,
        joint_broad_spawner,
        ears_controller_spawner,
        # quaterion_to_euler,
        mind,
        mqtt_bridge
    ])


