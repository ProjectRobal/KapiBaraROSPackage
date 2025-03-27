import os
import logging
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,TimerAction,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart,OnProcessExit

from launch_ros.actions import Node
import xacro

from launch.actions import SetEnvironmentVariable
import launch_ros.descriptions

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'kapibara'
    file_subpath = 'description/kapibara.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace = 'KapiBara',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': False}], # add other parameters here if required
        remappings=[
            ('/tf','/KapiBara/tf'),
            ('/tf_static','/KapiBara/tf_static')
        ]
    )

    controller_params_file = os.path.join(get_package_share_directory(pkg_name),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace = 'KapiBara',
        parameters=[{"robot_description": robot_description_raw},
                    controller_params_file],
        arguments=["--ros-args","--log-level","info"]
    )

    quaterion_to_euler = Node(
        package="kapibara",
        executable="quanterion_decoder.py",
        namespace = 'KapiBara',
        arguments=[]
    )
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace = 'KapiBara',
        arguments=["motors","--ros-args","--log-level","info"],
    )

    delayed_diff_drive_spawner= RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner]
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace = 'KapiBara',
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner= RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner]
        )
    )
    
    ears_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace = 'KapiBara',
        arguments=["ears_controller"],
    )

    delayed_ears_controller_spawner= RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[ears_controller_spawner]
        )
    )
    
    vel_saltis_bridge = Node(package="kapibara_vel_saltis_bridge",executable='bridge',
                             arguments=["--ros-args","--log-level","info"],
                             namespace = 'KapiBara',
                             output='screen')
    
    twingo_bridge = Node(package="kapibara_twingo_bridge",executable='bridge',
                             arguments=["--ros-args","--log-level","info"],
                             namespace = 'KapiBara',
                             output='screen')
    
    enable_board_services =  Node(package="modus_board",executable='enable_boards.py',
                             arguments=[],
                             namespace = 'KapiBara',
                             output='screen')
    
    reset_board_services =  Node(package="modus_board",executable='reset_boards.py',
                             arguments=[],
                             namespace = 'KapiBara',
                             output='screen')
    
    start_sequence = Node(package="start_sequence",executable='start.py',
                             arguments=[],
                             namespace = 'KapiBara',
                             output='screen',
                             parameters=[
                    {"imu_cfg_path": "/app/src/cfg/imu.json"},
                    {"pid_cfg_path": "/app/src/cfg/pid.json"},
                    {"fusion_cfg_path": "/app/src/cfg/fusion.json"}
                            ])
    
    delayed_start_sequence_vel_saltis = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=vel_saltis_bridge,
            on_start=[start_sequence]
        )
    )
    
    after_start_seq = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_sequence,
            on_exit=[controller_manager]
        )
    )
    
    
    # delayed_start_sequence = TimerAction(period=10.0,actions=[
    #     start_sequence
    # ])    
    
    
    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        namespace = 'KapiBara',
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[
            {
                "camera":0,
                "width":640,
                "height":480,
                "format":"RGB888",
                "role":"video"
            }
        ]
    )

    
    emotions = Node(
        package="emotion_estimer",
        executable="estimator.py",
        namespace = 'KapiBara',
        arguments=["--ros-args","--log-level","info"],
        parameters=[],
        output='screen'
    )
    
    mic = Node(
        package="microphone",
        executable="mic.py",
        namespace = 'KapiBara',
        arguments=[],
        parameters=[],
        output='screen'
    )
    
    mind = Node(
        package="kapibara_mind",
        executable="mind",
        namespace="KapiBara",
        arguments=["--ros-args","--log-level","info"],
        parameters=[]
    )
    
    camera_after_start_seq = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_sequence,
            on_exit=[camera_node]
        )
    )

    
    delayed_emotions = TimerAction(period=10.0,actions=[
        emotions
    ])   
    
    mic_after_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_sequence,
            on_start=[mic]
        )
    ) 
     
    emotions_after_start = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_sequence,
            on_exit=[emotions]
        )
    )
    
    mind_after_emotions = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=emotions,
            on_start=[mind]
        )
    ) 
    
    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
        enable_board_services,
        reset_board_services,
        vel_saltis_bridge,
        twingo_bridge,
        delayed_start_sequence_vel_saltis,
        after_start_seq,
        
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        delayed_ears_controller_spawner,
        mic_after_start,
        emotions_after_start,
        # mind_after_emotions,
        # quaterion_to_euler,
        camera_after_start_seq,
        # sport_face_after_camera
        # midas_after_camera
    ])


