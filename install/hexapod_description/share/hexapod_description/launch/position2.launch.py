import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler,TimerAction, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit,OnExecutionComplete
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command ,FindExecutable
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.substitutions import Command, FindExecutable ,PathJoinSubstitution

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    share_dir = get_package_share_directory('hexapod_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'hexapodOneLeg.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time':True,'publish_rate': 1000,'robot_description': robot_urdf}
        ]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'hexapod'],
                        output='screen')
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'true'
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )
       
    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        spawn_entity,
         TimerAction(
        period=5.0,
        actions=[
        Node(
            package="controller_manager",
            executable="spawner.py",   
            output='screen',
            arguments=["my_joint_state_broadcaster" ],
            ),
        Node(
            package="controller_manager",
            executable="spawner.py",
            output='screen',    
            arguments=["my_position_controller1" ],
            ),  
        Node(
            package="controller_manager",
            executable="spawner.py",
            output='screen',    
            arguments=["my_position_controller2" ],
            ),  
        Node(
            package="controller_manager",
            executable="spawner.py",
            output='screen',    
            arguments=["my_position_controller3" ],
            ),   

        ]
        ) ,
       
     
 
    ])

 


    ''' RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=[controller_node],
                on_exit=[spawn_entity],
            )
        ),
         RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=[myPosition_node],
                on_exit=[controller_node],
            )
        )
           TimerAction(
        period=5.0,
        actions=[
        Node(
            package="controller_manager",
            executable="spawner.py",   
            output='screen',
            arguments=["my_joint_state_broadcaster"],
            ),
        Node(
            package="controller_manager",
            executable="spawner.py",
            output='screen',    
            arguments=["my_position_controller"],
            ),   

        ]
        ) ,,'''
        