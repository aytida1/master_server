import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
import time



def generate_launch_description():

    package_dir = get_package_share_directory('master_server')
    bringup_dir = os.path.join(package_dir, 'launch', 'nav2_bringup')

    params_file = os.path.join(package_dir, 'config', 'nav2_params.yaml')

    ld = LaunchDescription()

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    map_server=Node(package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(package_dir, 'map', 'india_poc_floor.yaml'),
                     }],
        remappings=remappings)

    map_server_lifecyle=Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server']}])


    ld.add_action(map_server)
    ld.add_action(map_server_lifecyle)

    

    bringup_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'bringup_launch.py')),
                    launch_arguments={  
                                    'slam': 'False',
                                    'namespace': 'agv1',
                                    'use_namespace': 'True',
                                    'map': '',
                                    'map_server': 'False',
                                    'params_file': params_file,
                                    'default_bt_xml_filename': os.path.join(
                                        get_package_share_directory('nav2_bt_navigator'),
                                        'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                                    'autostart': 'true',
                                    'use_sim_time': 'False', 'log_level': 'info'}.items()
                                    )

    # Add a 5-second delay before starting bringup_cmd
    delayed_bringup_cmd = TimerAction(
        period=5.0,
        actions=[bringup_cmd]
    )

    ld.add_action(delayed_bringup_cmd)
    
    return ld