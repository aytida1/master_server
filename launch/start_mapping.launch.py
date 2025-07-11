import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    package_dir = get_package_share_directory('master_server')
    bringup_dir = os.path.join(package_dir, 'launch', 'nav2_bringup')

    ld = LaunchDescription()

    slam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'slam_launch.py')),
            launch_arguments={
                              'use_sim_time': 'False',
                              'namespace': 'agv1'
                            }.items())

    ld.add_action(slam_launch)
    
    return ld