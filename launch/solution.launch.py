import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')
    simulator_launch_dir = os.path.join(get_package_share_directory('mpc_rbt_simulator'), 'launch')


    # Simulator
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(simulator_launch_dir, 'simulation.launch.py'))
    )

    #Localization
    localization_node = Node(
        package='mpc_rbt_student',
        executable='localization_node',
        name='localization_node',
        output='screen'
    )

    # RViz2 and config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
    )

    
    return LaunchDescription([
        simulation,
        localization_node,
        rviz_node
    ])
