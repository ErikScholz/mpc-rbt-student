import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')
    simulator_launch_dir = os.path.join(get_package_share_directory('mpc_rbt_simulator'), 'launch')


    # Simulator
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(simulator_launch_dir, 'simulation.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Localization
    localization_node = Node(
        package='mpc_rbt_student',
        executable='localization_node',
        name='localization_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Planning
    planning_node = Node(
        package='mpc_rbt_student',
        executable='planning_node',
        name='planning_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # RViz2 and config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    
    return LaunchDescription([
        rviz_node,

        TimerAction(
            period = 3.0,
            actions = [simulator]
        ),

        TimerAction(
            period = 10.0,
            actions = [
                localization_node,
                planning_node
            ]
        )
    ])
