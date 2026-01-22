from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # pslam configのpath指定
    config_dir = os.path.join(
        get_package_share_directory('plain_slam_ros2'),
        'config'
    )
    lio_config_yaml = os.path.join(config_dir, 'lio_3d_config.yaml')
    rviz_config_path = os.path.join(config_dir, 'lio_3d.rviz')
  
    # default_value は指定がない場合に使われます
    use_mapdir_arg = DeclareLaunchArgument(
        'use_mapdir',
        default_value='/tmp/pslam_data/',
        description='Specifying the directory to use the map'
    )
  
    # Main Node (plain_slam_ros2)
    lio_node = Node(
        package='plain_slam_ros2',
        executable='lio_3d_node',
        name='lio_3d_node',
        output='screen',  # ログを見やすくするため screen に変更（お好みで log に戻してください）
        parameters=[
            lio_config_yaml,
            {'use_as_localizer': True},
            {'map_cloud_dir': LaunchConfiguration('use_mapdir')},
            {'param_files_dir': config_dir}
        ]
    )
  
    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        output='screen',
        arguments=['--display-config', rviz_config_path]
    )

    return LaunchDescription([
        use_mapdir_arg,
        lio_node,
        rviz_node
    ])