from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():
  # livox launch のpath指定
  livox_package_dir = get_package_share_directory('livox_ros_driver2')
  livox_launch_file = os.path.join(livox_package_dir, 'launch_ROS2', 'msg_MID360_launch.py')
  
  # pslam configのpath指定
  config_dir = os.path.join(
    get_package_share_directory('plain_slam_ros2'),
    'config'
  )
  lio_config_yaml = os.path.join(config_dir, 'lio_3d_config.yaml')
  rviz_config_path = os.path.join(config_dir, 'lio_3d.rviz')
  
  # 使用するマップのディレクトリに関する引数指定
  use_mapdir_arg = DeclareLaunchArgument(
        'use_mapdir',
        default_value='/tmp/pslam_data/',
        description='Specifying the directory to use the map'
    )
  
  # launch, nodeの立ち上げ
  livox_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(livox_launch_file)
    )
  
  lio_node = Node(
        package='plain_slam_ros2',
        executable='lio_3d_node',
        name='lio_3d_node',
        output='log',
        parameters=[
        lio_config_yaml,
        {'use_as_localizer': True},
        {'map_cloud_dir': LaunchConfiguration('use_mapdir')},
        {'param_files_dir': config_dir}
      ]
        )
  
  rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            output='screen',
            arguments=['--display-config', rviz_config_path]
        )

  return LaunchDescription([
    use_mapdir_arg,
    livox_launch,
    lio_node,
    rviz_node
  ])
