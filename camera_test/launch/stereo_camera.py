import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
  share_dir = get_package_share_directory('camera_test')
  parameter_file = os.path.join(share_dir, 'config', 'stereo_camera.yaml')
  namespace = LaunchConfiguration('namespace', default='')

  stereo_only = LaunchConfiguration('stereo_only')
  param_substitutions = {
    'stereo_only': stereo_only
  }

  configured_params = RewrittenYaml(
    source_file=parameter_file,
    root_key=namespace,
    param_rewrites=param_substitutions,
    convert_types=True)

  stereo_camera_node = LifecycleNode(
    package='camera_test',
    executable='stereo_camera',
    name='stereo_camera',
    namespace=namespace,
    output='screen',
    emulate_tty=True,
    parameters=[configured_params])

  return LaunchDescription(
    [
      DeclareLaunchArgument(
        'stereo_only', default_value='false',
        description='Use only stereo'),
      stereo_camera_node
    ]
  )
