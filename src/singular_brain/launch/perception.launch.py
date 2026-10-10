from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('singular_brain')
    
    # Models are installed in the share directory.
    model_path = os.path.join(pkg_dir, 'models', 'yolov5s.onnx')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot (e.g., amr_1)'
    )

    perception_node = Node(
        package='singular_brain',
        executable='perception_node',
        name='perception_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {'model_path': model_path},
            {'confidence_threshold': 0.5}
        ]
    )

    database_node = Node(
        package='singular_brain',
        executable='database_node',
        name='database_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {'global_frame': 'odom'},
            {'db_path': 'spatial_memory.db'}
        ]
    )

    return LaunchDescription([
        namespace_arg,
        perception_node,
        database_node
    ])
