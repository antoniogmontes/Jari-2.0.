from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():    
    return LaunchDescription([
    	# Nodo Camera
        Node(
            package="camera",
            executable="camera_node",
            name="camera_node",
            output='screen',  # Muestra salida en consola para depuración
        )
    ])

