import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Path configuration
    # Assuming your package is named robot_tfg_pkg
    pkg_dir = get_package_share_directory('robot_tfg_pkg')
    
    # Path to the URDF file (Make sure it is located in the package's urdf folder)
    urdf_path = os.path.join(pkg_dir, 'urdf', 'robot.urdf')

    # Read the contents of the URDF file (Equivalent to the $(cat ...) command in your script)
    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()
        
    return LaunchDescription([
        # 1. Arduino bridge (Motors and Servo)
        Node(
            package='robot_tfg_pkg',
            executable='arduino_bridge',
            name='arduino_bridge',
            output='screen'
        ),
        # 2. ESP32 sensors (TF-Luna LiDAR + MPU-9150 IMU)
        Node(
            package='robot_tfg_pkg',
            executable='esp32_bridge',
            name='esp32_bridge',
            output='screen'
        ),
        # 3. Robot controller
        Node(
            package='robot_tfg_pkg',
            executable='controller_node',
            name='robot_controller',
            output='screen'
        ),
        # 4. 2D Radar processing node
        Node(
            package='robot_tfg_pkg',
            executable='radar_scanner',
            name='radar_scanner',
            output='screen'
        ),
        # 5. Robot odometry
        Node(
            package='robot_tfg_pkg',
            executable='odometria',
            name='odometry_node'
        ),
        # 6. High-level controller
        Node(
            package='robot_tfg_pkg',
            executable='high_level_controller',
            name='high_level_controller_node'
        ),
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[
                {'frame_id': 'odom'},
                {'base_frame_id': 'base_link'},
                {'resolution': 0.05}
            ],
            remappings=[
                # Connect to the point cloud topic published by radar_scanner
                ('/cloud_in', '/cloud_in')
            ]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='robot_tfg_pkg',
            executable='camera_publish',
            name='camera_publish_node',
        )
    ])