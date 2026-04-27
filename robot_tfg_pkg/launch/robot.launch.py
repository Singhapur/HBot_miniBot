import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Configuración de rutas
    # Suponiendo que tu paquete se llama robot_tfg_pkg
    pkg_dir = get_package_share_directory('robot_tfg_pkg')
    
    # Ruta al archivo URDF (Asegúrate de que esté en la carpeta urdf del paquete)
    urdf_path = os.path.join(pkg_dir, 'urdf', 'robot.urdf')

    # Leer el contenido del archivo URDF (Equivale al $(cat ...) de tu script)
    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()
        
    return LaunchDescription([
        # 1. Puente con el Arduino (Motores y Servo)
        Node(
            package='robot_tfg_pkg',
            executable='arduino_bridge',
            name='puente_arduino',
            output='screen'
        ),
        
        # 2. Sensores del ESP32 (LiDAR TF-Luna + IMU MPU-9150)
        Node(
            package='robot_tfg_pkg',
            executable='esp32_bridge',
            name='puente_esp32',
            output='screen'
        ),
        
        # 3. Controlador de robot
        Node(
            package='robot_tfg_pkg',
            executable='controller_node',
            name='controller_robot',
            output='screen'
        ),
        
        # 4. Cerebro del Radar 2D
        Node(
            package='robot_tfg_pkg',
            executable='radar_scanner',
            name='radar_scanner',
            output='screen'
        ),
        # 5. Odometria Robot
        Node(
            package='robot_tfg_pkg',
            executable='odometria',
            name='odometria_node'
        ),
        # 6. Hight Level Controller
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
                # Conectamos el tópico que creaste en radar_scanner 
                ('/cloud_in', '/cloud_in') 
            ]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        )
        
        
        # Nota: Si también quieres arrancar la cámara a la vez, 
        # descomenta las líneas de abajo:
        # ,
        # Node(
        #     package='robot_tfg_pkg',
        #     executable='camera_reader',
        #     name='camera_mediapipe',
        #     output='screen'
        # )
    ])
