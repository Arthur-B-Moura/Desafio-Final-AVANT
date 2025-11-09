"""
vision.launch.py

Launch file para o nó de detecção de linha e mangueira.
Permite configurar parâmetros via linha de comando.

Uso:
    ros2 launch drone_vision vision.launch.py
    ros2 launch drone_vision vision.launch.py camera_topic:=/front_camera/image_raw
    ros2 launch drone_vision vision.launch.py calibration_mode:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declaração de argumentos de linha de comando
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value = '/camera/image_raw',
        description   = 'Tópico da câmera de entrada'
    )
    
    publish_debug_arg = DeclareLaunchArgument(
        'publish_debug_image',
        default_value = 'true',
        description   = 'Se deve publicar imagem de debug anotada'
    )
    
    calibration_mode_arg = DeclareLaunchArgument(
        'calibration_mode',
        default_value = 'false',
        description   = 'Ativa modo de calibração (mostra máscaras HSV)'
    )
    
    # Nó de detecção de linha
    line_detector_node = Node(
        package     = 'drone_vision',
        executable  = 'line_detector',
        name        = 'line_detector_node',
        output      = 'screen',
        emulate_tty = True,
        parameters  = [{
            # Tópico da câmera
            'camera_topic': LaunchConfiguration('camera_topic'),
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
            'calibration_mode': LaunchConfiguration('calibration_mode'),
            
            # Parâmetros HSV para azul (ajuste conforme necessário)
            'azul_h_min': 100,
            'azul_s_min': 150,
            'azul_v_min': 50,
            'azul_h_max': 140,
            'azul_s_max': 255,
            'azul_v_max': 255,
            
            # Parâmetros HSV para vermelho - Faixa 1 (0-10°)
            'red1_h_min': 0,
            'red1_s_min': 120,
            'red1_v_min': 70,
            'red1_h_max': 10,
            'red1_s_max': 255,
            'red1_v_max': 255,
            
            # Parâmetros HSV para vermelho - Faixa 2 (170-180°)
            'red2_h_min': 170,
            'red2_s_min': 120,
            'red2_v_min': 70,
            'red2_h_max': 180,
            'red2_s_max': 255,
            'red2_v_max': 255,
            
            # Processamento
            'smoothing_window': 5,          # Tamanho da janela de suavização
            'min_area': 300,                # Área mínima do contorno (pixels)
            'roi_vertical_fraction': 0.4,   # Fração inferior da imagem (0-1)
            'morph_kernel_size': 5,         # Kernel para morfologia
        }]
    )
    
    return LaunchDescription([
        camera_topic_arg,
        publish_debug_arg,
        calibration_mode_arg,
        line_detector_node,
    ])