from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo, TimerAction
import os

def generate_launch_description():
    
    # 1. MicroXRCEAgent
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    # 2. PX4 SITL con Gazebo (Usamos ExecuteProcess porque es un Makefile)
    # Importante: Aquí forzamos la versión de Gazebo y la ruta del Autopilot
    px4_autopilot_dir = os.path.expanduser('~/Documents/edag_dron/src/PX4-Autopilot')
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500'],
        cwd=px4_autopilot_dir,
        additional_env={'GZ_VERSION': 'harmonic'},
        output='screen'
    )

    # 3. Script Emisor de Cámara (el que lee de Gazebo y manda a UDP)
    # Si no lo tienes como nodo registrado, lo ejecutamos como proceso de python
    camera_script_path = os.path.expanduser('~/Documents/edag_dron/src/vision/scripts/camera.py')
    camera_emitter = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', camera_script_path],
                output='screen'
            )
        ]
    )

    # 4. Nodo Receptor UDP (El que crea el tópico de ROS 2)
    receptor_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='vision',
                executable='receptor_udp',
                output='screen'
            )
        ]
    )

    # 5. Tu Nodo de Misión (Mission Handler)
    mission_handler = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='master',
                executable='mission_handler',
                output='screen'
            )
        ]
    )

    foto = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='vision',
                executable='foto',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        LogInfo(msg="🚀 Iniciando ecosistema completo del Dron..."),
        micro_xrce_agent,
        px4_sitl,
        camera_emitter,
        receptor_node,
        mission_handler,
        foto
    ])