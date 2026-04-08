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

    px4_autopilot_dir = os.path.expanduser('~/Documents/edag_dron/src/PX4-Autopilot')
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500'],
        cwd=px4_autopilot_dir,
        additional_env={'GZ_VERSION': 'harmonic'},
        output='screen')

    nodo_camara = TimerAction(period=6.0,actions=[Node(
                package='vision',
                executable='nodo_camara',
                # parameters=[{'use_sim_time': True}],
                output='screen')])

    mission_handler = TimerAction(period=7.0, actions=[Node(
                package='master',
                executable='mission_handler',
                # parameters=[{'use_sim_time': True}],
                output='screen')])

    aruco_detector = TimerAction(period=7.0, actions=[Node(
                package='vision',
                executable='aruco_detector',
                # parameters=[{'use_sim_time': True}],
                output='screen')])

    foto = TimerAction(period=7.0,actions=[Node(
                package='vision',
                executable='foto',
                # parameters=[{'use_sim_time': True}],
                output='screen')])

    return LaunchDescription([
        LogInfo(msg="🚀 Iniciando ecosistema completo del Dron..."),
        micro_xrce_agent,
        px4_sitl,
        nodo_camara,
        aruco_detector,
        mission_handler,
        foto
    ])