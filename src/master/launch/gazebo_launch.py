from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os

# def launch_setup(context, *args, **kwargs):
#     num_drones = int(LaunchConfiguration('num_drones').perform(context))
#     nodes_to_start = []

#     for i in range(num_drones):
#         drone_id = i + 1

#     return nodes_to_start

# def generate_launch_description():
#     return LaunchDescription([
#         DeclareLaunchArgument('num_drones', default_value='1', description='Número de drones'),
#         OpaqueFunction(function=launch_setup)
#     ])

def generate_launch_description():

    env = os.environ.copy()
    env['PYTHONUNBUFFERED'] = '1'

    gui_dir = os.path.expanduser('~/Documents/edag_dron/src/drone_gui_pkg')
    gui = TimerAction(period= 6.0, actions=[ExecuteProcess(
        cmd = ['python3', 'main_gui.py'],
        cwd = gui_dir,
        additional_env=env,
        output = 'screen')])

    
    # 1. MicroXRCEAgent
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        # output='screen'
    )

    qgc_path = os.path.expanduser('~/Downloads/QGroundControl-x86_64.AppImage')
    qground_control = TimerAction(
        period=1.0,
        actions=[ExecuteProcess(
            cmd=[qgc_path],
            # output='screen'
        )]
    )

    # PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,0,0,0,0,0" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4
    # PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="1,0,0,0,0,0" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1


    px4_autopilot_dir = os.path.expanduser('~/Documents/edag_dron/src/PX4-Autopilot')
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500'],
        cwd=px4_autopilot_dir,
        additional_env={'GZ_VERSION': 'harmonic', 'PX4_GZ_VIEW_MODEL': ''},
        output='screen')
    

    nodo_camara = TimerAction(period=6.0,actions=[Node(
                package='vision',
                executable='nodo_camara',
                # parameters=[{'use_sim_time': True}],
                output='screen')])

    mission_handler = TimerAction(period=7.0, actions=[Node(
                package='master',
                executable='mission_handler_2',
                # parameters=[{'use_sim_time': True}],
                output='screen')])

    aruco_detector = TimerAction(period=7.0, actions=[Node(
                package='vision',
                executable='aruco_detector',
                # parameters=[{'use_sim_time': True}],
                output='screen')])
    
    qr_detector = TimerAction(period=7.0, actions=[Node(
                package='vision',
                executable='qr_detector',
                # parameters=[{'use_sim_time': True}],
                output='screen')])

    foto = TimerAction(period=7.0,actions=[Node(
                package='vision',
                executable='foto',
                # parameters=[{'use_sim_time': True}],
                output='screen')])
    
    pose_traducer = TimerAction(period=7.0,actions=[Node(
                package='control',
                executable='pose_teller'
                # parameters=[{'use_sim_time': True}],
                )])

    return LaunchDescription([
        LogInfo(msg="🚀 Iniciando ecosistema completo del Dron..."),
        micro_xrce_agent,
        px4_sitl,
        qground_control,
        gui,
        nodo_camara,
        aruco_detector,
        qr_detector,
        mission_handler,
        foto,
        pose_traducer
    ])