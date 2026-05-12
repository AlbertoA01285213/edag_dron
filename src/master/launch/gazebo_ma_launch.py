from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    env = os.environ.copy()
    env['PYTHONUNBUFFERED'] = '1'

    gui_dir = os.path.expanduser('~/Documents/edag_dron/src/drone_gui_pkg')
    qgc_path = os.path.expanduser('~/Downloads/QGroundControl-x86_64.AppImage')
    px4_autopilot_dir = os.path.expanduser('~/Documents/edag_dron/src/PX4-Autopilot')


    micro_xrce_agent = ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'])

    qground_control = TimerAction(period=2.0,actions=[ExecuteProcess(cmd=[qgc_path])])

    gui = TimerAction(period=8.0, actions=[ExecuteProcess(
        cmd = ['python3', 'ma_main_gui.py'], cwd = gui_dir, additional_env=env, output = 'screen')])

   
    drone_elements = []

    poses = [
        "0,0,0,0,0,0",
        "0,1,0,0,0,0",
        "0,2,0,0,0,0",
    ]

    px4_1_env = os.environ.copy()
    px4_1_env['PX4_SYS_AUTOSTART'] = '4001'
    px4_1_env['PX4_SIM_MODEL'] = 'gz_x500_depth'
    px4_1_env['PX4_GZ_MODEL_POSE'] = poses[0]
    px4_1_env['MAV_SYS_ID'] = '1'

    px4_1_sitl = TimerAction(
            period=0.0, # El segundo espera 5s para no saturar
            actions=[ExecuteProcess(
                cmd=['./build/px4_sitl_default/bin/px4', '-i', '1'],
                cwd=px4_autopilot_dir,
                additional_env=px4_1_env,
                output='screen'
            )]
        )
    drone_elements.append(px4_1_sitl)

    px4_2_env = os.environ.copy()
    px4_2_env['PX4_SYS_AUTOSTART'] = '4001'
    px4_2_env['PX4_SIM_MODEL'] = 'gz_x500_mono_cam'
    px4_2_env['PX4_GZ_MODEL_POSE'] = poses[1]
    px4_2_env['MAV_SYS_ID'] = '2'
    px4_2_env['PX4_GZ_STANDALONE'] = '1'

    px4_2_sitl = TimerAction(
            period=10.0, # El segundo espera 5s para no saturar
            actions=[ExecuteProcess(
                cmd=['./build/px4_sitl_default/bin/px4', '-i', '2'],
                cwd=px4_autopilot_dir,
                additional_env=px4_2_env,
                output='screen'
            )]
        )
    drone_elements.append(px4_2_sitl)

    for i in range(1, 3):
        instance = str(i)
        ns = f"px4_{instance}"
        gz_topic_arg = f"/camera_main_{instance}"

        if i == 1:
            px4_1_env['UXRCE_DDS_NS'] = ns
        else:
            px4_2_env['UXRCE_DDS_NS'] = ns

        nodes_delay = 5.0 + (float(i) * 2.0) 
        
        # nodes_group = TimerAction(
        #     period=nodes_delay,
        #     actions=[
        #         Node(package='control', executable='pose_teller',        namespace=ns, output='screen'),
        #         Node(package='master',  executable='mission_handler_2',  namespace=ns, remappings=[
        #                 ('fmu/out/vehicle_odometry', f'/px4_{instance}/fmu/out/vehicle_odometry'),
        #                 ('fmu/in/trajectory_setpoint', f'/px4_{instance}/fmu/in/trajectory_setpoint'),
        #                 ('fmu/in/vehicle_command', f'/px4_{instance}/fmu/in/vehicle_command'),
        #                 ('fmu/in/offboard_control_mode', f'/px4_{instance}/fmu/in/offboard_control_mode'),
        #             ], output='screen'),
        #         Node(package='vision',  executable='aruco_detector',     namespace=ns, output='screen'),
        #         Node(package='vision',  executable='qr_detector',        namespace=ns, output='screen'),
        #         Node(package='vision',  executable='foto',               namespace=ns, output='screen'),
        #         Node(package='vision',  executable='nodo_camara',        namespace=ns, parameters=[{'gz_topic': gz_topic_arg}], output='screen'),
        #     ]
        # )
        # drone_elements.append(nodes_group)

        nodes_group = TimerAction(
            period=nodes_delay,
            actions=[
                Node(package='control', executable='pose_teller',        namespace=ns, output='screen'),
                Node(package='master',  executable='mission_handler_2',  namespace=ns, output='screen'),
                Node(package='vision',  executable='aruco_detector',     namespace=ns, output='screen'),
                Node(package='vision',  executable='qr_detector',        namespace=ns, output='screen'),
                Node(package='vision',  executable='foto',               namespace=ns, output='screen'),
                Node(package='vision',  executable='nodo_camara',        namespace=ns, parameters=[{'gz_topic': gz_topic_arg}], output='screen'),
            ]
        )
        drone_elements.append(nodes_group)

    return LaunchDescription([
        LogInfo(msg="🚀 Iniciando Enjambre de Drones EDAG..."),
        micro_xrce_agent,
        qground_control,
        gui,
        *drone_elements
    ])

# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import ExecuteProcess, LogInfo, TimerAction, DeclareLaunchArgument, OpaqueFunction
# from launch.substitutions import LaunchConfiguration
# import os

# def generate_launch_description():

#     env = os.environ.copy()
#     env['PYTHONUNBUFFERED'] = '1'

#     gui_dir = os.path.expanduser('~/Documents/edag_dron/src/drone_gui_pkg')
#     qgc_path = os.path.expanduser('~/Downloads/QGroundControl-x86_64.AppImage')
#     px4_autopilot_dir = os.path.expanduser('~/Documents/edag_dron/src/PX4-Autopilot')


#     micro_xrce_agent = ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'])

#     qground_control = TimerAction(period=2.0,actions=[ExecuteProcess(cmd=[qgc_path])])

#     gui = TimerAction(period=8.0, actions=[ExecuteProcess(
#         cmd = ['python3', 'main_gui.py'], cwd = gui_dir, additional_env=env, output = 'screen')])

   
#     drone_elements = []

#     poses = [
#         "0,0,0,0,0,0",
#         "0,1,0,0,0,0",
#         "0,2,0,0,0,0",
#     ]

#     for i in range(1, 3):
#         instance = str(i)
#         ns = f"px4_{instance}"
    
#         px4_env = os.environ.copy()
#         px4_env['PX4_SYS_AUTOSTART'] = '4001'
#         px4_env['PX4_SIM_MODEL'] = 'gz_x500_depth'
#         px4_env['PX4_GZ_MODEL_POSE'] = poses[i-1]

#         px4_env['MAV_SYS_ID'] = instance
#         # px4_env['PX4_GZ_ID'] = ns

#         if i == 1:
#             startup_delay = 0.0

#         else:
#             px4_env['PX4_GZ_STANDALONE'] = '1'
#             startup_delay = 10.0*i

#         px4_sitl = TimerAction(
#             period=startup_delay, # El segundo espera 5s para no saturar
#             actions=[ExecuteProcess(
#                 cmd=['./build/px4_sitl_default/bin/px4', '-i', instance],
#                 cwd=px4_autopilot_dir,
#                 additional_env=px4_env,
#                 output='screen'
#             )]
#         )
#         drone_elements.append(px4_sitl)

#         # nodes_delay = 10.0 + (float(i) * 2.0) 
        
#         # nodes_group = TimerAction(
#         #     period=nodes_delay,
#         #     actions=[
#         #         Node(package='vision', executable='nodo_camara', namespace=ns, output='screen'),
#         #         Node(package='vision', executable='aruco_detector', namespace=ns, output='screen'),
#         #         Node(package='vision', executable='qr_detector', namespace=ns, output='screen'),
#         #         Node(package='vision', executable='foto', namespace=ns, output='screen'),
#         #         Node(package='control', executable='pose_teller', namespace=ns, output='screen'),
#         #         Node(package='master', executable='mission_handler_2', namespace=ns, output='screen'),
#         #     ]
#         # )
#         # drone_elements.append(nodes_group)


#     return LaunchDescription([
#         LogInfo(msg="🚀 Iniciando Enjambre de Drones EDAG..."),
#         micro_xrce_agent,
#         qground_control,
#         gui,
#         *drone_elements
#     ])