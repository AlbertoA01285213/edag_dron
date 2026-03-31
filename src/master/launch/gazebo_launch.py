from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    set_qt_platform = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')

    pkg_share_dron_vizualization = get_package_share_directory('visualizacion')
    pkg_dron_plugins = get_package_share_directory('gazebo_plugins')
    pkg_dron_descripcion = get_package_share_directory('description')
    pkg_models_path = os.path.join(get_package_share_directory('visualizacion'), 'models')

    path_modelos_descripcion = os.path.join(pkg_dron_descripcion, '..')

    visualizacion_install_path = os.path.join(pkg_share_dron_vizualization, '..')

    set_res_viz = AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', visualizacion_install_path)
    set_mod_viz = AppendEnvironmentVariable('IGN_GAZEBO_MODEL_PATH', visualizacion_install_path)

    set_plugin_path = AppendEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', os.path.join(pkg_dron_plugins, '../../lib'))
    set_ign_resource_path = AppendEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=path_modelos_descripcion)
    set_mod_path = AppendEnvironmentVariable('IGN_GAZEBO_MODEL_PATH', pkg_models_path)
    set_res_path = AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', pkg_models_path)

    set_partition = SetEnvironmentVariable('IGN_PARTITION', 'dron_sim')
    set_ip = SetEnvironmentVariable('IGN_IP', '127.0.0.1')

    set_plugin_path = AppendEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', os.path.join(get_package_share_directory('gazebo_plugins'), '../../lib/gazebo_plugins'))


    xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    px4_dir = os.path.expanduser('~/Documents/edag_dron/src/PX4-Autopilot')
    
    # 2. Definimos rutas absolutas para evitar el error de "not found"
    px4_bin = os.path.join(px4_dir, 'build/px4_sitl_default/bin/px4')
    px4_etc = os.path.join(px4_dir, 'ROMFS/px4fmu_common')
    px4_test_data = os.path.join(px4_dir, 'test_data')

    custom_env = os.environ.copy()
    custom_env.update({
        'IGN_PARTITION': 'default',
        'PX4_GZ_WORLD': 'default',
        'HEADLESS': '1',
        'PX4_GZ_STANDALONE': '1',
        'PX4_SYS_AUTOSTART': '4001',
        'PX4_SIM_SIMULATOR': 'gz',
        'PX4_GZ_MODEL_NAME': 'dron', # IMPORTANTE: nombre en Gazebo
        'PX_SIM_MODEL': 'gz_x500',
    })

    px4_sitl = ExecuteProcess(
        cmd=[
            px4_bin,
            px4_etc,
            '-s', 'etc/init.d-posix/rcS',
            '-t', px4_test_data
        ],
        cwd=px4_dir, 
        env=custom_env,
        output='screen'
    )

    # px4_with_delay = TimerAction(
    #     period=10.0,
    #     actions=[px4_sitl]
    # )

# Puente Gazebo ========================================
    sdf_file_path = os.path.join(pkg_share_dron_vizualization, 'urdf', 'dron2.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
            launch_arguments={'gz_args': '-r ' + os.path.join(pkg_share_dron_vizualization, 'worlds', 'mundo.sdf')}.items()
    )
    
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'dron',
            '-file', sdf_file_path,
            '-z', '0.25'
        ],
        output='screen'
    )
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # additional_env={'IGN_PARTITION': 'dron_sim'},
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/camera@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/dron/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'
        ],
        output='screen'
    )

# Cargar modelo del dron ====================================
    urdf_file_path = os.path.join(pkg_share_dron_vizualization, 'urdf', 'dron.urdf')
    with open(urdf_file_path, 'r') as f:
        robot_description_content = f.read()

    dron_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_content,
        }]
    )

# Nodos =====================================================
    prueba = Node(
        package='control',
        executable='prueba',
        name='prueba',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        set_qt_platform,
        set_partition,
        set_ip,
        set_plugin_path,
        set_mod_path,
        set_res_path,
        set_res_viz,
        set_mod_viz,
        set_ign_resource_path,
        gazebo,
        spawn_robot,
        bridge,
        dron_state_publisher,
        # xrce_agent,
        # px4_sitl
        # px4_with_delay


        # prueba
    ])
