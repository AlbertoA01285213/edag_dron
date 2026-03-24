from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable, ExecuteProcess
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

# Puente Gazebo ========================================
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
            '-topic', 'robot_description'
        ],
        output='screen'
    )
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        additional_env={'IGN_PARTITION': 'dron_sim'},
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/camera@sensor_msgs/msg/Image[ignition.msgs.Image',
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
    ])
