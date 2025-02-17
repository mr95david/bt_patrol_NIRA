# Seccion de importe de librerias
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
# Librerias para llamado de carpetasde paquetes existentes
from ament_index_python.packages import get_package_share_directory
# Seccion de llamado de librerias utilitarias 
import os

# Creacion de funcion de ejecucion de launch
def generate_launch_description():
    # Seccion de obtencion de paquetes instalados
    nav2_bringup = get_package_share_directory("nav2_bringup")
    hcs_bt_pkg = get_package_share_directory('hcs_bt_pkg')

    # Obtencion de ruta de carpetas de paquete
    launch_path = os.path.join(hcs_bt_pkg, 'launch')
    map_path = os.path.join(nav2_bringup, 'maps', 'turtlebot3_world.yaml')
    config_path = os.path.join(hcs_bt_pkg, 'params', 'hcs_config.yaml')

    # Definicion de nodos de ejecucion individual
    lifecycle_nodes = [
        'map_server', 
        'amcl',
        'planner_server',
        'controller_server',
        'recoveries_server',
    ]

    # Seccion de declaracion de nodos para ejecucion
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': True}, 
            {'yaml_filename':map_path}
        ]
    )
    amcl_server_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[config_path]
    )
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters= [config_path],
    )
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[config_path]
    )
    recovery_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='recoveries_server',
        parameters=[config_path],
        output='screen'
    )

    # Defincion de nodos cicle nodes
    node_lifecycle_life = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': lifecycle_nodes}
        ]
    )

    # Seccion de adicion de nodos de ejecucion de launch
    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    ld.add_action(map_server_node)
    ld.add_action(amcl_server_node)
    ld.add_action(controller_server_node)
    ld.add_action(planner_server_node)
    ld.add_action(recovery_server_node)
    
    ld.add_action(node_lifecycle_life)
    return ld