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
    hcs_bt_pkg = get_package_share_directory('hcs_bt_pkg')

    # Rutas de paquetes y launch pkg
    hcs_nav_launch = os.path.join(hcs_bt_pkg, "launch", "hcs_nav_launch.launch.py")
    
    # Inicializacion y definicion de nodos a lanzar
    hcs_bt_node = Node(
        package='hcs_bt_pkg',
        executable='main_bt_nav_ntegration',
        name='main_bt_nav_ntegration',
        output='screen',
    )

    # Seccion de adicion de nodos de ejecucion de launch
    ld = LaunchDescription()
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(hcs_nav_launch)))
    ld.add_action(hcs_bt_node)
    return ld