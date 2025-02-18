cmake_minimum_required(VERSION 3.8)
project(hcs_bt_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
# Inclusion de paquetes de encabezados
include_directories(include)

# Seccion de configuracion de nodos y ejecutables
add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs std_srvs)

# Ejecutable main
add_executable(main_bt src/main_bt.cpp)
# Vincula las dependencias necesarias al ejecutable
ament_target_dependencies(main_bt
  rclcpp
  behaviortree_cpp_v3
  ament_index_cpp
)
target_link_libraries(main_bt check_plugin always_running)

# configuracion de segundo main - Behavior tree general integrando uso de plugins propios y plugins de nav2
add_executable(main_bt_nav_ntegration src/main_bt_nav_ntegration.cpp)
# Vincula las dependencias necesarias al ejecutable
ament_target_dependencies(main_bt_nav_ntegration
  rclcpp
  behaviortree_cpp_v3
  ament_index_cpp
  nav2_behavior_tree
)
target_link_libraries(main_bt_nav_ntegration check_plugin always_running nav_to_pose_plugin goal_create_plugin)


# Seccion de creacion de ejecutor de plugin
# Plugin de chequeo de estatus
add_library(check_plugin SHARED src/check_plugin.cpp)
ament_export_libraries(check_plugin)
ament_target_dependencies(check_plugin rclcpp std_msgs behaviortree_cpp_v3)
# Plugin para continua ejecucion
add_library(always_running SHARED src/always_running.cpp)
ament_export_libraries(always_running)
ament_target_dependencies(always_running rclcpp behaviortree_cpp_v3)
# Plugin para activacion de navegacion
add_library(nav_to_pose_plugin SHARED src/nav_to_pose_plugin.cpp)
ament_export_libraries(nav_to_pose_plugin)
ament_target_dependencies(nav_to_pose_plugin rclcpp rclcpp_action behaviortree_cpp_v3 geometry_msgs nav2_msgs)
# Creador de objetivo
add_library(goal_create_plugin SHARED src/goal_create_plugin.cpp)
ament_export_libraries(goal_create_plugin)
ament_target_dependencies(goal_create_plugin rclcpp behaviortree_cpp_v3 geometry_msgs tf2)

# Instalacion de los plugins como librerias
install(
  TARGETS check_plugin always_running nav_to_pose_plugin goal_create_plugin main_bt main_bt_nav_ntegration
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Configuracion de ejecutables de nodo
install(TARGETS simple_publisher main_bt main_bt_nav_ntegration
  DESTINATION lib/${PROJECT_NAME}
)

# Orden de instalacion de paquetes "include"
install(
  DIRECTORY include/
  DESTINATION include/
)

# Instalacion de carpetas complementarias
install(
  DIRECTORY config launch params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
