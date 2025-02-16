cmake_minimum_required(VERSION 3.8)
project(test_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hcs_ollama_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)
# Seccion de creacion de ejecutables de nodos
add_executable(vlm_client src/vlm_client.cpp)
target_include_directories(vlm_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(vlm_client PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  vlm_client
  "rclcpp"
  "rclcpp_action"
  "sensor_msgs"
  "hcs_ollama_msgs"
)

add_executable(vlm_client_once src/vlm_client_once.cpp)
target_include_directories(vlm_client_once PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(vlm_client_once PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  vlm_client_once
  "rclcpp"
  "rclcpp_action"
  "sensor_msgs"
  "hcs_ollama_msgs"
)

add_executable(vlm_client_onh src/vlm_client_onh.cpp)
ament_target_dependencies(vlm_client_onh
  "rclcpp"
  "rclcpp_action"
  "sensor_msgs"
  "hcs_ollama_msgs"
)

add_executable(main_bt
  src/main_bt.cpp
  src/bt_nodes_registration.cpp
)
# Vincula las dependencias necesarias al ejecutable
ament_target_dependencies(main_bt
  rclcpp
  rclcpp_action
  sensor_msgs
  hcs_ollama_msgs
  behaviortree_cpp_v3
  ament_index_cpp
)
target_link_libraries(main_bt
  bt_action_vlm
)

# agregar plugin de bt
add_library(bt_action_vlm SHARED
  src/bt_action_vlm.cpp
)
ament_target_dependencies(bt_action_vlm
  rclcpp
  rclcpp_action
  sensor_msgs
  hcs_ollama_msgs
  behaviortree_cpp_v3
)

install(TARGETS 
  vlm_client
  vlm_client_once
  vlm_client_onh
  main_bt
  DESTINATION lib/${PROJECT_NAME})

# Instalacion como libreria
install(
  TARGETS bt_action_vlm main_bt
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES config/bt_tree.xml
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
