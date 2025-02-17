// main_bt.cpp
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "test_cpp/bt_action_vlm.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  // Inicialización de ROS2
  rclcpp::init(argc, argv);

  // Crear la fábrica y registrar los nodos (si no se hace de forma automática, se puede invocar la función del registro)
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<bt_action_vlm::GetVlmResponseBTAction>("GetVlmResponse");

  // Se asume que en bt_nodes_registration.cpp se han registrado los nodos
  // (si usas plugins compartidos, la fábrica los carga dinámicamente)

  // Cargar el árbol a partir del archivo XML
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("test_cpp");
    std::string bt_xml_file = package_share_dir + "/config/bt_tree.xml";
    auto tree = factory.createTreeFromFile(bt_xml_file);
//   std::string bt_xml_file = "path/to/test_vlm_bt.xml";  // Actualiza con la ruta correcta
//   auto tree = factory.createTreeFromFile(bt_xml_file);

  // Bucle de tick: se ejecuta el árbol hasta que retorne SUCCESS o FAILURE
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING)
  {
    status = tree.tickRoot();
    // Permitir que se procesen callbacks de ROS (por ejemplo, para recibir la imagen o la respuesta)
    rclcpp::spin_some(rclcpp::Node::make_shared("dummy_node"));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Finalizar ROS2
  rclcpp::shutdown();
  return 0;
}
