/*
    Nodo encargado de la ejecucion general de un bt, 
    incluyendo la declaracion y registro de las etiquetas 
    propias o existentes usadas para la ejecucion general.
*/

// Seccion de llamado de librerias necesarias
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "hcs_bt_pkg/always_running.hpp"
#include "hcs_bt_pkg/check_plugin.hpp"
// Libreria para busqueda de direcciones de un paquete especifico
#include "ament_index_cpp/get_package_share_directory.hpp"
// Librerias utilitarias
#include <chrono>
#include <thread>

// Llamado de alias usadas de maanera comun en el programa
using namespace std::chrono_literals;


// Ejecucion de main general
int main(int argc, char ** argv)
{
    // Inicializacion de ros2
    rclcpp::init(argc, argv);

    // Creacion de fabrica para registro inicial de etiquetas, nodos y actions para el behavior tree
    BT::BehaviorTreeFactory factory;

    // registro de nodos personalizados
    factory.registerNodeType<always_running::AlwaysRunning>("AlwaysRunning");
    factory.registerNodeType<check_plugin::CheckValCondition>("CheckValCondition");

    // Inscripcion de arbol de comportamiento
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("hcs_bt_pkg");
    std::string bt_xml_file = package_share_dir + "/config/first_bt.xml";
    auto tree = factory.createTreeFromFile(bt_xml_file);

    // Nodo Dummy para procesar callbacks de ROS2
    auto node = rclcpp::Node::make_shared("dummy_node");
    

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING)
    // while (rclcpp::ok())
    {
        BT::NodeStatus status = tree.tickRoot();

        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE)
        {
            // RCLCPP_INFO(node->get_logger(), "BT Finalizado. Reiniciando...");
            status = BT::NodeStatus::RUNNING;
        }

    }

    // Finalizar ROS2
    rclcpp::shutdown();
    return 0;
}   