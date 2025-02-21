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
#include "hcs_bt_pkg/nav_to_pose_plugin.hpp"
#include "hcs_bt_pkg/goal_create_plugin.hpp"
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
    factory.registerNodeType<nav_to_pose_plugin::NavigateToPoseNode>("NavigateToPoseNode");
    factory.registerNodeType<goal_create_plugin::CreateGoalPose>("CreateGoalPose");
    // factory.registerNodeType<nav_to_pose_plugin::NavigateToPoseNode>("NavigateToPoseNode");

    // Inscripcion de arbol de comportamiento
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("hcs_bt_pkg");
    std::string bt_xml_file = package_share_dir + "/config/third_bt.xml";
    auto tree = factory.createTreeFromFile(bt_xml_file);

    // Nodo Dummy para procesar callbacks de ROS2
    auto node = rclcpp::Node::make_shared("dummy_node2");

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    // while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
    while (rclcpp::ok())
    {
        // Tick del árbol
        status = tree.tickRoot();

        // Procesamos callbacks del dummy_node2 (si es que hace algo)
        // Iterate through all nodes
        for (const auto& node : tree.nodes)
        {
            std::cout << "Node [" << node->name() << "] status: "
                      << toStr(node->status()) << std::endl;
        }

        if (status != BT::NodeStatus::RUNNING)
        {
            std::cout << "BT execution ended with status: " << BT::toStr(status) << std::endl;
            tree.rootNode()->halt();
        }

        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Finalizar ROS2
    rclcpp::shutdown();
    return 0;
}   