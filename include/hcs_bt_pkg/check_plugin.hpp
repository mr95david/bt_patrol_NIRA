#ifndef HCS_BT_PKG__CHECK_PLUGIN_HPP_
#define HCS_BT_PKG__CHECK_PLUGIN_HPP_

// Seccion de importe de librerias necesarias para la ejecucion de nodos
// Librerias de funcionamiento de ros2
#include "rclcpp/rclcpp.hpp"
// Librerias para configuracion de plugins e interfaces
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <std_msgs/msg/bool.hpp>
// Librerias utilitarias
#include <thread>
#include <atomic>
#include <string>

// Llamado de alias y acceso rapido a funciones necesarias
namespace check_plugin
{
// Clase principal de nodo de ejecucion
class CheckValCondition : public BT::ConditionNode
{
    public:
        // Variables de scope publico
        // Constructor de clase
        CheckValCondition(const std::string& name, const BT::NodeConfiguration& config);
        ~CheckValCondition() override;

        // Creacion de puertos de entrada del nodo
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("topic_name")
            };
        };

        // Funcion de control de ejecucion de nodo de bt
        BT::NodeStatus tick() override;

    // Seccion de creaciones del scope privado
    private:
        // Creacion de subscriptor de nodo
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_sub_;
        std_msgs::msg::Bool::SharedPtr bool_val_;
        bool value_;
        rclcpp::Node::SharedPtr node_;
        std::thread executor_thread_;

        // Callback de subscriptor
        void boolCallback(const std_msgs::msg::Bool::SharedPtr msg);
};
}
#endif