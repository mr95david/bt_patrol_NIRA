// Importe de librerias propias y necesarias
#include "hcs_bt_pkg/always_running.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

// Inicializacion de alias de clase de ejecucion de nodo
namespace always_running
{
    // Constructor de ejecucion
    AlwaysRunning::AlwaysRunning(const std::string & name, const BT::NodeConfiguration & config)
    : BT::AsyncActionNode(name, config) 
    {
        if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    }
    // Desctructor de nodo
    AlwaysRunning::~AlwaysRunning(){}

    // Llamado de tick de nodo, en este caso el nodo solo tiene la interaccion de retorno continuo de ejecucion
    BT::NodeStatus AlwaysRunning::tick()
    {
        //  RCLCPP_INFO(node_->get_logger(), "Validacion de nodo de ejecucion continua.");
        return BT::NodeStatus::RUNNING;
    }
} // namespace always_running

// Registro de nodo para ejecucion en bt
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<always_running::AlwaysRunning>("AlwaysRunning");
}