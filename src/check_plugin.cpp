// Importe de librerias
#include "hcs_bt_pkg/check_plugin.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace check_plugin
{   
    // Constructor de clase
    CheckValCondition::CheckValCondition(const std::string & name, const BT::NodeConfiguration & config)
    : BT::ConditionNode(name, config),
    value_(false)
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
          }
        
    }

    // Destructor de clase
    CheckValCondition::~CheckValCondition()
    {
    // ATENCIÓN:
    // La llamada a rclcpp::shutdown() es global y debe gestionarse con cuidado en aplicaciones con múltiples nodos.
    if (node_)
    {
        rclcpp::shutdown();
    }
    if (executor_thread_.joinable())
    {
        executor_thread_.join();
    }
    }

    // Recepcion de valores de entrada
    BT::NodeStatus CheckValCondition::tick()
    {
        std::string topic_name;
        if (!getInput<std::string>("topic_name", topic_name))
        {
            throw BT::RuntimeError("Missing parameter [topic_name] in CheckValCondition");
        }

        node_ = rclcpp::Node::make_shared("checkCondition_bt_action_node");
        bool_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            topic_name, 10,
            std::bind(&CheckValCondition::boolCallback, this, _1)
        );

        auto start_time = std::chrono::steady_clock::now();
        while (!value_ && (std::chrono::steady_clock::now() - start_time < 5s))
        {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(100ms);
        }
        // executor_thread_ = std::thread([this]() {
        // rclcpp::spin(node_);});


        if (value_)
        {
            RCLCPP_INFO(node_->get_logger(), "Se completo correctamente la tarea");
            value_ = false;
            return BT::NodeStatus::SUCCESS;
        }
        value_ = false;
        RCLCPP_INFO(node_->get_logger(), "No se completo correctamente la tarea");
        return BT::NodeStatus::FAILURE;
    }

    // Callback de subscriptor
    void CheckValCondition::boolCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        value_ = msg->data;
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<check_plugin::CheckValCondition>("CheckValCondition");
}