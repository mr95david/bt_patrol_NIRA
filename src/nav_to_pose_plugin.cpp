#include "hcs_bt_pkg/nav_to_pose_plugin.hpp"

namespace nav_to_pose_plugin
{
    // Ejecucion de construccion de clase
    NavigateToPoseNode::NavigateToPoseNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config), goal_sent_(false)
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        node_ = rclcpp::Node::make_shared("navigate_to_pose_bt_node");
        action_client_ = rclcpp_action::create_client<NavigateToPose>(
            node_, 
            "navigate_to_pose"
        );

        // Crear un executor en un hilo aparte para procesar callbacks de este nodo
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);

        executor_thread_ = std::thread([this]() {
            executor_->spin();
        });
        // Ciclo de espera a activacion de servidor de action de navegacion.
        while (!action_client_->wait_for_action_server(std::chrono::seconds(2)))
        {
            RCLCPP_WARN(node_->get_logger(), "Waiting for NavigateToPose action server...");
        }
    }

    NavigateToPoseNode::~NavigateToPoseNode()
    {
        if (executor_) {
            executor_->cancel();
        }
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
    }

    // Estado de inicio de ejecucion de nodo
    BT::NodeStatus NavigateToPoseNode::onStart()
    {
        
        // Variables de definicio  de objetivo
        geometry_msgs::msg::PoseStamped goal_pose;
        if (!getInput("goal_pose", goal_pose))
        {
            RCLCPP_ERROR(node_->get_logger(), "Missing required input [goal_pose]");
            return BT::NodeStatus::FAILURE;
        }
        // Definicion de mensaje de objetivo para el action
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose;
        // Seccion de envio de objetivo
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                setStatus(BT::NodeStatus::SUCCESS);
            }
            else
            {
                setStatus(BT::NodeStatus::FAILURE);
            }
        };
        RCLCPP_ERROR(node_->get_logger(), "Envio en start ejecutado correctamente.");
        goal_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
        goal_sent_ = true;
        return BT::NodeStatus::RUNNING;
    }

    // Seccion de definicion de ejecucion de estado durante ejecucion

    BT::NodeStatus NavigateToPoseNode::onRunning()
    {
        if (!goal_sent_)
        {
            return BT::NodeStatus::FAILURE;
        }

        auto goal_handle = goal_future_.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the server.");
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_ERROR(node_->get_logger(), "Envio en running ejecutado correctamente.");
        auto result_future = action_client_->async_get_result(goal_handle);
        if (result_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready)
        {
            
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }


    // Definicion de estado de cancelado de busqueda de objetivo
    void NavigateToPoseNode::onHalted()
    {
        if (goal_sent_)
        {
            auto goal_handle = goal_future_.get();
            if (goal_handle)
            {
                action_client_->async_cancel_goal(goal_handle);
                RCLCPP_INFO(node_->get_logger(), "NavigateToPose goal canceled.");
            }
        }
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav_to_pose_plugin::NavigateToPoseNode>("NavigateToPoseNode");
}