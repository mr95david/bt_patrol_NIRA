#ifndef HCS_BT_PKG__NAV_TO_POSE_PLUGIN_HPP_
#define HCS_BT_PKG__NAV_TO_POSE_PLUGIN_HPP_

// Seccion de importe de librerias de ros2
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include <behaviortree_cpp_v3/behavior_tree.h>
// Librerias necesarias para el manejo de interfaces y mensajes
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
// Seccion de importe de librerias utilitarias
#include <string>
#include <memory>

// Defincion de alias de clase, para uso dinamico
namespace nav_to_pose_plugin
{
    // Definicion de clase principal
    class NavigateToPoseNode : public BT::StatefulActionNode
    {
        // Definicion de scope local de variables y funciones de clase
        public:
            // Inicializacion de variables para interaccion con accion de navegacion
            using NavigateToPose = nav2_msgs::action::NavigateToPose;
            using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

            // constructor de clase
            NavigateToPoseNode(const std::string &name, const BT::NodeConfiguration &config);
            ~NavigateToPoseNode() override;

            // Funcion para lectura de puertos de entrada
            static BT::PortsList providedPorts()
            {
                return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose")};
            }

            // Inicializacion de estados de nodo
            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;
        // Scope privado de funciones y variables de nodo
        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
            std::shared_future<GoalHandleNavigateToPose::SharedPtr> goal_future_;
            bool goal_sent_;

            // Definicion de ejecutores de nodo
            std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
            std::thread executor_thread_;
    };
} // namespace nav_to_pose_plugin

#endif