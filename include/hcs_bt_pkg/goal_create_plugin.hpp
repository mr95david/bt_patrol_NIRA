#ifndef HCS_BT_PKG__GOAL_CREATE_PLUGIN_HPP_
#define HCS_BT_PKG__GOAL_CREATE_PLUGIN_HPP_

// Seccion de importe de librerias
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/action_node.h>
// Librerias de interfaces
#include <geometry_msgs/msg/pose_stamped.hpp>
// Librerias para calculo de transformaciones
#include <tf2/LinearMath/Quaternion.h>

// Inicializacion de alias de clase
namespace goal_create_plugin
{
    class CreateGoalPose : public BT::SyncActionNode
    {
        // Inicializacion de clase principal
        public:
            CreateGoalPose(const std::string &name, const BT::NodeConfiguration &config) : BT::SyncActionNode(name, config) {};

            static BT::PortsList providedPorts()
            {
                return {
                    BT::InputPort<double>("x"),
                    BT::InputPort<double>("y"),
                    BT::InputPort<double>("theta"),
                    BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose")};
            }

            BT::NodeStatus tick() override;
    };
} // namespace goal_create_plugin


#endif