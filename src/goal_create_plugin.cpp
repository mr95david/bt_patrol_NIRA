#include "hcs_bt_pkg/goal_create_plugin.hpp"

namespace goal_create_plugin
{
    BT::NodeStatus CreateGoalPose::tick()
    {
        double x, y, theta;
        
        if (!getInput("x", x) || !getInput("y", y) || !getInput("theta", theta))
        {
            RCLCPP_ERROR(rclcpp::get_logger("CreateGoalPose"), "Missing x, y, or theta input.");
            return BT::NodeStatus::FAILURE;
        }

        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.stamp = rclcpp::Clock().now();
        goal_pose.header.frame_id = "map";

        goal_pose.pose.position.x = x;
        goal_pose.pose.position.y = y;
        goal_pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        goal_pose.pose.orientation.x = q.x();
        goal_pose.pose.orientation.y = q.y();
        goal_pose.pose.orientation.z = q.z();
        goal_pose.pose.orientation.w = q.w();

        setOutput("goal_pose", goal_pose);
        
        RCLCPP_INFO(rclcpp::get_logger("CreateGoalPose"), "Generated PoseStamped: [x: %.2f, y: %.2f, theta: %.2f]", x, y, theta);
        
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<nav_to_pose_plugin::NavigateToPoseNode>("NavigateToPoseNode");
// }