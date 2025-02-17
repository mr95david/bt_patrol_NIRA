// Seccion de inclusion de librerias
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

class ValidationNode : public rclcpp::Node
{
public:
    ValidationNode() : Node("validation_node"), validation_value_(false)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("/validation", 10);
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_validation",
            std::bind(&ValidationNode::handle_service, this, std::placeholders::_1, std::placeholders::_2));
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&ValidationNode::publish_validation, this));
    }

private:
    void publish_validation()
    {
        auto message = std_msgs::msg::Bool();
        message.data = validation_value_;
        publisher_->publish(message);
    }

    void handle_service(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        validation_value_ = request->data;
        response->success = true;
        response->message = "Validation value updated successfully";
        RCLCPP_INFO(this->get_logger(), "Validation value set to: %s", validation_value_ ? "true" : "false");
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool validation_value_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ValidationNode>());
    rclcpp::shutdown();
    return 0;
}
