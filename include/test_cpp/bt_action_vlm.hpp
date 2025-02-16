#ifndef TEST_CPP__BT_ACTION_VLM_HPP_
#define TEST_CPP__BT_ACTION_VLM_HPP_

// Importe de librerias utilitarias
#include <chrono>
#include <memory>
#include <thread>
// Importe de librerias necesarias de ros
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// Importe de librerias de interfaces
#include "sensor_msgs/msg/image.hpp"
#include "hcs_ollama_msgs/action/get_vlm_response.hpp"

namespace bt_action_vlm
{
// Alias para la acción GetVlmResponse
using GetVlmResponse = hcs_ollama_msgs::action::GetVlmResponse;

// Creacion de clase como un action
class GetVlmResponseBTAction : public BT::SyncActionNode
{
public:
  /// Constructor. Se inicializa el nodo ROS interno, el cliente de acción y la suscripción a la imagen.
  GetVlmResponseBTAction(const std::string & name, const BT::NodeConfiguration & config);

  /// Se definen los puertos (inputs y outputs) del BT node.
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("prompt"),
      BT::InputPort<bool>("validate_stream"),
      BT::OutputPort<std::string>("response")
    };
  }

  /// Función que se invoca cada vez que el BT tickea este nodo.
  BT::NodeStatus tick() override;

private:
  /// Nodo ROS utilizado internamente.
  rclcpp::Node::SharedPtr node_;

  /// Cliente de acción para GetVlmResponse.
  rclcpp_action::Client<GetVlmResponse>::SharedPtr action_client_;

  /// Suscripción para obtener la imagen.
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  /// Última imagen recibida.
  sensor_msgs::msg::Image::SharedPtr image_;

  /// Callback de la suscripción a la imagen.
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
};

}  // namespace TEST_CPP

#endif  // TEST_CPP__GET_VLM_RESPONSE_BT_NODE_HPP_
