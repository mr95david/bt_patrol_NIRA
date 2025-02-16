#ifndef TEST_CPP__VLM_CLIENT_ONH_HPP_
#define TEST_CPP__VLM_CLIENT_ONH_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "hcs_ollama_msgs/action/get_vlm_response.hpp"

namespace vlm_client_onh {

using GetVlmResponse = hcs_ollama_msgs::action::GetVlmResponse;

class ActionClientVlm : public rclcpp::Node
{
public:
  // Alias para el tipo de GoalHandle de la acción.
  using GoalHandleGetVlmResponse = rclcpp_action::ClientGoalHandle<GetVlmResponse>;

  ActionClientVlm();
  virtual ~ActionClientVlm() = default;

private:
  // Callback invocado al recibir una imagen.
  void callbackImagen(const sensor_msgs::msg::Image::SharedPtr msg);

  // Método para enviar la meta a la acción.
  void enviarMeta(const std::string & prompt, bool validate_stream);

  // Callback para recibir feedback (respuesta parcial).
  void feedbackCallback(
    GoalHandleGetVlmResponse::SharedPtr goal_handle,
    const std::shared_ptr<const GetVlmResponse::Feedback> feedback);

  // Callback para recibir el resultado final de la acción.
  void getResultCallback(const GoalHandleGetVlmResponse::WrappedResult & result);

  // Cliente de acción.
  rclcpp_action::Client<GetVlmResponse>::SharedPtr action_client_;

  // Subscripción al tópico de la imagen.
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  // Puntero a la imagen recibida.
  sensor_msgs::msg::Image::SharedPtr image_;

  // Bandera para asegurar que se envíe la meta solo una vez.
  bool goal_sent_;
};

}  // namespace cliente_de_accion

#endif  // CLIENTE_DE_ACCION__CLIENTE_DE_ACCION_HPP_
