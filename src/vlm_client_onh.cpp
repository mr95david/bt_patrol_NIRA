#include "test_cpp/vlm_client_onh.hpp"
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace vlm_client_onh {

ActionClientVlm::ActionClientVlm()
: Node("vlm_client_onh"), goal_sent_(false)
{
  // Crear el cliente de acción.
  action_client_ = rclcpp_action::create_client<GetVlmResponse>(this, "/get_vlm_response");

  // Suscribirse al tópico de la imagen.
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/color/image_raw",
    10,
    std::bind(&ActionClientVlm::callbackImagen, this, std::placeholders::_1)
  );
}

void ActionClientVlm::callbackImagen(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!goal_sent_) {
    RCLCPP_INFO(this->get_logger(), "Imagen recibida");
    image_ = msg;

    // Parámetros fijos para la meta (puedes modificarlos o parametrizarlos según necesites).
    std::string prompt = "what objects can see, do a list for me.";
    bool validate_stream = true;

    enviarMeta(prompt, validate_stream);

    // Marcar que ya se envió la meta y cancelar la suscripción para evitar reenvíos.
    goal_sent_ = true;
    image_sub_.reset();
  }
}

void ActionClientVlm::enviarMeta(const std::string & prompt, bool validate_stream)
{
  if (!image_) {
    RCLCPP_INFO(this->get_logger(), "No se ha recibido ninguna imagen todavía.");
    return;
  }

  // Esperar a que el servidor de acción esté disponible (hasta 10 segundos).
  if (!action_client_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(this->get_logger(), "Servidor de acción no disponible después de 10 segundos");
    rclcpp::shutdown();
    return;
  }

  // Configurar la meta.
  auto goal_msg = GetVlmResponse::Goal();
  goal_msg.prompt = prompt;
  goal_msg.image = *image_;  // Se copia la imagen recibida.
  goal_msg.validate_stream = validate_stream;

  // Configurar las opciones para enviar la meta, incluyendo callbacks de feedback y resultado.
  auto send_goal_options = rclcpp_action::Client<GetVlmResponse>::SendGoalOptions();
  send_goal_options.feedback_callback =
    std::bind(&ActionClientVlm::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&ActionClientVlm::getResultCallback, this, std::placeholders::_1);

  // Enviar la meta de forma asíncrona.
  action_client_->async_send_goal(goal_msg, send_goal_options);
}

void ActionClientVlm::feedbackCallback(
  GoalHandleGetVlmResponse::SharedPtr /*goal_handle*/,
  const std::shared_ptr<const GetVlmResponse::Feedback> feedback)
{
//   RCLCPP_INFO(rclcpp::get_logger("ActionClientVlm"), "Respuesta parcial: %s", feedback->partial_response.c_str());
  RCLCPP_INFO(rclcpp::get_logger("ActionClientVlm"), "Respuesta parcial almacenada.");
}


void ActionClientVlm::getResultCallback(const GoalHandleGetVlmResponse::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "La meta fue abortada");
      rclcpp::shutdown();
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "La meta fue cancelada");
      rclcpp::shutdown();
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Resultado desconocido");
      rclcpp::shutdown();
      return;
  }
  RCLCPP_INFO(this->get_logger(), "Resultado: %s", result.result->response.c_str());
  rclcpp::shutdown();
}

}  // namespace cliente_de_accion

// Función main
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vlm_client_onh::ActionClientVlm>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
