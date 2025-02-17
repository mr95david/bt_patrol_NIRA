#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "hcs_ollama_msgs/action/get_vlm_response.hpp"

using namespace std::chrono_literals;
using GetVlmResponse = hcs_ollama_msgs::action::GetVlmResponse;

class ActionClientVlm : public rclcpp::Node
{
public:
  // Alias para el tipo de GoalHandle de la acción.
  using GoalHandleGetVlmResponse = rclcpp_action::ClientGoalHandle<GetVlmResponse>;

  ActionClientVlm()
  : Node("cliente_de_accion"), goal_sent_(false)
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

  // Callback que se ejecuta cuando se recibe una imagen.
  // Si aún no se ha enviado la meta, se envía inmediatamente y se desactiva la suscripción.
  void callbackImagen(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!goal_sent_) {
      RCLCPP_INFO(this->get_logger(), "Imagen recibida");
      image_ = msg;
      
      // Se definen los parámetros (en este ejemplo se usan valores fijos).
      std::string prompt = "Can you see the capacity in liters that is shown on the box?";
      bool validate_stream = true;

      enviarMeta(prompt, validate_stream);

      // Evitar el reenvío del objetivo en futuras recepciones.
      goal_sent_ = true;
      
      // Cancelar la suscripción para que no se invoque nuevamente este callback.
      image_sub_.reset();
    }
  }

  // Método para enviar la meta a la acción.
  void enviarMeta(const std::string & prompt, bool validate_stream)
  {
    if (!image_) {
      RCLCPP_INFO(this->get_logger(), "No se ha recibido ninguna imagen todavía.");
      return;
    }

    // Esperar a que el servidor de acción esté disponible (se espera hasta 10 segundos).
    if (!action_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Servidor de acción no disponible después de 10 segundos");
      rclcpp::shutdown();
      return;
    }

    // Configurar la meta.
    auto goal_msg = GetVlmResponse::Goal();
    goal_msg.prompt = prompt;
    goal_msg.image = *image_;  // Se hace una copia de la imagen recibida.
    goal_msg.validate_stream = validate_stream;

    // Configurar las opciones para enviar la meta, incluyendo los callbacks de feedback y resultado.
    auto send_goal_options = rclcpp_action::Client<GetVlmResponse>::SendGoalOptions();
    send_goal_options.feedback_callback =
      std::bind(&ActionClientVlm::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&ActionClientVlm::getResultCallback, this, std::placeholders::_1);

    // Enviar la meta de forma asíncrona.
    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  // Callback para procesar feedback (respuesta parcial) de la acción.
  void feedbackCallback(
    GoalHandleGetVlmResponse::SharedPtr /*goal_handle*/,
    const std::shared_ptr<const GetVlmResponse::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Respuesta parcial: %s", feedback->partial_response.c_str());
  }

  // Callback que se ejecuta cuando se obtiene el resultado final de la acción.
  // Al finalizar, se cierra el nodo.
  void getResultCallback(const GoalHandleGetVlmResponse::WrappedResult & result)
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

private:
  // Cliente de acción.
  rclcpp_action::Client<GetVlmResponse>::SharedPtr action_client_;
  // Subscripción al tópico de imagen.
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  // Puntero a la imagen recibida.
  sensor_msgs::msg::Image::SharedPtr image_;
  // Bandera para asegurar que se envíe la meta solo una vez.
  bool goal_sent_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Crear el nodo.
  auto node = std::make_shared<ActionClientVlm>();

  // Spin bloqueante; el nodo se cerrará una vez que se reciba el resultado de la acción.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
