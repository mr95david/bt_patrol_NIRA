#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <functional>
// Inlcusion de librerias propias y ros
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "hcs_ollama_msgs/action/get_vlm_response.hpp"

// Definicion de uso de llamado de alias
using namespace std::chrono_literals;
using GetVlmResponse = hcs_ollama_msgs::action::GetVlmResponse;

// Definicion de clase de ejecucion
class ActionClientVlm : public rclcpp::Node
{
  // Creacion de acceso publico
  public:
    using GoalHandleGetVlmResponse = rclcpp_action::ClientGoalHandle<GetVlmResponse>;

    // Construccion de cclase de ejecucion de nodo
    ActionClientVlm() : 
    // Variables de instancia de clase
    Node("node_action_client"), received_image_(false)
    {
      // Crecion de cliente de action propio
      action_client_ = rclcpp_action::create_client<GetVlmResponse>(
        this, "/get_vlm_response");
      // subscription creado para lectura de imagen en tiempo real
      image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/color/image_raw",
        10,
        std::bind(&ActionClientVlm::callbackImagen, this, std::placeholders::_1)
      );
    }

    // Funcion para validacion de recibimiento de imagen
    bool has_image() const
    {
      return image_ != nullptr;
    }

    // Función que se ejecuta al recibir un mensaje de imagen
    void callbackImagen(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      image_ = msg;
      RCLCPP_INFO(this->get_logger(), "Imagen recibida");
    }

    // Función para enviar la meta a la acción
    void enviarMeta(const std::string & prompt, bool validate_stream)
    {
      if (!image_) {
        RCLCPP_INFO(this->get_logger(), "No se ha recibido ninguna imagen todavía.");
        return;
      }

      // Esperar a que el servidor de acción esté disponible (se espera 10 segundos)
      if (!action_client_->wait_for_action_server(10s)) {
        RCLCPP_ERROR(this->get_logger(), "Servidor de acción no disponible después de 10 segundos");
        return;
      }

      // Configurar la meta
      auto goal_msg = GetVlmResponse::Goal();
      goal_msg.prompt = prompt;
      goal_msg.image = *image_;  // Se hace una copia de la imagen recibida
      goal_msg.validate_stream = validate_stream;

      // Configurar las opciones para enviar la meta, incluyendo callbacks de feedback y resultado
      auto send_goal_options = rclcpp_action::Client<GetVlmResponse>::SendGoalOptions();
      send_goal_options.feedback_callback =
        std::bind(&ActionClientVlm::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
      send_goal_options.result_callback =
        std::bind(&ActionClientVlm::getResultCallback, this, std::placeholders::_1);

      // Enviar la meta de forma asíncrona
      action_client_->async_send_goal(goal_msg, send_goal_options);

      received_image_ = true;
    }

  private:
    // Cliente de acción
    rclcpp_action::Client<GetVlmResponse>::SharedPtr action_client_;
    // Subscripción para recibir la imagen
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    // Puntero a la imagen recibida
    sensor_msgs::msg::Image::SharedPtr image_;
    // Bandera para saber si ya se envió la meta (se actualiza una vez que se recibe la imagen y se envía la meta)
    bool received_image_;

    // Callback para recibir feedback parcial de la acción
    void feedbackCallback(
      GoalHandleGetVlmResponse::SharedPtr /*goal_handle*/,
      const std::shared_ptr<const GetVlmResponse::Feedback> feedback)
    {
      RCLCPP_INFO(this->get_logger(), "Respuesta parcial: %s", feedback->partial_response.c_str());
    }

    // Callback para procesar el resultado final de la acción
    void getResultCallback(const GoalHandleGetVlmResponse::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "La meta fue abortada");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "La meta fue cancelada");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Resultado desconocido");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Resultado: %s", result.result->response.c_str());
      // Apagar el nodo tras recibir el resultado
      rclcpp::shutdown();
    }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto cliente = std::make_shared<ActionClientVlm>();

  // Parámetros para la meta
  std::string prompt = "what objects can see, do a list for me.";
  bool validate_stream = true;

  // Se crea una tasa para el loop
  rclcpp::Rate rate(10);

  // Esperar hasta recibir una imagen
  while (rclcpp::ok() && !cliente->has_image()) {
    rclcpp::spin_some(cliente);
    rate.sleep();
  }

  // Una vez recibida la imagen, se envía la meta
  cliente->enviarMeta(prompt, validate_stream);

  // Se sigue procesando callbacks hasta que se reciba el resultado y se cierre el nodo
  rclcpp::spin(cliente);
  rclcpp::shutdown();
  return 0;
}
