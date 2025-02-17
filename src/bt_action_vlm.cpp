#include "test_cpp/bt_action_vlm.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

using namespace std::chrono_literals;

namespace bt_action_vlm
{

GetVlmResponseBTAction::GetVlmResponseBTAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
{
  // Inicializar ROS (en un BT ya iniciado se puede omitir si ya existe un rclcpp::init)
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  // Crear un nodo ROS propio para este action node
  node_ = rclcpp::Node::make_shared("get_vlm_response_bt_action_node");

  // Crear el cliente de acción
  action_client_ = rclcpp_action::create_client<GetVlmResponse>(node_, "/get_vlm_response");

  // Suscribirse al tópico de la imagen
  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "/color/image_raw", 10,
    std::bind(&GetVlmResponseBTAction::imageCallback, this, std::placeholders::_1)
  );
}

void GetVlmResponseBTAction::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Guardamos la última imagen recibida
  image_ = msg;
}

BT::NodeStatus GetVlmResponseBTAction::tick()
{
  // Obtener parámetros de entrada del BT
  std::string prompt;
  bool validate_stream = true;
  if (!getInput<std::string>("prompt", prompt))
  {
    throw BT::RuntimeError("Missing parameter [prompt] in GetVlmResponseBTAction");
  }
  if (!getInput<bool>("validate_stream", validate_stream))
  {
    // Si no se recibe el parámetro, se utiliza true por defecto
    validate_stream = true;
  }

  // Esperar a recibir una imagen (se espera hasta 5 segundos)
  auto start_time = std::chrono::steady_clock::now();
  while (!image_ && (std::chrono::steady_clock::now() - start_time < 5s))
  {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(100ms);
  }
  if (!image_)
  {
    RCLCPP_ERROR(node_->get_logger(), "No se recibió imagen en el tiempo esperado.");
    return BT::NodeStatus::FAILURE;
  }

  // Esperar a que el servidor de acción esté disponible (hasta 10 segundos)
  if (!action_client_->wait_for_action_server(10s))
  {
    RCLCPP_ERROR(node_->get_logger(), "Servidor de acción no disponible.");
    return BT::NodeStatus::FAILURE;
  }

  // Crear el mensaje de meta
  auto goal_msg = GetVlmResponse::Goal();
  goal_msg.prompt = prompt;
  goal_msg.image = *image_;  // Se copia la imagen recibida.
  goal_msg.validate_stream = validate_stream;

  // Enviar la meta de forma asíncrona y esperar a que se envíe
  auto future_goal = action_client_->async_send_goal(goal_msg);
  if (rclcpp::spin_until_future_complete(node_, future_goal, 10s) !=
       rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error al enviar la meta.");
    return BT::NodeStatus::FAILURE;
  }
  auto goal_handle = future_goal.get();
  if (!goal_handle)
  {
    RCLCPP_ERROR(node_->get_logger(), "Meta rechazada por el servidor.");
    return BT::NodeStatus::FAILURE;
  }

  // Esperar el resultado (se espera hasta 30 segundos)
  auto future_result = action_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node_, future_result, 30s) !=
       rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error al obtener el resultado.");
    return BT::NodeStatus::FAILURE;
  }

  auto result = future_result.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_ERROR(node_->get_logger(), "La meta no tuvo éxito.");
    return BT::NodeStatus::FAILURE;
  }

  // Escribir la respuesta en el puerto de salida del BT
  setOutput("response", result.result->response);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace cliente_de_accion

// Registrar el nodo en la Factoría de BehaviorTree.CPP
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_action_vlm::GetVlmResponseBTAction>("GetVlmResponse");
}
