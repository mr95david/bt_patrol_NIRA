// bt_nodes_registration.cpp
#include "behaviortree_cpp_v3/bt_factory.h"
// #include "publish_image_node.hpp"
// #include "log_response_node.hpp"
#include "test_cpp/bt_action_vlm.hpp"  // Tu plugin ya existente

BT_REGISTER_NODES(factory)
{
  // Registro de tu plugin de acción
  factory.registerNodeType<bt_action_vlm::GetVlmResponseBTAction>("GetVlmResponse");

  // Registro de los nodos auxiliares
//   factory.registerNodeType<PublishImage>("PublishImage");
//   factory.registerNodeType<LogResponse>("LogResponse");
}
