#ifndef HCS_BT_PKG__ALWAYS_RUNNING_HPP_
#define HCS_BT_PKG__ALWAYS_RUNNING_HPP_

// Seccion de llamado de librerias
#include "rclcpp/rclcpp.hpp"
// Libreria para definicion de plugin compatible con behavior trees
#include "behaviortree_cpp_v3/behavior_tree.h"

// Inicializacion de alias de paquete de plugin
namespace always_running
{
    // Creacion de clase de ejecucion de plugin/Nodo
    class AlwaysRunning : public BT::AsyncActionNode
    {   
        // Scope local de variables y funciones de clase
        public:
            // Constructor de clase
            AlwaysRunning(const std::string& name, const BT::NodeConfiguration& config);
            // Destructor de clase
            ~AlwaysRunning() override;

            // Definicion de posibles entradas de etiqueta de nodo
            static BT::PortsList providedPorts() {return {};}; 

            // Inicializacion de tick para ejecucio de nodo
            BT::NodeStatus tick() override;
        // Scope privado de clase
        private:
    };
} // namespace always_running

#endif