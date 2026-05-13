#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace dvl_composable_node
{
class DVLNode : public rclcpp::Node
{
public:
  explicit DVLNode(const rclcpp::NodeOptions & options)
  : Node("dvl_msg_converter", options)
  {
    
    publisher_dvl_filtered = this->create_publisher<std_msgs::msg::String>("topico_saida", 10);
    
    subscriber_dvl_sim = this->create_subscription<std_msgs::msg::String>("topico_entrada",
                                                                         10, 
                                                                         std::bind(&NoProcessador::on_message_received, 
                                                                         this, std::placeholders::_1));
    
  }

private:
  // 3. O Callback: Executado toda vez que chegar uma mensagem no 'topico_entrada'
  void on_message_received(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Recebido: '%s'", msg->data.c_str());

    // Processa o dado recebido
    auto nova_mensagem = std_msgs::msg::String();
    nova_mensagem.data = "Processado: " + msg->data;

    // Publica o resultado no tópico de saída
    RCLCPP_INFO(this->get_logger(), "Publicando: '%s'", nova_mensagem.data.c_str());
    publisher_dvl_filtered->publish(nova_mensagem);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_dvl_sim;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_dvl_filtered;
};
} // namespace meu_projeto_composicao

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dvl_composable_node::DVLNode)
