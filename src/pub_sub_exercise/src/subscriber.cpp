/**
 * @file subscriber.cpp
 * @author Iván Podoroska
 * @brief 
 * @version 0.1
 * @date 2025-06-11
 * 
 * @copyright Copyright (c) 2025
 * 
 * --------
 * Subscription Topic:
 *  /publisher_topic - std_msgs/Int32
 * --------
 * Publishing Topics:
 *  None
 * 
 */

#include "rclcpp/rclcpp.hpp" // ROS 2 C++ Client library
#include "std_msgs/msg/int32.hpp" // Handling String messages
#include "std_srvs/srv/empty.hpp" // Standard service type for empty requests

#define RESET_AT_DEFAULT static_cast<std::int32_t>(50) // Default value for reset_at parameter
#define RESET_AT_MIN     1 // Minimum value for reset_at parameter
#define RESET_AT_MAX     std::numeric_limits<std::int32_t>::max() // Maximum value for reset_at parameter

using std::placeholders::_1; // For using placeholders in callbacks

class CppSubscriber : public rclcpp::Node
{
  public:
    CppSubscriber() : Node("cpp_subscriber")
    {
      // Declarar y obtener el parámetro reset_at
      this->declare_parameter<std::int32_t>("reset_at", RESET_AT_DEFAULT);
      reset_at_ = this->get_parameter("reset_at").as_int(); // Devuelve int64_t

      // Validar el parámetro reset_at
      if (reset_at_ < RESET_AT_MIN || reset_at_ > RESET_AT_MAX) {
        throw std::runtime_error(
          "reset_at parameter must be between " + std::to_string(RESET_AT_MIN) +
          " and " + std::to_string(RESET_AT_MAX)
        );
      }
      RCLCPP_INFO(get_logger(), "Reset at: %d (less 1)", reset_at_);

      // Crear el cliente para el servicio de reseteo
      reset_client_ = this->create_client<std_srvs::srv::Empty>("reset_counter");

      // Suscribirse al tópico del contador
      subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        "/publisher_topic", 10,
        std::bind(&CppSubscriber::topicCallback, this, _1)
      );

      RCLCPP_INFO(get_logger(), "Subscribed to /publisher_topic, will reset at: %d", reset_at_);
    }

  private:
    void topicCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
      RCLCPP_INFO(get_logger(), "Received: %d", msg->data);
      if (msg->data == (reset_at_ - 1)) {
        RCLCPP_INFO(get_logger(), "Counter reached %d, calling reset service...", reset_at_);
        if (!reset_client_->wait_for_service(std::chrono::seconds(1))) {
          RCLCPP_WARN(get_logger(), "Reset service not available.");
          return;
        }
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = reset_client_->async_send_request(request);
        // No es necesario esperar la respuesta para este servicio vacío
      }
    }

    std::int32_t reset_at_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv); // Initialize the ROS 2 client library

  auto cpp_subscriber_node = std::make_shared<CppSubscriber>(); // Create an instance of the CppSubscriber node
  rclcpp::spin(cpp_subscriber_node); // Spin the node to keep it active and processing callbacks
  rclcpp::shutdown(); // Shutdown the ROS 2 client library

  return 0; // Exit the program
}
