/**
 * @file publisher.cpp
 * @author Iván Podoroska
 * @brief 
 * @version 0.1
 * @date 2025-06-11
 * 
 * @copyright Copyright (c) 2025
 * 
 * --------
 * Subscription Topic:
 *  None
 * --------
 * Publishing Topics:
 *  /publisher_topic - std_msgs/Int32
 * 
 */
#include "rclcpp/rclcpp.hpp" // ROS 2 C++ Client library
#include "std_msgs/msg/int32.hpp" // Standard message type for integer messages
#include "std_srvs/srv/empty.hpp" // Standard service type for empty requests

#define DEFAULT_FREQUENCY static_cast<std::int32_t>(5)   // Default publishing frequency in Hz
#define DEFAULT_MAX_COUNT static_cast<std::int32_t>(50)  // Default maximum count for published messages
#define MIN_FREQUENCY     static_cast<std::int32_t>(1)   // Minimum frequency allowed. Can not be zero.
#define MAX_FREQUENCY     static_cast<std::int32_t>(100) // Maximum frequency allowed
#define MIN_MAX_COUNT     1                              // Minimun count allowed
#define MAX_MAX_COUNT     std::numeric_limits<std::int32_t>::max() // Maximum count allowed

using namespace std::chrono_literals; // For using time literals like 1s

class CppPublisher : public rclcpp::Node
{
  public:
    CppPublisher() : Node("cpp_publisher"), count_(0)
    {
      // Declarar y obtener parámetros
      this->declare_parameter<std::int32_t>("frequency", DEFAULT_FREQUENCY);
      this->declare_parameter<std::int32_t>("max_count", DEFAULT_MAX_COUNT);

      auto freq_param = this->get_parameter("frequency").as_int(); // Devuelve int64_t
      auto max_param = this->get_parameter("max_count").as_int();  // Devuelve int64_t

      // Validar parámetros
      if (freq_param < MIN_FREQUENCY || freq_param > MAX_FREQUENCY) {
        throw std::runtime_error(
          "frequency parameter must be between " + std::to_string(MIN_FREQUENCY) +
          " and " + std::to_string(MAX_FREQUENCY)
        );
      }
      if (max_param < MIN_MAX_COUNT || max_param > MAX_MAX_COUNT) {
        throw std::runtime_error(
          "max_count parameter must be between " + std::to_string(MIN_MAX_COUNT) +
          " and " + std::to_string(MAX_MAX_COUNT)
        );
      }

      frequency_ = static_cast<std::int32_t>(freq_param);
      max_count_ = static_cast<std::int32_t>(max_param);

      // Create a publisher for std_msgs::msg::Int32 messages
      publisher_ = create_publisher<std_msgs::msg::Int32>("/publisher_topic", 10);

      // Crear servicio para resetear el contador
      reset_service_ = create_service<std_srvs::srv::Empty>(
        "reset_counter",
        std::bind(&CppPublisher::resetCallback, this, std::placeholders::_1, std::placeholders::_2)
      );

      // Crear timer con la frecuencia configurada. 
      // WARN: la división por cero debe estar prevenida en el chequeo inicial.
      auto period = std::chrono::milliseconds(static_cast<std::int32_t>(1000 / frequency_));
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&CppPublisher::timerCallback, this)
    );
      
      RCLCPP_INFO(get_logger(), "Publishing at %d Hz, max_count: %d", frequency_, max_count_);
    }

    private:
    void timerCallback() {
      if (count_ >= max_count_) {
        RCLCPP_INFO(get_logger(), "Reached max_count (%d), not publishing.", max_count_);
        return;
      }
      auto message = std_msgs::msg::Int32();
      message.data = count_++;
      RCLCPP_INFO(get_logger(), "Publishing: %d", message.data);
      publisher_->publish(message);
    }

    void resetCallback(
      const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      RCLCPP_INFO(get_logger(), "Counter reset by service call.");
      count_ = 0;
    }

    std::int32_t count_;     // Current count of published messages
    std::int32_t frequency_; // Frequency of publishing messages in Hz
    std::int32_t max_count_; // Maximum number of messages to publish before stopping
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  };

int main(int args, char *argv[])
{
  rclcpp::init(args, argv); // Initialize the ROS 2 client library
  auto cpp_publisher_node = std::make_shared<CppPublisher>(); // Create an instance of the publisher node
  rclcpp::spin(cpp_publisher_node); // Spin the node to keep it active and processing callbacks
  rclcpp::shutdown(); // Shutdown the ROS 2 client library

  return 0; // Exit the program
}