#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class EpicNode : public rclcpp::Node
{
  public:
    EpicNode(): Node("epic_node"), count_(0) {

        // Create node_name publisher
        name_publisher_ = this->create_publisher<std_msgs::msg::String>("node_name", 10);
        
        // Creates timer that publishes node_name every X ms
        timer_ = this->create_wall_timer(500ms, std::bind(&EpicNode::timer_callback, this));

        // Battery percentage publisher
        battery_percent_publisher_ = this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);

        // Battery voltage subscriber
        battery_voltage_subscriber_ = this->create_subscription<std_msgs::msg::Float32>("battery_voltage", 10,
            std::bind(&EpicNode::battery_callback, this, std::placeholders::_1));
        
        // Min voltage parameter
        auto batt_volt_min_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        batt_volt_min_param_desc.description = "This parameter changes the minimum battery voltage!";
        this->declare_parameter("battery_min_voltage_v", 32.0, batt_volt_min_param_desc);
        
        // Max voltage parameter
        auto batt_volt_max_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        batt_volt_max_param_desc.description = "This parameter changes the maximus battery voltage!";
        this->declare_parameter("battery_max_voltage_v", 42.0, batt_volt_max_param_desc);
    }

  private:
    void timer_callback() {

        // Create message
        auto message = std_msgs::msg::String();

        // Set messsage data
        message.data = "epic_node";

        // Log console and publish
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        name_publisher_->publish(message);
    }

    void battery_callback(std_msgs::msg::Float32::SharedPtr msg) {
        // Get data
        float voltage = msg->data;

        // Get parameter data
        float min_voltage = static_cast<float>(this->get_parameter("battery_min_voltage_v").as_double());
        float max_voltage = static_cast<float>(this->get_parameter("battery_max_voltage_v").as_double());

        // Convert to percentage
        float percentage = ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100.0f;

        // Publish and log
        auto message = std_msgs::msg::Float32();
        message.data = percentage;
        battery_percent_publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Voltage: %.2f V (%.2f %%)", voltage, percentage);
    }
    

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr name_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_percent_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_voltage_subscriber_;
    size_t count_;
};



int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EpicNode>());
    rclcpp::shutdown();
    return 0;
}