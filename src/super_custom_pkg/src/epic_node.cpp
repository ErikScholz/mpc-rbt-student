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

        // Battery percentage publisher
        battery_percent_publisher_ = this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);

        // Battery voltage subscriber
        battery_voltage_subscriber_ = this->create_subscription<std_msgs::msg::Float32>("battery_voltage", 10,
            std::bind(&EpicNode::battery_callback, this, std::placeholders::_1));

        // Creates timer that publishes node_name every X ms
        timer_ = this->create_wall_timer(500ms, std::bind(&EpicNode::timer_callback, this));
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

        // Convert to percentage
        float percentage = ((voltage - 32.0f) / (42.0f - 32.0f)) * 100.0f;

        // Publish and log
        auto message = std_msgs::msg::Float32();
        message.data = percentage;
        battery_percent_publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Voltage: '%.2f'V -> Percent: '%.2f' %%", voltage, percentage);
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