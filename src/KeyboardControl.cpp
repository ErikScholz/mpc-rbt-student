#include <chrono>
#include <functional>
#include <unistd.h>
#include <fcntl.h>

#include "KeyboardControl.hpp"

using namespace std::chrono_literals;

KeyboardControlNode::KeyboardControlNode(): rclcpp::Node("keyboard_control_node")
{
    // Parameters for speed change
    this->declare_parameter("linear_speed", 0.5);
    this->declare_parameter("angular_speed", 0.75);
    

    // Publisher for robot control
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    

    // Polling timer
    timer_ = this->create_wall_timer(10ms, std::bind(&KeyboardControlNode::timerCallback, this));


    // Set terminal settings to non-blocking
    tcgetattr(STDIN_FILENO, &old_termios_);
    struct termios new_termios = old_termios_;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);


    RCLCPP_INFO(this->get_logger(), "Use Arrow Keys to control the robot. Press 'ctrl+c' to quit.");
    RCLCPP_INFO(get_logger(), "Keyboard Control node started.");
}

KeyboardControlNode::~KeyboardControlNode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
}

void KeyboardControlNode::timerCallback() {
    
    // Message
    geometry_msgs::msg::Twist twist{};


    // Get speed parameters
    double l_speed = this->get_parameter("linear_speed").as_double();
    double a_speed = this->get_parameter("angular_speed").as_double();
    char c;


    // Non-blocking read
    fd_set readfds;
    struct timeval timeout;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int retval = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);


    // Invalid data guard
    if (retval <= 0 || !FD_ISSET(STDIN_FILENO, &readfds))
        return;


    // Empty read guard
    if (read(STDIN_FILENO, &c, 1) != 1) 
        return;


    // Process input
    if (c == '\033') {
        char seq[2];

        // If rest of the sequence is arrow format, control
        if (read(STDIN_FILENO, &seq, 2) == 2 && seq[0] == '[') {
            switch (seq[1]) {
                case 'A': twist.linear.x = l_speed; break;  // Up
                case 'B': twist.linear.x = -l_speed; break; // Down
                case 'C': twist.angular.z = -a_speed; break; // Right
                case 'D': twist.angular.z = a_speed; break; // Left
            }
        }
    }


    // Publish
    twist_publisher_->publish(twist);
}
