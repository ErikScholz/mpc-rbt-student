#include <mpc-rbt-solution/Receiver.hpp>

void Receiver::Node::run()
{
  while (errno != EINTR) {
    RCLCPP_INFO(logger, "Waiting for data ...");
    Socket::IPFrame frame{};

    if (receive(frame)) {
      RCLCPP_INFO(logger, "Received data from host: '%s:%d'", frame.address.c_str(), frame.port);

      callback(frame);

    } else {
      RCLCPP_WARN(logger, "Failed to receive data.");
    }
  }
}

void Receiver::Node::onDataReceived(const Socket::IPFrame & frame)
{
  if (Utils::Message::deserialize(frame, data)) {
    RCLCPP_INFO(logger, "\nstamp: %ld\nframe: %s\nx: %.2f\ny: %.2f\nz: %.2f\n", 
                  data.timestamp, data.frame.c_str(), data.x, data.y, data.z);
  }
  else {
    RCLCPP_WARN(logger, "Failed to deserialize from %s:%d", frame.address.c_str(), frame.port);
  }  
}
