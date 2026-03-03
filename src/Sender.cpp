#include <mpc-rbt-solution/Sender.hpp>

void Sender::Node::run()
{
  while (errno != EINTR) {
    if ((std::chrono::steady_clock::now() - timer_tick) < timer_period) continue;
    timer_tick = std::chrono::steady_clock::now();

    callback();
  }
}

void Sender::Node::onDataTimerTick()
{
  // Update data
  data.frame = "baller_frame";
  data.x += 4.2;
  data.y += 6.7;
  data.z += 6.9;

  // Update timestamp
  data.timestamp =
    static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());

  // Prerare data frame
  Socket::IPFrame frame{
    .port = config.remotePort,
    .address = config.remoteAddress,
  };

  // Serialize
  Utils::Message::serialize(frame, data);

  // Send
  if(send(frame)){
    RCLCPP_INFO(logger, "Sending to %s:%d | \nstamp: %ld\nframe: %s\nx: %.2f\ny: %.2f\nz: %.2f\n", 
                  frame.address.c_str(), frame.port, data.timestamp, data.frame.c_str(), data.x, data.y, data.z);
  }
  else{
    RCLCPP_WARN(logger, "Failed to send to %s", frame.address.c_str());
  }
}
