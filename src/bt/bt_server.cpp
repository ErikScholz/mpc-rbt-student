#include "behaviortree_ros2/tree_execution_server.hpp"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"


class BTServer : public BT::TreeExecutionServer {
public:
    BTServer(const rclcpp::NodeOptions& options) : TreeExecutionServer(options) {}

    void onTreeCreated(BT::Tree& tree) override {
        static BT::StdCoutLogger logger_count(tree);
    }
};

int main(int argc, char** argv)
{
    // Init
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    // Create server
    auto server = std::make_shared<BTServer>(options);

    // Create MultiThreadedExecutor and spin (me right round baby right round...)
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(server->node());
    exec.spin();

    
    rclcpp::shutdown();
    return 0;
}
