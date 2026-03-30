#include "Planning.hpp"

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

        // Client for map
        map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/get_map");


        // Service for path
        plan_service_ = this->create_service<nav_msgs::srv::GetPlan>("plan_path",
            std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2)
        );
        

        // Publisher for path
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);


        RCLCPP_INFO(get_logger(), "Planning node started.");


        // Connect to map server
        while (rclcpp::ok() && !map_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "Waiting for map_server service...");
        }

        if (!rclcpp::ok()) return;


        // Request map
        auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
        map_client_->async_send_request(request, std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1));
        

        RCLCPP_INFO(get_logger(), "Trying to fetch map...");
    }

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    auto response = future.get();
    if (response) {
        ...
    }
    */
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    aStar(request->start, request->goal);
    smoothPath();

    path_pub_->publish(path_);
    */
}

void PlanningNode::dilateMap() {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;
    ... processing ...
    map_ = dilatedMap;
    */
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    Cell cStart(...x-map..., ...y-map...);
    Cell cGoal(...x-map..., ...y-map...);

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(map_.info.height * map_.info.width, false);

    openList.push_back(std::make_shared<Cell>(cStart));

    while(!openList.empty() && rclcpp::ok()) {
        ...
    }

    RCLCPP_ERROR(get_logger(), "Unable to plan path.");
    */
}

void PlanningNode::smoothPath() {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;
    ... processing ...
    path_.poses = newPath;
    */
}

Cell::Cell(int c, int r) {
    // add code here
}
