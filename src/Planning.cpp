#include "Planning.hpp"

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

        // Client for map
        map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");


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
        auto future = map_client_->async_send_request(request, std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1));
        

        RCLCPP_INFO(get_logger(), "Trying to fetch map...");
    }



void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    // Get response
    auto response = future.get();

    // Valid response
    if (response) {
        // Save map
        map_ = response->map;

        // Log
        RCLCPP_INFO(get_logger(), "Map received successfully! (%d x %d)", map_.info.width, map_.info.height);
    }

    // No/invalid response
    else {
        RCLCPP_ERROR(get_logger(), "Failed to fetch map from map_server.");
    }
}



void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {
    // Log
    RCLCPP_INFO(get_logger(),
        "Received planning request from [%.2f, %.2f] to [%.2f, %.2f]",
        request->start.pose.position.x, request->start.pose.position.y,
        request->goal.pose.position.x, request->goal.pose.position.y
    );


    // Start A* algo
    aStar(request->start, request->goal);


    // Smooth calculated path
    smoothPath();

    // Save plan to response
    response->plan = path_;

    // Publish path and log
    path_pub_->publish(path_);
    RCLCPP_INFO(get_logger(), "Path planned and published with %zu waypoints.", path_.poses.size());
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
    // Get map parameters
    auto map_resolution = map_.info.resolution;
    auto map_origin = map_.info.origin;

    int map_width = map_.info.width;
    int map_height = map_.info.height;


    // Helper functions
    auto get_index = [map_width](int x, int y) {
        return y * map_width + x;
    };


    auto is_inside_map = [map_width, map_height](int x, int y) {
        return (x > 0 || x < (int)map_width) && (y > 0 && y < (int)map_height);
    };


    auto is_obstacle = [this, get_index](int x, int y) {
        return map_.data[get_index(x, y)] > 50;
    };


    auto world_to_map_coords = [this, map_origin, map_resolution, is_inside_map](double world_x, double world_y, int &map_x, int &map_y) {
        // Convert coords
        map_x = std::round((world_x - map_origin.position.x) / map_resolution);
        map_y = std::round((world_y - map_origin.position.y) / map_resolution);

        // Check that they are inside map
        //return (map_x >= 0 && map_x < (int)map_.info.width && map_y >= 0 && map_y < (int)map_.info.height);
        return is_inside_map(map_x, map_y);
    };






    // Log start
    RCLCPP_INFO(get_logger(), "Starting A* ...");


    // Get start and goal coords and check if they are inside map
    int start_x, start_y, goal_x, goal_y;

    if (!world_to_map_coords(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
        RCLCPP_ERROR(get_logger(), "Start is outside the map!");
        return;
    }
    else if (!world_to_map_coords(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
        RCLCPP_ERROR(get_logger(), "Goal is outside the map!");
        return;
    }


    // Init cells
    Cell start_cell(start_x, start_y);
    Cell goal_cell(goal_x, goal_y);


    // Lists
    std::vector<std::shared_ptr<Cell>> open_list;
    std::vector<bool> closed_list(map_.info.height * map_.info.width, false);

    open_list.push_back(std::make_shared<Cell>(start_cell));


    // A* loop
    while (!open_list.empty() && rclcpp::ok())
    {
        // Find cell with lowest cost in open list
        auto it = std::min_element(open_list.begin(), open_list.end(),
            [](const std::shared_ptr<Cell>& a, const std::shared_ptr<Cell>& b) {
                return a->f < b->f;
            }
        );


        // Get current lowest
        std::shared_ptr<Cell> current = *it;


        // Move it from open to closed list
        open_list.erase(it);

        int curr_idx = get_index(current->x, current->y);
        closed_list[curr_idx] = true;


        // Check for reaching the goal
        if (current->x == goal_x && current->y == goal_y) {
            //reconstruct_path(current);
            return;
        }


        // Check neighbors
        for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++) {

            // Skip current
            if (dx == 0 && dy == 0) continue;

            // Get neighbor coords
            int nb_x = current->x + dx;
            int nb_y = current->y + dy;
            int nb_idx = get_index(nb_x, nb_y);


            // Already visited
            if (closed_list[nb_idx]) continue;

            // Map boundary check
            if (!is_inside_map(nb_x, nb_y)) continue;

            // Obstacle check
            if (is_obstacle(nb_x, nb_y)) continue;


            // Calculate stuff
            float move_cost = (dx != 0 && dy != 0) ? 1.414f : 1.0f;
            float new_g = current->g + move_cost;
            float new_h = std::hypot(nb_x - goal_x, nb_y - goal_y);


            // Update neighbor cell
            auto neighbor = std::make_shared<Cell>(nb_x, nb_y);
            neighbor->g = new_g;
            neighbor->h = new_h;
            neighbor->f = new_g + new_h;
            neighbor->parent = current;

            // Add to open list
            open_list.push_back(neighbor);
        }
    }
    

    // No valid path found
    RCLCPP_ERROR(get_logger(), "Unable to plan path.");
    

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
    x = c;
    y = r;
    g = h = f = 0.0f;
    parent = nullptr;
}
