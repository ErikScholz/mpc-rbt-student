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
        // Save map and log
        map_ = response->map;
        RCLCPP_INFO(get_logger(), "Map received successfully! (%d x %d)", map_.info.width, map_.info.height);

        // Dilate map
        dilateMap();
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
    if (!path_.poses.empty()) smoothPath();

    // Save plan to response
    response->plan = path_;

    // Publish path and log
    path_pub_->publish(path_);
    RCLCPP_INFO(get_logger(), "Path planned and published with %zu waypoints.", path_.poses.size());
}



void PlanningNode::dilateMap() {
    // Create copy of the map
    nav_msgs::msg::OccupancyGrid dilated_map = map_;

    // Get map size
    int map_width = map_.info.width;
    int map_height = map_.info.height;

    // Settings
    int dilation_size = 7;


    // Go through the map
    for (int x = 0; x < map_width; x++)
    for (int y = 0; y < map_height; y++)
    {
        // Get index for current coord
        int curr_idx = y * map_width + x;

        // Not an obstacle
        if (map_.data[curr_idx] <= 50) continue;

        // Check neighbors
        for (int dx = -dilation_size; dx <= dilation_size; dx++)
        for (int dy = -dilation_size; dy <= dilation_size; dy++)
        {
            // Get neighbor coords
            int nb_x = x + dx;
            int nb_y = y + dy;

            // Dilate if inside map
            if ((nb_x >= 0 && nb_x < map_width) && (nb_y >= 0 && nb_y < map_height))
                dilated_map.data[nb_y * map_width + nb_x] = 100;
        }
    }


    // Update original map
    map_ = dilated_map;
    RCLCPP_INFO(get_logger(), "Map dilation finished.");
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
        return (x >= 0 && x < (int)map_width) && (y >= 0 && y < (int)map_height);
    };


    auto is_obstacle = [this, get_index](int x, int y) {
        return map_.data[get_index(x, y)] > 50;
    };


    auto world_to_map_coords = [this, map_origin, map_resolution, is_inside_map](double world_x, double world_y, int &map_x, int &map_y) {
        // Convert coords
        map_x = std::round((world_x - map_origin.position.x) / map_resolution);
        map_y = std::round((world_y - map_origin.position.y) / map_resolution);

        // Check that they are inside map
        return is_inside_map(map_x, map_y);
    };


    auto map_to_world_coords = [map_origin, map_resolution](int map_x, int map_y, double &world_x, double &world_y) {
        world_x = map_x * map_resolution + map_origin.position.x;
        world_y = map_y * map_resolution + map_origin.position.y;
    };


    auto reconstruct_path = [&](std::shared_ptr<Cell> current) {
        // Init header
        path_.poses.clear();
        path_.header.frame_id = map_.header.frame_id;
        path_.header.stamp = this->now();

        while (current)
        {
            // Init pose
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_.header;

            // Convert map coord to world coords
            double world_x, world_y;
            map_to_world_coords(current->x, current->y, world_x, world_y);

            // Update data
            pose.pose.position.x = world_x;
            pose.pose.position.y = world_y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            
            // Add point to path
            path_.poses.push_back(pose);
            current = current->parent;
        }

        // Reverse order
        std::reverse(path_.poses.begin(), path_.poses.end());
        RCLCPP_INFO(this->get_logger(), "Path reconstructed with %zu points", path_.poses.size());
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

    if (is_obstacle(start_x, start_y)) {
        RCLCPP_ERROR(get_logger(), "A* failed: Start is an obstacle!");
        return;
    }

    else if (is_obstacle(goal_x, goal_y)) {
        RCLCPP_ERROR(get_logger(), "A* failed: Goal is an obstacle!");
        return;
    }


    // Init cells
    Cell start_cell(start_x, start_y);
    Cell goal_cell(goal_x, goal_y);


    // Lists
    std::vector<std::shared_ptr<Cell>> open_list;
    std::vector<bool> closed_list(map_.info.height * map_.info.width, false);

    open_list.push_back(std::make_shared<Cell>(start_cell));


    // G-score map
    std::vector<float> g_scores(map_.info.width * map_.info.height, std::numeric_limits<float>::max());

    int start_idx = get_index(start_x, start_y);
    g_scores[start_idx] = 0.0f;


    // Max iterations
    int max_iterations = map_width * map_height;
    int iterations = 0;


    // A* loop
    while (!open_list.empty() && rclcpp::ok())
    {
        // Increment iteration counter
        iterations++;

        // Prevent infinite loop
        if (iterations > max_iterations) {
            RCLCPP_ERROR(get_logger(), "A* reached maximum iterations (%d). Planning aborted.", max_iterations);
            break; 
        }


        // Find cell with lowest cost in open list
        auto it = std::min_element(open_list.begin(), open_list.end(),
            [](const std::shared_ptr<Cell>& a, const std::shared_ptr<Cell>& b) {
                return a->f < b->f;
            }
        );


        // Get current lowest
        std::shared_ptr<Cell> current = *it;

        // Remove it from open list
        open_list.erase(it);

        // Add it to close list
        int curr_idx = get_index(current->x, current->y);
        closed_list[curr_idx] = true;


        // Check for reaching the goal
        if (current->x == goal_x && current->y == goal_y) {
            reconstruct_path(current);
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


            // We found better way to neighbor
            if (new_g < g_scores[nb_idx]) {

                // Update G-score map
                g_scores[nb_idx] = new_g;

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
    }
    

    // No valid path found
    RCLCPP_ERROR(get_logger(), "Unable to plan path.");
}



void PlanningNode::smoothPath() {
    // Nothing to smooth
    if (path_.poses.size() < 3) return;

    // Parameters
    float weight_data = 0.5f;
    float weight_smooth = 0.3f;
    float tolerance = 0.001f;
    int max_iterations = 500;

    // Create copy of path points
    std::vector<geometry_msgs::msg::PoseStamped> smoothed_path = path_.poses;

    float change = tolerance;
    int iterations = 0;


    // Smoothing loop
    while (change >= tolerance && iterations < max_iterations) {
        change = 0.0f;
        iterations++;

        // Go from second first to second to last points
        for (size_t i = 1; i < smoothed_path.size() - 1; i++) {
            // Store old values
            double old_x = smoothed_path[i].pose.position.x;
            double old_y = smoothed_path[i].pose.position.y;


            // Helper vars for clarity
            double sp_curr_x = smoothed_path[i].pose.position.x;
            double sp_prev_x = smoothed_path[i - 1].pose.position.x;
            double sp_next_x = smoothed_path[i + 1].pose.position.x;

            double sp_curr_y = smoothed_path[i].pose.position.y;
            double sp_prev_y = smoothed_path[i - 1].pose.position.y;
            double sp_next_y = smoothed_path[i + 1].pose.position.y;


            // Calculate stuff
            double update_x = weight_data * (path_.poses[i].pose.position.x - sp_curr_x) + weight_smooth * (sp_prev_x + sp_next_x - 2.0 * sp_curr_x);
            double update_y = weight_data * (path_.poses[i].pose.position.y - sp_curr_y) + weight_smooth * (sp_prev_y + sp_next_y - 2.0 * sp_curr_y);
            
            // Reflect update
            smoothed_path[i].pose.position.x += update_x;
            smoothed_path[i].pose.position.y += update_y;
            

            // Add absolute change to total change for this iteration
            change += std::abs(old_x - smoothed_path[i].pose.position.x);
            change += std::abs(old_y - smoothed_path[i].pose.position.y);
        }
    }

    // Update path
    path_.poses = smoothed_path;
    RCLCPP_INFO(get_logger(), "Path smoothed in %d iterations.", iterations);
}



Cell::Cell(int c, int r) {
    x = c;
    y = r;
    g = h = f = 0.0f;
    parent = nullptr;
}
