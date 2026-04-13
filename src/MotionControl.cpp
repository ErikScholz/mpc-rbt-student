#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"

MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node") {
        // Collision status
        collision_detected = false;

        // Subscribers for odometry and laser scans
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry", 10,
            std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1)
        );

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/tiago_base/Hokuyo_URG_04LX_UG01", 10,
            std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1)
        );
        
        // Publisher for robot control
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Client for path planning
        plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("/plan_path");

        // Action server
        nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(this, "go_to_goal",
            std::bind(&MotionControlNode::navHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotionControlNode::navHandleCancel, this, std::placeholders::_1),
            std::bind(&MotionControlNode::navHandleAccepted, this, std::placeholders::_1)
        );


        RCLCPP_INFO(get_logger(), "Motion control node started.");


        // Connect to path planning service server
        while (rclcpp::ok() && !plan_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "Waiting for planning service...");
        }
    }



void MotionControlNode::checkCollision() {
    // No lidar data
    if (laser_scan_.ranges.empty()) return;


    // Parameters
    const float stop_dist = 0.35f;
    const int check_angle_deg = 40;


    // Helpers
    int center = laser_scan_.ranges.size() / 2;
    double desired_fov_rad = check_angle_deg * M_PI / 180.0; 
    int window = (desired_fov_rad / std::abs(laser_scan_.angle_increment)) / 2;

    //RCLCPP_INFO(get_logger(), "Checking window: %d rays around center %d", window, center);


    // Check for collisions
    collision_detected = false;
    for (int i = center - window; i < center + window; ++i) {

        // Out of bounds check
        if (i < 0 || i >= (int)laser_scan_.ranges.size()) continue;

        // Get current ray length
        float r = laser_scan_.ranges[i];

        // Obstacle in range
        if (std::isfinite(r) && r > 0.05f && r < stop_dist) {
            collision_detected = true;
            break;
        }
    }


    // No obstacles detected
    if (!collision_detected) return;


    // OBSTACLE ABORT ABORT
    RCLCPP_ERROR(get_logger(), "EMERGENCY STOP: Obstacle detected!");
    
    
    // Stop robot
    geometry_msgs::msg::Twist stop;
    twist_publisher_->publish(stop);


    // Abort navigation
    if (goal_handle_ && goal_handle_->is_active()) {
        auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
        goal_handle_->abort(result);
    }
}




void MotionControlNode::updateTwist() {
    // No path
    if (path_.poses.empty()) return;


    // Find path index closest to the robot
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < path_.poses.size(); ++i) {
        // Calculate distance
        double dist = std::hypot(
            path_.poses[i].pose.position.x - current_pose_.pose.position.x,
            path_.poses[i].pose.position.y - current_pose_.pose.position.y
        );

        // Update lowest
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }



    // Settings
    const size_t n_look_ahead = 3;
    const double max_lin_speed = 0.35;
    const double P_gain = 2.5;
    const double angle_threshold = 0.5;



    // Select target waypoint
    size_t target_idx = std::min(closest_idx + n_look_ahead, path_.poses.size() - 1);
    auto target_pose = path_.poses[target_idx].pose;


    // Calculate position difference (robot to waypoint)
    double dx = target_pose.position.x - current_pose_.pose.position.x;
    double dy = target_pose.position.y - current_pose_.pose.position.y;


    // Get robot orientation
    tf2::Quaternion q(
        current_pose_.pose.orientation.x,
        current_pose_.pose.orientation.y,
        current_pose_.pose.orientation.z,
        current_pose_.pose.orientation.w
    );

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);



    // Transform to local coords
    double local_x = dx * std::cos(yaw) + dy * std::sin(yaw);
    double local_y = -dx * std::sin(yaw) + dy * std::cos(yaw);


    // Calculate angle error
    double angle_error = std::atan2(local_y, local_x);


    // Calculate adaptive speed
    geometry_msgs::msg::Twist twist;

    // Angular
    twist.angular.z = P_gain * angle_error;

    // Linear
    if (std::abs(angle_error) > angle_threshold) twist.linear.x = 0.0;
    else {
        twist.linear.x = max_lin_speed * (1.0 - (std::abs(angle_error) / angle_threshold));
        if (twist.linear.x < 0.02) twist.linear.x = 0.0; 
    }


    // Cap speed
    if (twist.angular.z > 1.0) twist.angular.z = 1.0;
    if (twist.angular.z < -1.0) twist.angular.z = -1.0;


    // Send new twist data
    twist_publisher_->publish(twist);
}



rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
    // Log
    (void)uuid;
    RCLCPP_INFO(get_logger(), "Received goal request to x: %.2f, y: %.2f", goal->pose.pose.position.x, goal->pose.pose.position.y);
    
    // Accept and defer
    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
}



rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    // Log
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");

    // Accept cancel
    return rclcpp_action::CancelResponse::ACCEPT;
}



void MotionControlNode::navHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    // Save handle and goal
    goal_handle_ = goal_handle;
    auto goal = goal_handle->get_goal();
    goal_pose_ = goal->pose;


    // Clear old path
    path_.poses.clear();
    laser_scan_.ranges.clear();


    // Create /plan_path service request
    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->goal.header.frame_id = "map";
    request->start = current_pose_;
    request->goal = goal_pose_;
    request->tolerance = 0.1;


    // Send request
    auto future = plan_client_->async_send_request(request, std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Requesting path from planner...");
}



void MotionControlNode::execute() {
    // Run at X Hz
    rclcpp::Rate loop_rate(10.0);

    // Init
    auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();
    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();


    RCLCPP_INFO(get_logger(), "Executing navigation...");


    // Navigate
    while (rclcpp::ok()) {

        // Navigation cancel check
        if (goal_handle_->is_canceling()) {
            // Log
            RCLCPP_INFO(get_logger(), "Goal canceled.");

            // Cancel
            goal_handle_->canceled(result);

            // Stop robot
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            return;
        }


        // Calculate distance to goal
        double dist_to_goal = std::hypot(
            goal_pose_.pose.position.x - current_pose_.pose.position.x,
            goal_pose_.pose.position.y - current_pose_.pose.position.y
        );

        // Reached goal check
        if (dist_to_goal < 0.1) {
            // Log
            RCLCPP_INFO(get_logger(), "Goal reached!");

            // Success
            goal_handle_->succeed(result);

            // Stop robot
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            return;
        }


        // Send feedback
        feedback->distance_remaining = dist_to_goal;
        goal_handle_->publish_feedback(feedback);

        loop_rate.sleep();
    }
}



void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
    // Get response
    auto response = future.get();
    
    // Valid response
    if (response && response->plan.poses.size() > 0) {
        // Save path
        path_ = response->plan; 

        // Log
        RCLCPP_INFO(get_logger(), "Path received. Starting execution...");
        
        // Execute
        goal_handle_->execute();
        std::thread(&MotionControlNode::execute, this).detach();
    }
    

    // Invalid response
    else {
        // Log
        RCLCPP_ERROR(get_logger(), "Failed to plan path. Aborting goal.");

        // Abort
        auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
        goal_handle_->abort(result);
    }
}



void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    // Store current position
    current_pose_.header = msg.header;
    current_pose_.pose = msg.pose.pose;

    // If navigatiing
    if (goal_handle_ && goal_handle_->is_active() && !path_.poses.empty()) {
        checkCollision();

        // Only update if no collisions detected
        if (!collision_detected) updateTwist();
    }
}



void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    // Store data
    laser_scan_ = msg;
}
