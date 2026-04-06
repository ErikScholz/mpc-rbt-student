#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"

MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node") {

        // Subscribers for odometry and laser scans
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
            std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1)
        );

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10,
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
    // add code here

    // ********
    // * Help *
    // ********
    /*
    if (laser_scan_.ranges[i] < thresh) {
        geometry_msgs::msg::Twist stop;
        twist_publisher_->publish(stop);
    }
    */
}



void MotionControlNode::updateTwist() {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    geometry_msgs::msg::Twist twist;
    twist.angular.z = P * xte;
    twist.linear.x = v_max;

    twist_publisher_->publish(twist);
    */
}



rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
    // Log
    (void)uuid;
    RCLCPP_INFO(get_logger(), "Received goal request to x: %.2f, y: %.2f", goal->pose.pose.position.x, goal->pose.pose.position.y);
    
    // Accept
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}



rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    

    // ********
    // * Help *
    // ********
    /*
    (void)goal_handle;
    ...
    return ...;
    */
}



void MotionControlNode::navHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    // Save handle and goal
    goal_handle_ = goal_handle;
    auto goal = goal_handle->get_goal();
    goal_pose_ = goal->pose;


    // Create /plan_path service request
    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
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
        //std::thread{std::bind(&MotionControlNode::execute, this)}.detach();
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
    if (goal_handle_ && goal_handle_->is_active()) {
        checkCollision();
        updateTwist();
    }
}



void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    // add code here
}
