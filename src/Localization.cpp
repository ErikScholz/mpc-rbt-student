#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "Localization.hpp"


LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"), 
    last_time_(this->get_clock()->now()) {
    

    // Odometry message initialization
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";

    odometry_.pose.pose.position.x = -0.5;
    odometry_.pose.pose.position.y = 0.0;
    odometry_.pose.pose.position.z = 0.0956;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0.00187365);
    odometry_.pose.pose.orientation = tf2::toMsg(q);



    // Subscriber for joint_states
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1));


    // Publisher for odometry
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);


    // Transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


    // Init status
    first_run_ = true;


    RCLCPP_INFO(get_logger(), "Localization node started.");
}



void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    // Get current time
    auto current_time = this->get_clock()->now();

    // Simulation not running yet
    if (current_time.seconds() <= 0.0) return;


    // Calculate time diff
    double dt = (current_time - last_time_).seconds();
    
    // Guard
    if (dt <= 0.0 || dt > 1.0) {
        last_time_ = current_time;
        return;
    }


    // First init
    if (first_run_) {
        last_time_ = current_time;
        first_run_ = false;
        return;
    }


    // Update odometry
    updateOdometry(msg.velocity[1], msg.velocity[0], dt);


    // Publish
    last_time_ = current_time;
    publishOdometry();
    publishTransform();
}


void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    // Convert from rad/s to m/s and calculate speed
    double v_left = left_wheel_vel * robot_config::WHEEL_RADIUS;
    double v_right = right_wheel_vel * robot_config::WHEEL_RADIUS;
    double L = 2.0 * robot_config::HALF_DISTANCE_BETWEEN_WHEELS;

    double linear = (v_right + v_left) / 2.0;
    double angular = (v_right - v_left) / L;


    // Save speeds
    odometry_.twist.twist.linear.x = linear;
    odometry_.twist.twist.angular.z = angular;


    // Get current orientation
    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);

    
    // Euler integration
    odometry_.pose.pose.position.x += linear * cos(theta) * dt;
    odometry_.pose.pose.position.y += linear * sin(theta) * dt;
    theta += angular * dt;


    // Normalize angle
    theta = std::atan2(std::sin(theta), std::cos(theta));


    // Convert back to quaternion (bleeeh)
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    odometry_.pose.pose.orientation = tf2::toMsg(q);
}


void LocalizationNode::publishOdometry() {
    // Set timestamp
    odometry_.header.stamp = this->get_clock()->now();

    // Publish
    odometry_publisher_->publish(odometry_);
}


void LocalizationNode::publishTransform() {
    // Create msg
    geometry_msgs::msg::TransformStamped tf;

    // Set timestamp and frame names
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_link";

    
    // Position to transform
    tf.transform.translation.x = odometry_.pose.pose.position.x;
    tf.transform.translation.y = odometry_.pose.pose.position.y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odometry_.pose.pose.orientation;


    // Publish
    tf_broadcaster_->sendTransform(tf);
}
