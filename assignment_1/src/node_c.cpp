#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <cmath>
#include <numeric>

class NodeC {
private:
    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_;
    ros::Publisher cmd_vel_pub_;

    // Control parameters
    double desired_wall_distance_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    double kp_distance_;
    double kp_angle_;

    // Safety parameters
    double min_front_distance_;

public:
    NodeC() : nh_("~") {
        // Initialize parameters with default values
        nh_.param("desired_wall_distance", desired_wall_distance_, 0.5); //Target distance to maintain from walls
        nh_.param("max_linear_velocity", max_linear_velocity_, 0.3); // Maximum forward speed
        nh_.param("max_angular_velocity", max_angular_velocity_, 0.5); // Maximum turning rate
        nh_.param("kp_distance", kp_distance_, 1.0); //  Proportional gain for distance control
        nh_.param("kp_angle", kp_angle_, 1.0); // Proportional gain for angular control
        nh_.param("min_front_distance", min_front_distance_, 0.5); // Minimum safe distance to obstacles ahead

        // Set up ROS communication
        laser_sub_ = nh_.subscribe("/scan", 1, &NodeC::laserCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 10);
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        // Extract relevant ranges from laser scan
        std::vector<float> right_side, left_side, front;
        int num_samples = 30;
        
        // Get indices for different regions
        int total_samples = scan_msg->ranges.size();
        int front_start = total_samples / 2 - num_samples / 2;
        int right_start = 0;
        int left_start = total_samples - num_samples;
        
        // Extract ranges for each region
        for (int i = 0; i < num_samples; i++) {
            if (std::isfinite(scan_msg->ranges[right_start + i])) {
                right_side.push_back(scan_msg->ranges[right_start + i]);
            }
            if (std::isfinite(scan_msg->ranges[left_start + i])) {
                left_side.push_back(scan_msg->ranges[left_start + i]);
            }
            if (std::isfinite(scan_msg->ranges[front_start + i])) {
                front.push_back(scan_msg->ranges[front_start + i]);
            }
        }
        
        // Calculate average distances
        double right_dist = calculateAverage(right_side);
        double left_dist = calculateAverage(left_side);
        double front_dist = calculateAverage(front);
        
        // Debug information
        ROS_INFO_THROTTLE(1.0, "Distances - Right: %.2f, Left: %.2f, Front: %.2f", 
                         right_dist, left_dist, front_dist);
        
        // Generate control commands
        geometry_msgs::Twist cmd_vel;
        
        // Make sure we have valid measurements
        if (right_dist > 0.0 && left_dist > 0.0 && front_dist > 0.0) {
            // Calculate corridor center error
            double corridor_center = (right_dist + left_dist) / 2.0;
            double lateral_error = right_dist - corridor_center;
            
            // Calculate angular velocity based on lateral error
            double angular_velocity = -kp_angle_ * lateral_error;
            
            // Apply velocity limits
            angular_velocity = std::max(-max_angular_velocity_,
                                     std::min(max_angular_velocity_, angular_velocity));
            
            if (front_dist < min_front_distance_) {
                // Obstacle ahead - stop and turn
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = (right_dist > left_dist) ? max_angular_velocity_ : -max_angular_velocity_;
                ROS_WARN_THROTTLE(1.0, "Obstacle detected! Turning...");
            } else {
                // Normal corridor navigation
                // Calculate forward velocity based on front distance and angular velocity
                double velocity_factor = 1.0 - std::abs(angular_velocity) / max_angular_velocity_;
                cmd_vel.linear.x = max_linear_velocity_ * velocity_factor;
                cmd_vel.angular.z = angular_velocity;
                
                ROS_INFO_THROTTLE(1.0, "Command - Linear: %.2f, Angular: %.2f", 
                                cmd_vel.linear.x, cmd_vel.angular.z);
            }
        } else {
            ROS_WARN_THROTTLE(1.0, "Invalid measurements detected!");
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
        }
        
        // Publish control commands
        cmd_vel_pub_.publish(cmd_vel);
    }

private:
    double calculateAverage(const std::vector<float>& values) {
        if (values.empty()) return 0.0;
        return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_c");
    NodeC navigator;
    ros::spin();
    return 0;
}