#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <vector>
#include <algorithm>

// Array to store tags received from Node A
std::vector<int> received_tags;

// Callback to process IDs from Node A
void apriltagCallback(const std_msgs::Int32MultiArray::ConstPtr& ids_msg) {
    ROS_INFO("[Node B] Received Apriltag IDs from Node A:");
    received_tags.clear(); // Clear old data before storing new tags
    for (int id : ids_msg->data) {
        ROS_INFO("Apriltag ID: %d", id);
        received_tags.push_back(id);
    }
    ROS_INFO("[Node B] Stored %lu tags from Node A.", received_tags.size());
}

// Callback to process local AprilTag detections
void localTagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& detections_msg) {
    ROS_INFO("[Node B] Detected Apriltags locally:");
    std::vector<int> local_tags;

    for (const auto& detection : detections_msg->detections) {
        int id = detection.id[0];
        ROS_INFO("Local Apriltag ID: %d", id);
        local_tags.push_back(id);
    }

    // Compare local tags with received tags
    for (int local_id : local_tags) {
        if (std::find(received_tags.begin(), received_tags.end(), local_id) != received_tags.end()) {
            ROS_INFO("[Node B] Local tag ID %d matches a tag from Node A.", local_id);
        } else {
            ROS_INFO("[Node B] Local tag ID %d does not match any tag from Node A.", local_id);
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_b");
    ros::NodeHandle nh;

    // Subscribe to Node A's topic
    ros::Subscriber apriltag_subscriber = nh.subscribe("apriltag_ids_topic", 10, apriltagCallback);

    // Subscribe to local detections
    ros::Subscriber local_tag_subscriber = nh.subscribe("tag_detections", 10, localTagCallback);

    ros::spin();
    return 0;
}

