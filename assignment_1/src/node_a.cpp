#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/PoseArray.h"
#include "assignment_1/ApriltagsIdS.h"

// Publisher for sending apriltag IDs to Node B
ros::Publisher ids_publisher;

// Callback to handle feedback from Node B
void feedbackCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("[Node A] Feedback from Node B: %s", msg->data.c_str());
}

// Callback to receive the final list of cube positions from Node B
void cubePositionsCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    ROS_INFO("[Node A] Received final list of cube positions:");
    for (const auto& pose : msg->poses) {
        ROS_INFO("[Node A] Cube at: [x: %f, y: %f, z: %f]",
                 pose.position.x, pose.position.y, pose.position.z);
    }
}

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "node_a");
    ros::NodeHandle nh;

    // Wait for the apriltag service to be available
    ROS_INFO("[Node A] Waiting for service apriltag_ids_srv...");
    if (!ros::service::waitForService("apriltag_ids_srv", ros::Duration(10.0))) {
        ROS_ERROR("[Node A] Service apriltag_ids_srv not available.");
        return 1;
    }

    // Create a service client to request apriltag IDs
    ros::ServiceClient client = nh.serviceClient<assignment_1::ApriltagsIdS>("apriltag_ids_srv");

    // Initialize the publisher to send IDs to Node B
    ids_publisher = nh.advertise<std_msgs::Int32MultiArray>("/apriltag_ids_topic", 10);

    // Subscribe to feedback from Node B
    ros::Subscriber feedback_subscriber = nh.subscribe("/node_b_feedback", 10, feedbackCallback);

    // Subscribe to receive the final list of cube positions from Node B
    ros::Subscriber positions_subscriber = nh.subscribe("/final_cube_positions", 10, cubePositionsCallback);

    // Request IDs from the apriltag service
    assignment_1::ApriltagsIdS srv;
    if (client.call(srv)) {
        ROS_INFO("[Node A] Received IDs from the server.");
        std_msgs::Int32MultiArray id_msg;
        id_msg.data = srv.response.ids;
        ids_publisher.publish(id_msg);
        ROS_INFO("[Node A] Published IDs to Node B.");
    } else {
        ROS_ERROR("[Node A] Failed to call service apriltag_ids_srv.");
        return 1;
    }

    // Spin to process callbacks
    ros::spin();

    return 0;
}

