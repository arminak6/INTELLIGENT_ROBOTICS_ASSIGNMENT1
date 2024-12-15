#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/PoseArray.h"
#include "assignment_1/ApriltagsIdS.h"

// Publisher for sending apriltag IDs to Node B
ros::Publisher ids_publisher;

// Callback to handle feedback from Node B
void feedbackCallback(const std_msgs::String::ConstPtr& msg) {
    std::string feedback = msg->data;

    // Parse the feedback to print meaningful messages
    if (feedback.find("moving") != std::string::npos) {
        ROS_INFO("[Node A] Status: The robot is moving.");
    } else if (feedback.find("scanning") != std::string::npos) {
        ROS_INFO("[Node A] Status: The robot is scanning.");
    } else if (feedback.find("found Apriltags") != std::string::npos) {
        ROS_INFO("[Node A] Status: %s", feedback.c_str()); 
    } else if (feedback.find("detection is finished") != std::string::npos) {
        ROS_INFO("[Node A] Status: Detection is finished.");
    } else {
        ROS_INFO("[Node A] Status: %s", feedback.c_str()); 
    }
}

void cubePositionsCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    ROS_INFO("[Node A] Received cube positions from Node B.");
    int cube_count = 0;
    for (const auto& pose : msg->poses) {
        ROS_INFO("Cube %d position: [x: %f, y: %f, z: %f]", ++cube_count, pose.position.x, pose.position.y, pose.position.z);
    }
    ROS_INFO("[Node A] Detection and cube processing are finished.");
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
    ids_publisher = nh.advertise<std_msgs::Int32MultiArray>("apriltag_ids_topic", 10);

    // Subscribe to feedback from Node B
    ros::Subscriber feedback_subscriber = nh.subscribe("node_b_feedback", 10, feedbackCallback);

    // Subscribe to cube positions from Node B
    ros::Subscriber cube_positions_subscriber = nh.subscribe("cube_positions_topic", 10, cubePositionsCallback);

    // Prepare the service request and response
    assignment_1::ApriltagsIdS srv;
    srv.request.ready = true;

    // Call the service to get the apriltag IDs
    if (client.call(srv)) {
        ROS_INFO("[Node A] Received IDs:");

        // Prepare the message to send IDs to Node B
        std_msgs::Int32MultiArray ids_msg;
        for (int id : srv.response.ids) {
            ROS_INFO("%d", id);
            ids_msg.data.push_back(id);
        }

        // Publish the IDs
        ros::Rate loop_rate(1); 
        for (int i = 0; i < 5; ++i) { 
            ids_publisher.publish(ids_msg);
            ROS_INFO("[Node A] Published IDs to Node B (attempt %d).", i + 1);
            loop_rate.sleep();
        }

        // Inform the user that Node A is waiting for feedback and cube positions
        ROS_INFO("[Node A] Waiting for feedback and cube positions from Node B...");
    } else {
        ROS_ERROR("[Node A] Failed to call service apriltag_ids_srv.");
        return 1;
    }

    ros::spin();

    return 0;
}

