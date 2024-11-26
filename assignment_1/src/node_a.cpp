#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"
#include "assignment_1/ApriltagsIdS.h"

// Publisher for sending apriltag IDs to Node B
ros::Publisher ids_publisher;

// Callback to handle feedback from Node B
void feedbackCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("[Node A] Feedback from Node B: %s", msg->data.c_str());
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
        ros::Rate loop_rate(1); // 1 Hz
        for (int i = 0; i < 5; ++i) { // Publish multiple times to ensure Node B gets the message
            ids_publisher.publish(ids_msg);
            ROS_INFO("[Node A] Published IDs to Node B (attempt %d).", i + 1);
            loop_rate.sleep();
        }
    } else {
        ROS_ERROR("[Node A] Failed to call service apriltag_ids_srv.");
        return 1;
    }

    // Spin to handle callbacks (e.g., feedback from Node B)
    ros::spin();

    return 0;
}

