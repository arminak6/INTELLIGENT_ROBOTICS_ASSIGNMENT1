#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"
#include "assignment_1/ApriltagsIdS.h" 


ros::Publisher ids_publisher;


void feedbackCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("[Node A] Feedback from Node B: %s", msg->data.c_str());
}

int main(int argc, char **argv) {

    // Initialize the ROS node
    ros::init(argc, argv, "node_a");
    ros::NodeHandle nh;

    // Wait for the service to be available
    ros::service::waitForService("apriltag_ids_srv");

    // Create a service client
    ros::ServiceClient client = nh.serviceClient<assignment_1::ApriltagsIdS>("apriltag_ids_srv");

    // Prepare the service request and response
    assignment_1::ApriltagsIdS srv;
    srv.request.ready = true;

    // Call the service
    if (client.call(srv)) {
        // Log the response
        ROS_INFO("Received IDs:");
        for (int id : srv.response.ids) {
            ROS_INFO("%d", id);
        }
    } else {
        ROS_ERROR("Failed to call service apriltag_ids_srv");
        return 1;
    }

    return 0;
}

