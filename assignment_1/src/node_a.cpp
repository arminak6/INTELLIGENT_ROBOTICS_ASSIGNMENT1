#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"
#include "assignment_1/ApriltagsIdS.h" 


ros::Publisher ids_publisher;


void feedbackCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("[Node A] Feedback from Node B: %s", msg->data.c_str());
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "node_a");
    ros::NodeHandle nh;


    ros::ServiceClient client = nh.serviceClient<assignment_1::ApriltagsIdS>("/apriltags_ids_srv");


    ids_publisher = nh.advertise<std_msgs::Int32MultiArray>("apriltags_ids", 10);


    ros::Subscriber feedback_subscriber = nh.subscribe("node_b_feedback", 10, feedbackCallback);


    ROS_INFO("[Node A] Requesting IDs from server...");
    assignment_1::ApriltagsIdS srv; 
    if (client.call(srv)) {
        ROS_INFO("[Node A] Received IDs:");
        std_msgs::Int32MultiArray ids_msg;

        for (const auto& id : srv.response.ids) {
            ROS_INFO("  - %d", id);
            ids_msg.data.push_back(id);
        }


        ROS_INFO("[Node A] Sending IDs to Node B...");
        ids_publisher.publish(ids_msg);
    } else {
        ROS_ERROR("[Node A] Failed to call service /apriltags_ids_srv");
        return 1;
    }

    ros::spin();

    return 0;
}

