#include "ros/ros.h"
#include "std_msgs/String.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sstream>
#include <vector>
#include <algorithm> // For std::find

// Type alias for convenience
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NodeB {
public:
    NodeB(ros::NodeHandle& nh) : action_client("move_base", true), tf_listener(tf_buffer) {
        // Subscriber for AprilTag detections
        tag_detections_subscriber = nh.subscribe("/tag_detections", 10, &NodeB::tagDetectionsCallback, this);

        // Subscriber for target IDs from Node A
        target_ids_subscriber = nh.subscribe("/node_b/target_ids", 10, &NodeB::targetIdsCallback, this);

        // Publisher for transformed poses back to Node A
        transformed_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/node_b/transformed_poses", 10);

        // Publisher for feedback to Node A
        feedback_publisher = nh.advertise<std_msgs::String>("/node_b/feedback", 10);

        ROS_INFO("Waiting for move_base action server...");
        action_client.waitForServer();
        ROS_INFO("Node B initialized and move_base is ready.");

        // Define navigation goals (hardcoded example points)
        defineGoals();
        startNavigation();
    }

private:
    ros::Subscriber tag_detections_subscriber;  // Subscriber for AprilTag detections
    ros::Subscriber target_ids_subscriber;     // Subscriber for target IDs
    ros::Publisher transformed_pose_publisher; // Publisher for transformed poses
    ros::Publisher feedback_publisher;         // Publisher for feedback
    tf2_ros::Buffer tf_buffer;                 // Buffer for TF transforms
    tf2_ros::TransformListener tf_listener;    // Listener for TF transforms
    MoveBaseClient action_client;              // Action client for move_base navigation

    std::vector<int> target_ids;  // List of target AprilTag IDs
    std::vector<geometry_msgs::PoseStamped> goals;  // List of navigation goals
    size_t current_goal_index = 0;  // Track the current goal index

    // Helper function to transform poses using TF2
	geometry_msgs::PoseStamped transformToMapFrame(const geometry_msgs::PoseStamped& input_pose) {
		geometry_msgs::PoseStamped output_pose;
		try {
		    geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(
		        "map", input_pose.header.frame_id, ros::Time(0));
		    tf2::doTransform(input_pose, output_pose, transform);
		} catch (tf2::TransformException& ex) {
		    ROS_WARN("Transform error to map frame: %s", ex.what());
		}
		return output_pose;
	}
    geometry_msgs::PoseStamped transformPose(const geometry_msgs::PoseStamped& input_pose, const std::string& target_frame) {
        geometry_msgs::PoseStamped output_pose;
        try {
            geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(
                target_frame, input_pose.header.frame_id, ros::Time(0));
            tf2::doTransform(input_pose, output_pose, transform);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Transform error: %s", ex.what());
        }
        return output_pose;
    }

    void defineGoals() {
        geometry_msgs::PoseStamped goal1, goal2;
        goal1.header.frame_id = "map";
        goal1.pose.position.x = 1.0;
        goal1.pose.position.y = 2.0;
        goal1.pose.orientation.w = 1.0;

        goal2.header.frame_id = "map";
        goal2.pose.position.x = 2.0;
        goal2.pose.position.y = 3.0;
        goal2.pose.orientation.w = 1.0;

        goals.push_back(goal1);
        goals.push_back(goal2);

        ROS_INFO("Navigation goals defined.");
    }

    void startNavigation() {
        if (goals.empty()) {
            ROS_WARN("No navigation goals defined. Exiting navigation.");
            return;
        }
        sendNextGoal();
    }

    void sendNextGoal() {
        if (current_goal_index >= goals.size()) {
            ROS_INFO("All navigation goals reached!");
            publishFeedback("Task completed! All goals reached.");
            return;
        }

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = goals[current_goal_index];

        ROS_INFO("Sending goal %lu: (x: %f, y: %f)", current_goal_index,
                 goals[current_goal_index].pose.position.x,
                 goals[current_goal_index].pose.position.y);

        action_client.sendGoal(goal,
                               boost::bind(&NodeB::doneCallback, this, _1, _2),
                               boost::bind(&NodeB::activeCallback, this),
                               boost::bind(&NodeB::feedbackCallback, this, _1));
    }

    void resumeNavigation() {
        ROS_INFO("Resuming navigation...");
        current_goal_index++;
        sendNextGoal();
    }

    void investigateAprilTag(const geometry_msgs::Pose& tag_pose) {
        ROS_INFO("Investigating AprilTag at position (x: %f, y: %f, z: %f)",
                 tag_pose.position.x, tag_pose.position.y, tag_pose.position.z);

        publishFeedback("Investigating detected tag...");
        resumeNavigation();
    }

    void publishFeedback(const std::string& message) {
        std_msgs::String feedback_msg;
        feedback_msg.data = message;
        feedback_publisher.publish(feedback_msg);
    }

    void targetIdsCallback(const std_msgs::String::ConstPtr& msg) {
        target_ids.clear();
        std::istringstream ss(msg->data);
        std::string id_str;

        while (std::getline(ss, id_str, ',')) {
            target_ids.push_back(std::stoi(id_str));
        }

        ROS_INFO("Received target IDs: %s", msg->data.c_str());
        publishFeedback("Target IDs received: " + msg->data);
    }

    void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
        if (msg->detections.empty()) {
            ROS_INFO("No AprilTags detected.");
            return;
        }

        for (const auto& detection : msg->detections) {
            int tag_id = detection.id[0];

            if (std::find(target_ids.begin(), target_ids.end(), tag_id) == target_ids.end()) {
                ROS_INFO("Ignoring tag ID: %d (not in target list)", tag_id);
                continue;
            }

            ROS_INFO("Processing tag ID: %d", tag_id);

            geometry_msgs::PoseStamped tag_pose_in_camera;
            tag_pose_in_camera.header = detection.pose.header;
            tag_pose_in_camera.pose = detection.pose.pose.pose;

            geometry_msgs::PoseStamped tag_pose_in_base = transformPose(tag_pose_in_camera, "base_link");

            ROS_INFO_STREAM("Original Pose:\n" << tag_pose_in_camera);
            ROS_INFO_STREAM("Transformed Pose:\n" << tag_pose_in_base);

            transformed_pose_publisher.publish(tag_pose_in_base);

            publishFeedback("Processed AprilTag ID: " + std::to_string(tag_id));
            action_client.cancelGoal();
            investigateAprilTag(tag_pose_in_base.pose);
            return;
        }
    }

    void doneCallback(const actionlib::SimpleClientGoalState& state,
                      const move_base_msgs::MoveBaseResult::ConstPtr& result) {
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Reached goal successfully.");
            current_goal_index++;
            sendNextGoal();
        } else {
            ROS_WARN("Failed to reach goal. Retrying...");
            sendNextGoal();
        }
    }

    void activeCallback() {
        ROS_INFO("Navigation goal is now active.");
    }

    void feedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback) {
        ROS_INFO("Navigation feedback: Current position (x: %f, y: %f)",
                 feedback->base_position.pose.position.x,
                 feedback->base_position.pose.position.y);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_b");
    ros::NodeHandle nh;

    NodeB node_b(nh);

    ros::spin();
    return 0;
}

