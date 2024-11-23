#include "ros/ros.h"
#include "std_msgs/String.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
        generateDynamicGoal();
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
    std::vector<geometry_msgs::PoseStamped> detected_cubes; // List of detected cubes in map frame
    size_t current_goal_index = 0;  // Track the current goal index

    // Transform a pose to a target frame
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

    // Transform a pose to the map frame
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

	geometry_msgs::PoseStamped generateDynamicGoal() {
		geometry_msgs::PoseStamped new_goal;
		new_goal.header.frame_id = "map";
		new_goal.header.stamp = ros::Time::now();

		// Example: Generate random coordinates within a predefined boundary
		new_goal.pose.position.x = randomCoordinate(-5.0, 5.0);
		new_goal.pose.position.y = randomCoordinate(-5.0, 5.0);
		new_goal.pose.orientation.w = 1.0;  // Default orientation

		ROS_INFO("Generated dynamic goal at (x: %f, y: %f)", new_goal.pose.position.x, new_goal.pose.position.y);
		return new_goal;
	}

	double randomCoordinate(double min, double max) {
		return min + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (max - min)));
	}
	
	bool isGoalValid(const geometry_msgs::PoseStamped& goal) {
		// Example: Validate goal position is within a reasonable range
		if (goal.pose.position.x < -10.0 || goal.pose.position.x > 10.0 ||
		    goal.pose.position.y < -10.0 || goal.pose.position.y > 10.0) {
		    return false;
		}
		return true;
	}


	void startNavigation() {
		ros::Rate loop_rate(0.2); // Example: One goal every 5 seconds
		ROS_INFO("Starting dynamic navigation...");

		while (ros::ok()) {
		    geometry_msgs::PoseStamped next_goal = generateDynamicGoal();

		    if (isGoalValid(next_goal)) {
		        sendNextGoal(next_goal);
		    } else {
		        ROS_WARN("Generated goal is invalid or unreachable. Skipping...");
		    }

		    loop_rate.sleep();
		}
	}

	void sendNextGoal(const geometry_msgs::PoseStamped& goal_pose) {
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose = goal_pose;

		ROS_INFO("Sending dynamic goal: (x: %f, y: %f)", 
		         goal_pose.pose.position.x, goal_pose.pose.position.y);

		publishFeedback("Moving to dynamic goal at (x: " + 
		                std::to_string(goal.target_pose.pose.position.x) + ", y: " +
		                std::to_string(goal.target_pose.pose.position.y) + ").");

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

    void investigateAprilTag(const geometry_msgs::PoseStamped& tag_pose_in_map) {
        ROS_INFO("Investigating AprilTag at position (x: %f, y: %f, z: %f)",
                 tag_pose_in_map.pose.position.x, tag_pose_in_map.pose.position.y, tag_pose_in_map.pose.position.z);

        detected_cubes.push_back(tag_pose_in_map);
        publishFeedback("Investigating detected tag...");
        resumeNavigation();
    }

    void publishFeedback(const std::string& message) {
        std_msgs::String feedback_msg;
        feedback_msg.data = message;
        feedback_publisher.publish(feedback_msg);
    }

    void sendCubePositionsToNodeA() {
        ROS_INFO("Sending cube positions to Node A:");
        for (const auto& cube_pose : detected_cubes) {
            ROS_INFO("Cube at (x: %f, y: %f)", cube_pose.pose.position.x, cube_pose.pose.position.y);
            // Extend this section with actual communication to Node A
        }
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
            geometry_msgs::PoseStamped tag_pose_in_map = transformToMapFrame(tag_pose_in_base);

            ROS_INFO_STREAM("Tag Pose in Map Frame:\n" << tag_pose_in_map);

            transformed_pose_publisher.publish(tag_pose_in_map);

            publishFeedback("Processed AprilTag ID: " + std::to_string(tag_id));
            action_client.cancelGoal();
            investigateAprilTag(tag_pose_in_map);
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

