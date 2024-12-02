
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <algorithm>
#include <tf/transform_datatypes.h>
#include "assignment_1/SendCubePositions.h"  // Replace 'assignment_1' with your package name
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <geometry_msgs/Twist.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> HeadClient;

class NodeB {
public:
    NodeB(ros::NodeHandle& nh) : action_client("move_base", true), tf_listener(tf_buffer), it(nh) {
        // Initialize subscribers and publishers
        tag_detections_subscriber = nh.subscribe("/tag_detections", 10, &NodeB::tagDetectionsCallback, this);
        target_ids_subscriber = nh.subscribe("/apriltag_ids_topic", 10, &NodeB::targetIdsCallback, this);
        feedback_publisher = nh.advertise<std_msgs::String>("/node_b/feedback", 10);
        cube_positions_publisher = nh.advertise<geometry_msgs::PoseArray>("/node_b/cube_positions", 10);

        // Subscribe to RGB-D data
        rgb_image_subscriber = it.subscribe("/xtion/rgb/image_raw", 10, &NodeB::rgbImageCallback, this);
        depth_image_subscriber = it.subscribe("/xtion/depth/image_raw", 10, &NodeB::depthImageCallback, this);

        // Initialize service client
        send_positions_client = nh.serviceClient<assignment_1::SendCubePositions>("/node_a/send_cube_positions");

        ROS_INFO("Waiting for move_base action server...");
        action_client.waitForServer();
        ROS_INFO("Node B initialized and move_base is ready.");

        // adjust camera
        bool success = adjustCameraAngle(-0.5);
        if (success) {
            ROS_INFO("Camera movement completed successfully");
        } else {
            ROS_ERROR("Failed to move camera");
        }

        // Define navigation goals
        defineNavigationGoals();
        startNavigation();
    }

    // adjusting camera angle
	bool  adjustCameraAngle(double tilt_angle) {

        HeadClient* head_client_ = new HeadClient("head_controller/follow_joint_trajectory", true);
        
        while(!head_client_->waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for head controller action server to come up...");
        }

        // moveCamera
        control_msgs::FollowJointTrajectoryGoal goal;
        
        goal.trajectory.joint_names.push_back("head_1_joint");
        goal.trajectory.joint_names.push_back("head_2_joint");
        
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.push_back(0.0);
        point.positions.push_back(tilt_angle);
        
        point.velocities.push_back(0.1);
        point.velocities.push_back(0.1);
        
        point.time_from_start = ros::Duration(2.0);
        
        goal.trajectory.points.push_back(point);
        
        // Send the goal and wait for result with timeout
        bool finished_before_timeout = head_client_->sendGoalAndWait(goal, ros::Duration(5.0)) == actionlib::SimpleClientGoalState::SUCCEEDED;
        
        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = head_client_->getState();
            ROS_INFO("Camera movement succeeded! Final state: %s", state.toString().c_str());
            
            // Get the result
            control_msgs::FollowJointTrajectoryResultConstPtr result = head_client_->getResult();
            if (result) {
                // Check error code
                if (result->error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL) {
                    ROS_INFO("Movement completed successfully with no errors");
                } else {
                    ROS_WARN("Movement completed but with error code: %d", result->error_code);
                }
            }
            delete head_client_;
            return true;
        } else {
            ROS_ERROR("Camera movement failed or timed out!");
            actionlib::SimpleClientGoalState state = head_client_->getState();
            ROS_ERROR("Final state: %s", state.toString().c_str());
            delete head_client_;
            return false;
        }
	}


private:
	ros::NodeHandle nh;
    ros::Subscriber tag_detections_subscriber;
    ros::Subscriber target_ids_subscriber;
    ros::Publisher feedback_publisher;
    ros::Publisher cube_positions_publisher;
    ros::ServiceClient send_positions_client;

    image_transport::ImageTransport it;
    image_transport::Subscriber rgb_image_subscriber;
    image_transport::Subscriber depth_image_subscriber;
    cv::Mat latest_rgb_image;
    cv::Mat latest_depth_image;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    MoveBaseClient action_client;

    std::vector<int> received_tags;
    std::vector<geometry_msgs::PoseStamped> detected_cubes;
    std::vector<geometry_msgs::PoseStamped> navigation_goals;

    size_t current_goal_index = 0;
    bool goal_active = false;
	void printReceivedTags() {
		std::ostringstream oss;
		oss << "Received tags: ";
		for (int id : received_tags) {
		    oss << id << " ";
		}
		ROS_INFO("[Node B] %s", oss.str().c_str());
	}


	void targetIdsCallback(const std_msgs::Int32MultiArray::ConstPtr& ids_msg) {
		received_tags.clear();
		for (int id : ids_msg->data) {
		    received_tags.push_back(id);
		}
		printReceivedTags();
	}



    void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            latest_rgb_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
            //ROS_INFO("[Node B] Received RGB image.");
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            latest_depth_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
            ROS_INFO("[Node B] Received depth image.");
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& detections_msg) {
        if (detections_msg->detections.empty()) {
            ROS_INFO_THROTTLE(1, "[Node B] No tags detected.");
            publishFeedback("The robot is scanning.");
            return;
        }

        for (const auto& detection : detections_msg->detections) {
            int tag_id = detection.id[0];

            if (std::find(received_tags.begin(), received_tags.end(), tag_id) != received_tags.end()) {
                ROS_INFO("[Node B] Detected target Apriltag ID: %d", tag_id);

                geometry_msgs::PoseStamped tag_pose_in_camera;
                tag_pose_in_camera.header = detection.pose.header;
                tag_pose_in_camera.pose = detection.pose.pose.pose;

                int u = static_cast<int>(tag_pose_in_camera.pose.position.x * 100);
                int v = static_cast<int>(tag_pose_in_camera.pose.position.y * 100);
                if (u >= 0 && v >= 0 && u < latest_depth_image.cols && v < latest_depth_image.rows) {
                    double depth = latest_depth_image.at<uint16_t>(v, u) * 0.001;
                    if (depth > 0.2 && depth < 2.0) {
                        ROS_INFO("[Node B] Valid depth: %f meters", depth);
                    } else {
                        ROS_WARN("[Node B] Invalid depth for tag %d. Ignoring.", tag_id);
                        continue;
                    }
                } else {
                    ROS_WARN("[Node B] Tag %d out of image bounds. Ignoring.", tag_id);
                    continue;
                }

                geometry_msgs::PoseStamped tag_pose_in_map = transformPose(tag_pose_in_camera, "map");
                if (!tag_pose_in_map.header.frame_id.empty()) {
                    detected_cubes.push_back(tag_pose_in_map);
                    ROS_INFO("[Node B] Transformed pose to map frame: (x: %f, y: %f, z: %f)",
                             tag_pose_in_map.pose.position.x,
                             tag_pose_in_map.pose.position.y,
                             tag_pose_in_map.pose.position.z);
                }

                publishFeedback("The robot has already found Apriltags with IDs: " +
                                std::to_string(tag_id));

                if (goal_active) {
                    action_client.cancelGoal();
                    goal_active = false;
                }

                if (detected_cubes.size() == received_tags.size()) {
                    ROS_INFO("[Node B] All target tags detected. Task complete.");
                    publishFeedback("The detection is finished. All Apriltags found.");
                    sendCubePositionsToNodeA();
                    ros::shutdown();
                }
            }
        }
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

    void defineNavigationGoals() {
        geometry_msgs::PoseStamped goal1;
        goal1.header.frame_id = "map";
        goal1.header.stamp = ros::Time::now();
        goal1.pose.position.x = 5.0;
        goal1.pose.position.y = 0.0;
        goal1.pose.orientation.w = 1.0;

        geometry_msgs::PoseStamped goal2 = goal1;
        goal2.pose.position.x = 10.5;
        goal2.pose.position.y = -3.7;

        geometry_msgs::PoseStamped goal3 = goal1;
        goal3.pose.position.x = 9.6;
        goal3.pose.position.y = 0.7;

        double yaw = M_PI;
        geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(yaw);
        goal3.pose.orientation = quaternion;

        navigation_goals.push_back(goal1);
        navigation_goals.push_back(goal2);
        navigation_goals.push_back(goal3);
    }

    void startNavigation() {
        ROS_INFO("Starting navigation...");
        while (current_goal_index < navigation_goals.size() && ros::ok()) {
            if (!goal_active) {
                sendNextGoal(navigation_goals[current_goal_index]);
            }
            ros::spinOnce();
        }
    }

    void sendNextGoal(const geometry_msgs::PoseStamped& goal_pose) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = goal_pose;

        ROS_INFO("[Node B] Sending goal to (x: %f, y: %f), index = %ld", goal_pose.pose.position.x, goal_pose.pose.position.y, current_goal_index);
        publishFeedback("The robot is moving.");
        goal_active = true;

        action_client.sendGoal(goal,
            boost::bind(&NodeB::doneCallback, this, _1, _2),
            boost::bind(&NodeB::activeCallback, this),
            boost::bind(&NodeB::feedbackCallback, this, _1));
    }

    void doneCallback(const actionlib::SimpleClientGoalState& state,
                      const move_base_msgs::MoveBaseResult::ConstPtr& result) {
        goal_active = false;
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("[Node B] Reached goal successfully.");
            current_goal_index++;
            publishFeedback("The robot has reached its goal.");
        } else {
            ROS_WARN("[Node B] Failed to reach goal. Retrying...");
            publishFeedback("The robot failed to reach its goal. Retrying...");
        }
    }

    void activeCallback() {
        ROS_INFO("[Node B] Goal is now active.");
    }

    void feedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback) {
        ROS_INFO("[Node B] Feedback: Current position (x: %f, y: %f)",
                 feedback->base_position.pose.position.x,
                 feedback->base_position.pose.position.y);
    }

    void sendCubePositionsToNodeA() {
        assignment_1::SendCubePositions srv;
        geometry_msgs::PoseArray cube_positions;
        cube_positions.header.frame_id = "map";
        cube_positions.header.stamp = ros::Time::now();

        for (const auto& cube_pose : detected_cubes) {
            cube_positions.poses.push_back(cube_pose.pose);
            ROS_INFO("Cube at (x: %f, y: %f)", cube_pose.pose.position.x, cube_pose.pose.position.y);
        }

        srv.request.positions = cube_positions;

        if (send_positions_client.call(srv)) {
            if (srv.response.success) {
                ROS_INFO("Cube positions successfully sent to Node A.");
            } else {
                ROS_WARN("Node A failed to process cube positions.");
            }
        } else {
            ROS_ERROR("Failed to call service /node_a/send_cube_positions.");
        }
    }

    void publishFeedback(const std::string& message) {
        std_msgs::String feedback_msg;
        feedback_msg.data = message;
        feedback_publisher.publish(feedback_msg);
    }


};

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_b");
    ros::NodeHandle nh;

    NodeB node_b(nh); // Pass NodeHandle when creating the NodeB object


    ros::spin();
    return 0;
}

