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
#include "assignment_1/SendCubePositions.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


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

        // Add a new subscriber for messages from Node A
        node_a_numbers_subscriber = nh.subscribe("/node_a/numbers_topic", 10, &NodeB::nodeANumbersCallback, this);

        // Subscribe to RGB-D data
        rgb_image_subscriber = it.subscribe("/xtion/rgb/image_raw", 10, &NodeB::imageCallback, this); //&NodeB::rgbImageCallback
        depth_image_subscriber = it.subscribe("/xtion/depth/image_raw", 10, &NodeB::depthImageCallback, this);

        // Publish the masked image and centroids
        mask_pub = it.advertise("/red_mask/image", 1);
        centroids_pub_ = nh.advertise<geometry_msgs::PointStamped>("/red_cubes/centroids", 10);

        // Initialize service client
        send_positions_client = nh.serviceClient<assignment_1::SendCubePositions>("/node_a/send_cube_positions");

        //ROS_INFO("Waiting for move_base action server...");
        action_client.waitForServer();
        //ROS_INFO("Node B initialized and move_base is ready.");

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
            //ROS_INFO("Waiting for head controller action server to come up...");
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
                    //ROS_INFO("Movement completed successfully with no errors");
                } else {
                    //ROS_WARN("Movement completed but with error code: %d", result->error_code);
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
    // Add a new subscriber member
    ros::Subscriber node_a_numbers_subscriber;

    image_transport::ImageTransport it;
    image_transport::Subscriber rgb_image_subscriber;
    image_transport::Subscriber depth_image_subscriber;
    image_transport::Publisher mask_pub;
    ros::Publisher centroids_pub_;
    cv::Mat latest_rgb_image;
    cv::Mat latest_depth_image;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    MoveBaseClient action_client;

    // Image data

    ros::Time latest_rgb_timestamp;  // Timestamp of the latest RGB image
    ros::Time latest_depth_timestamp;

    std::vector<int> received_tags;
    std::vector<geometry_msgs::PoseStamped> detected_cubes;
    std::vector<geometry_msgs::PoseStamped> navigation_goals;

    // Minimum area to consider as a valid cube (adjust as needed)
    const int MIN_AREA = 10;

    size_t current_goal_index = 0;
    bool goal_active = false;



	//-----------------------------------------------------------------
    // Create a callback function to handle numbers from Node A
    void nodeANumbersCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
        ROS_INFO("Received numbers from Node A:");
        for (size_t i = 0; i < msg->data.size(); ++i) {
            ROS_INFO("Number %zu: %d", i + 1, msg->data[i]);
        }
    }

    // -----------------------------------------------------------------

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Check if depth image is available
    if (latest_depth_image.empty()) {
        ROS_WARN("Depth image not available for processing");
        return;
    }
{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
		    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
		    ROS_ERROR("cv_bridge exception: %s", e.what());
		    return;
		}

		// Check if depth image is available
		if (latest_depth_image.empty()) {
		    ROS_WARN("Depth image not available for processing");
		    return;
		}

		// Convert to HSV for robust color detection
		cv::Mat hsv;
		cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

		// Improved Red Color Mask
		cv::Mat red_mask = createRobustRedMask(hsv);

		// Connected components analysis
		cv::Mat labels, stats, centroids;
		int num_labels = cv::connectedComponentsWithStats(red_mask, labels, stats, centroids, 8, CV_32S);

		// Vector to store valid cube poses
		std::vector<geometry_msgs::PoseStamped> detected_cubes;

		// Process each detected component
		for(int i = 1; i < num_labels; i++) // Start from 1 to skip background
		{
		    // Robust cube filtering
		    if (!isValidCube(stats, i)) continue;

		    // Get centroid coordinates
		    double x_pixel = centroids.at<double>(i, 0);
		    double y_pixel = centroids.at<double>(i, 1);

		    // Depth and 3D Position Estimation
		    geometry_msgs::PoseStamped cube_pose = estimateCubePose(x_pixel, y_pixel);
		    
		    if (cube_pose.header.frame_id.empty()) {
		        // Invalid pose estimation
		        continue;
		    }

		    // Add to detected cubes
		    detected_cubes.push_back(cube_pose);

		    // Visualization
		    cv::circle(cv_ptr->image, cv::Point(x_pixel, y_pixel), 5, cv::Scalar(0, 255, 0), -1);
		}

		// Publish detected cube positions
		publishDetectedCubes(detected_cubes);

		// Publish visualization
		sensor_msgs::ImagePtr vis_msg = 
		    cv_bridge::CvImage(msg->header, "bgr8", cv_ptr->image).toImageMsg();
		mask_pub.publish(vis_msg);
	}
    // Convert to HSV for robust color detection
    cv::Mat hsv;
    cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

    // Improved Red Color Mask
    cv::Mat red_mask = createRobustRedMask(hsv);

    // Connected components analysis
    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(red_mask, labels, stats, centroids, 8, CV_32S);

    // Vector to store valid cube poses
    std::vector<geometry_msgs::PoseStamped> detected_cubes;

    // Process each detected component
    for(int i = 1; i < num_labels; i++) // Start from 1 to skip background
    {
        // Robust cube filtering
        if (!isValidCube(stats, i)) continue;

        // Get centroid coordinates
        double x_pixel = centroids.at<double>(i, 0);
        double y_pixel = centroids.at<double>(i, 1);

        // Depth and 3D Position Estimation
        geometry_msgs::PoseStamped cube_pose = estimateCubePose(x_pixel, y_pixel);
        
        if (cube_pose.header.frame_id.empty()) {
            // Invalid pose estimation
            continue;
        }

        // Add to detected cubes
        detected_cubes.push_back(cube_pose);

        // Visualization
        cv::circle(cv_ptr->image, cv::Point(x_pixel, y_pixel), 5, cv::Scalar(0, 255, 0), -1);
    }

    // Publish detected cube positions
    publishDetectedCubes(detected_cubes);

    // Publish visualization
    sensor_msgs::ImagePtr vis_msg = 
        cv_bridge::CvImage(msg->header, "bgr8", cv_ptr->image).toImageMsg();
    mask_pub.publish(vis_msg);
}

cv::Mat createRobustRedMask(const cv::Mat& hsv)
{
    cv::Mat mask1, mask2, mask3;
    
    // Define red color ranges in HSV
    cv::Scalar lower_red1(0, 100, 100);
    cv::Scalar upper_red1(10, 255, 255);
    cv::Scalar lower_red2(160, 100, 100);
    cv::Scalar upper_red2(180, 255, 255);
    
    // Create masks for red ranges
    cv::inRange(hsv, lower_red1, upper_red1, mask1);
    cv::inRange(hsv, lower_red2, upper_red2, mask2);
    
    // Combine red masks
    cv::Mat red_mask = mask1 | mask2;
    
    // Morphological operations for noise reduction
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);
    
    return red_mask;
}

bool isValidCube(const cv::Mat& stats, int label)
{
    // Cube filtering parameters
    const int MIN_AREA = 100;     // Minimum pixel area
    const int MAX_AREA = 5000;    // Maximum pixel area
    const double MAX_ASPECT_RATIO_DEVIATION = 0.5;
    
    int area = stats.at<int>(label, cv::CC_STAT_AREA);
    int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
    int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
    
    // Area filtering
    if (area < MIN_AREA || area > MAX_AREA) {
        return false;
    }
    
    // Aspect ratio check (expecting roughly square shape)
    double aspect_ratio = static_cast<double>(width) / height;
    if (std::abs(aspect_ratio - 1.0) > MAX_ASPECT_RATIO_DEVIATION) {
        return false;
    }
    
    return true;
}

geometry_msgs::PoseStamped estimateCubePose(double x_pixel, double y_pixel)
{
    geometry_msgs::PoseStamped cube_pose;
    
    // Validate pixel coordinates
    if (x_pixel < 0 || y_pixel < 0 || 
        x_pixel >= latest_depth_image.cols || 
        y_pixel >= latest_depth_image.rows) {
        ROS_WARN("Invalid pixel coordinates");
        return cube_pose;
    }
    
    // Get depth value
    uint16_t depth_value = latest_depth_image.at<uint16_t>(y_pixel, x_pixel);
    
    // Filter out invalid depth readings
    const uint16_t MIN_DEPTH = 500;   // 0.5 meters
    const uint16_t MAX_DEPTH = 5000;  // 5 meters
    if (depth_value < MIN_DEPTH || depth_value > MAX_DEPTH) {
        ROS_WARN("Depth value out of valid range");
        return cube_pose;
    }
    
    // Convert depth to meters
    double depth_meters = depth_value * 0.001; // Assuming millimeters
    
    // Camera intrinsic parameters (example values, replace with your camera's calibration)
    const double fx = 525.0;  // Focal length x
    const double fy = 525.0;  // Focal length y
    const double cx = 319.5;  // Principal point x
    const double cy = 239.5;  // Principal point y
    
    // Convert pixel coordinates to 3D camera frame
    double x_camera = (x_pixel - cx) * depth_meters / fx;
    double y_camera = (y_pixel - cy) * depth_meters / fy;
    double z_camera = depth_meters;
    
    // Transform to map frame
    try {
        geometry_msgs::PoseStamped camera_pose;
        camera_pose.header.frame_id = "camera_depth_optical_frame";
        camera_pose.header.stamp = ros::Time::now();
        camera_pose.pose.position.x = x_camera;
        camera_pose.pose.position.y = y_camera;
        camera_pose.pose.position.z = z_camera;
        camera_pose.pose.orientation.w = 1.0;
        
        // Transform to map frame
        geometry_msgs::PoseStamped map_pose = 
            tf_buffer.transform(camera_pose, "map", ros::Duration(1.0));
        
        return map_pose;
    }
    catch (tf2::TransformException& ex) {
        ROS_ERROR("Pose transformation failed: %s", ex.what());
        return cube_pose;
    }
}

void publishDetectedCubes(const std::vector<geometry_msgs::PoseStamped>& cubes)
{
    // Publish cube positions as a PoseArray
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "map";
    pose_array.header.stamp = ros::Time::now();
    
    for (const auto& cube : cubes) {
        pose_array.poses.push_back(cube.pose);
        
        // Optionally add to navigation goals
        addUniqueNavigationGoal(cube);
        
        ROS_INFO("Detected Cube at (x: %.2f, y: %.2f, z: %.2f)", 
                 cube.pose.position.x, 
                 cube.pose.position.y, 
                 cube.pose.position.z);
    }
    
    // Publish pose array
    cube_positions_publisher.publish(pose_array);
}

void addUniqueNavigationGoal(const geometry_msgs::PoseStamped& new_goal)
{
    const double MIN_GOAL_DISTANCE = 0.5; // meters
    
    // Check if goal is too close to existing goals
    auto is_too_close = [&](const geometry_msgs::PoseStamped& existing_goal) {
        double dx = new_goal.pose.position.x - existing_goal.pose.position.x;
        double dy = new_goal.pose.position.y - existing_goal.pose.position.y;
        return std::sqrt(dx*dx + dy*dy) < MIN_GOAL_DISTANCE;
    };
    
    // Check if goal already exists
    auto exists = std::any_of(navigation_goals.begin(), navigation_goals.end(), is_too_close);
    
    if (!exists) {
        navigation_goals.push_back(new_goal);
    }
}


    /// -----------------------------------------------------------------
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
		    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		    latest_rgb_image = cv_ptr->image;
		    latest_rgb_timestamp = msg->header.stamp; // Store timestamp
		} catch (cv_bridge::Exception& e) {
		    ROS_ERROR("cv_bridge exception: %s", e.what());
		}
	}

	void depthImageCallback(const sensor_msgs::ImageConstPtr& msg) {
		try {
		    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
		    latest_depth_image = cv_ptr->image;
		    latest_depth_timestamp = msg->header.stamp; // Store timestamp
		} catch (cv_bridge::Exception& e) {
		    ROS_ERROR("cv_bridge exception: %s", e.what());
		}
	}

    void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
        std::string target_frame = "map";
        std::string source_frame = msg->header.frame_id;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        while(!tfBuffer.canTransform(target_frame, source_frame, ros::Time(0)))
            ros::Duration(0.5).sleep();

        geometry_msgs::TransformStamped transformed = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
        
        //Transform available
        geometry_msgs::PoseStamped pos_in;
        geometry_msgs::PoseStamped pos_out;

        for(int i = 0; i < msg->detections.size(); ++i)
        {
            pos_in.header.frame_id = msg->detections.at(i).pose.header.frame_id;
            pos_in.pose.position.x = msg->detections.at(i).pose.pose.pose.position.x;
            pos_in.pose.position.y = msg->detections.at(i).pose.pose.pose.position.y;
            pos_in.pose.position.z = msg->detections.at(i).pose.pose.pose.position.z;
            pos_in.pose.orientation.x = msg->detections.at(i).pose.pose.pose.orientation.x;
            pos_in.pose.orientation.y = msg->detections.at(i).pose.pose.pose.orientation.y;
            pos_in.pose.orientation.z = msg->detections.at(i).pose.pose.pose.orientation.z;
            pos_in.pose.orientation.w = msg->detections.at(i).pose.pose.pose.orientation.w;
        
            tf2::doTransform(pos_in, pos_out, transformed);

            // original pose
            const auto& pp = pos_in.pose.position;
            const auto& po = pos_in.pose.orientation;
            // transformed pose
            const auto& mp = pos_out.pose.position;
            const auto& mo = pos_out.pose.orientation;

            //ROS_INFO_STREAM("Obj with ID: " << msg->detections.at(i).id[0]);
            //ROS_INFO_STREAM("Original pose:    ( " << pp.x << " , " << pp.y << " , " << pp.z << " )");
            //ROS_INFO_STREAM("Transformed pose: ( " << mp.x << " , " << mp.y << " , " << mp.z << " )");
        }
    }

/*
ROS_INFO_STREAM("Orientation (quaternion):\n"
                        "  x: " << q.x << "\n"
                        "  y: " << q.y << "\n"
                        "  z: " << q.z << "\n"
                        "  w: " << q.w);
*/
        
      



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
		    adjustHeadAngle(-0.5);  // Adjust head by -30 degrees (radians)
		} else {
		    ROS_WARN("[Node B] Failed to reach goal. Retrying...");
		    publishFeedback("The robot failed to reach its goal. Retrying...");
		}
	}

	bool adjustHeadAngle(double tilt_angle_radians) {
		HeadClient head_client("head_controller/follow_joint_trajectory", true);
		head_client.waitForServer();
		control_msgs::FollowJointTrajectoryGoal goal;
		goal.trajectory.joint_names.push_back("head_tilt_joint");  // Assuming the joint's name
		trajectory_msgs::JointTrajectoryPoint point;
		point.positions.push_back(tilt_angle_radians);
		point.time_from_start = ros::Duration(1);
		goal.trajectory.points.push_back(point);
		head_client.sendGoal(goal);
		bool finished_before_timeout = head_client.waitForResult(ros::Duration(5.0));
		if (finished_before_timeout) {
		    actionlib::SimpleClientGoalState state = head_client.getState();
		    ROS_INFO("Head movement completed: %s", state.toString().c_str());
		    return true;
		} else {
		    ROS_ERROR("Failed to adjust head angle.");
		    return false;
		}
	}


    void activeCallback() {
        ROS_INFO("[Node B] Goal is now active.");
    }

    void feedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback) {
        //ROS_INFO("[Node B] Feedback: Current position (x: %f, y: %f)",
        //         feedback->base_position.pose.position.x,
        //         feedback->base_position.pose.position.y);
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
