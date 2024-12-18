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
    /**
     * Subscribes to topics, advertises publishers and sets up action clients.
     */
    NodeB(ros::NodeHandle& nh) : action_client("move_base", true), tf_listener(tf_buffer), it(nh) {
        tag_detections_subscriber = nh.subscribe("/tag_detections", 10, &NodeB::tagDetectionsCallback, this);
        target_ids_subscriber = nh.subscribe("/apriltag_ids_topic", 10, &NodeB::targetIdsCallback, this);
        feedback_publisher = nh.advertise<std_msgs::String>("/node_b/feedback", 10);
        cube_positions_publisher = nh.advertise<geometry_msgs::PoseArray>("/node_b/cube_positions", 10);

        rgb_image_subscriber = it.subscribe("/xtion/rgb/image_raw", 10, &NodeB::imageCallback, this);
        depth_image_subscriber = it.subscribe("/xtion/depth_registered/image_raw", 10, &NodeB::depthImageCallback, this);   

        sub_camera_info = nh.subscribe("/xtion/depth_registered/camera_info", 10, &NodeB::cameraInfoCallback, this);  

        mask_pub = it.advertise("/red_mask/image", 1);
        centroids_pub_ = nh.advertise<geometry_msgs::PointStamped>("/red_cubes/centroids", 10);

        send_positions_client = nh.serviceClient<assignment_1::SendCubePositions>("/node_a/send_cube_positions");

		cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 10);
		feedback_publisher = nh.advertise<std_msgs::String>("node_b_feedback", 10);

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


    bool adjustCameraAngle(double tilt_angle) {
        // Create an action client to control the head
        HeadClient* head_client_ = new HeadClient("head_controller/follow_joint_trajectory", true);
        
        // Wait for the action server to come up
        while(!head_client_->waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for head controller action server to come up...");
        }
        
        // Create a goal to move the camera
        control_msgs::FollowJointTrajectoryGoal goal;
        
        // Set the joint names
        goal.trajectory.joint_names.push_back("head_1_joint");
        goal.trajectory.joint_names.push_back("head_2_joint");
        
        // Set the joint positions
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.push_back(0.0);
        point.positions.push_back(tilt_angle);
        
        // Set the joint velocities
        point.velocities.push_back(0.1);
        point.velocities.push_back(0.1);
        
        // Set the time from start
        point.time_from_start = ros::Duration(2.0);
        
        // Add the point to the trajectory
        goal.trajectory.points.push_back(point);
        
        // Send the goal and wait for result with timeout
        bool finished_before_timeout = head_client_->sendGoalAndWait(goal, ros::Duration(5.0)) == actionlib::SimpleClientGoalState::SUCCEEDED;
        
        if (finished_before_timeout) {
            // Get the result
            control_msgs::FollowJointTrajectoryResultConstPtr result = head_client_->getResult();
            if (result) {
                // Check error code
                if (result->error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL) {
                    ROS_INFO("Camera movement completed successfully with no errors");
                } else {
                    ROS_WARN("Camera movement completed but with error code: %d", result->error_code);
                }
            }
            
            // Clean up
            delete head_client_;
            return true;
        } else {
            delete head_client_;
            ROS_ERROR("Camera movement failed or timed out!");
            return false;
        }
    }


private:
	ros::NodeHandle nh;
    ros::Subscriber tag_detections_subscriber;
    ros::Subscriber target_ids_subscriber;
    ros::Subscriber sub_camera_info;
    ros::Publisher feedback_publisher;
    ros::Publisher cube_positions_publisher;
    ros::ServiceClient send_positions_client;	
	std::set<int> seen_tags;																																																																																																						              			

	ros::Publisher cmd_vel_publisher;

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

    std::vector<int> received_tags;
    std::vector<geometry_msgs::PoseStamped> detected_cubes;
    std::vector<geometry_msgs::PoseStamped> navigation_goals;

    // Minimum area to consider as a valid cube (adjust as needed)
    const int MIN_AREA = 10;

    size_t current_goal_index = 0;
    bool goal_active = false;

    // -----------------------------------------------------------------
    // intrinsic parameters
    double fx,fy,cx,cy;
    bool intrinsicParamInit = false;
    bool depthInit = false;
    //vector for the raw depth values, h:480, w:640
    //std::vector<std::vector<float>> depth_matrix(HEIGHT, std::vector<float>(WIDTH));


    // -----------------------------------------------------------------

	void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
		cv_bridge::CvImagePtr cv_ptr;
		try {
		    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
		    ROS_ERROR("cv_bridge exception: %s", e.what());
		    return;
		}

		// Convert BGR to HSV
		cv::Mat hsv;
		cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

		// Define range for red color in HSV
		cv::Mat mask1, mask2;
		cv::Scalar lower_red1(0, 100, 100);
		cv::Scalar upper_red1(10, 255, 255);
		cv::Scalar lower_red2(160, 100, 100);
		cv::Scalar upper_red2(180, 255, 255);

		// Create masks for both red ranges
		cv::inRange(hsv, lower_red1, upper_red1, mask1);
		cv::inRange(hsv, lower_red2, upper_red2, mask2);

		// Combine the masks
		cv::Mat red_mask = mask1 | mask2;

		// Apply morphological operations
		int kernel_size = 5;
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
		cv::morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel);
		cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);

		// Connected components analysis
		cv::Mat labels, stats, centroids;
		int num_labels = cv::connectedComponentsWithStats(red_mask, labels, stats, centroids, 8, CV_32S);

		// Process each detected component
		for (int i = 1; i < num_labels; i++) { // Start from 1 to skip the background
		    int area = stats.at<int>(i, cv::CC_STAT_AREA);
		    if (area < MIN_AREA) continue; // Ignore small blobs

		    // Get centroid
		    double x = centroids.at<double>(i, 0);
		    double y = centroids.at<double>(i, 1);

		    // Get depth at the centroid
		    if (depthInit && intrinsicParamInit) {
		        int x_int = static_cast<int>(std::round(x));
		        int y_int = static_cast<int>(std::round(y));
		        float depth = latest_depth_image.at<float>(y_int, x_int); // Depth at pixel (y, x)

		        if (depth > 0) { // Ensure valid depth
		            // Compute 3D coordinates in the camera frame
		            float X = (x - cx) * depth / fx;
		            float Y = (y - cy) * depth / fy;
		            float Z = depth;

		            geometry_msgs::PointStamped camera_point, map_point;
		            camera_point.header.frame_id = "xtion_rgb_optical_frame";
		            camera_point.header.stamp = ros::Time(0); // Use the latest transform
		            camera_point.point.x = X;
		            camera_point.point.y = Y;
		            camera_point.point.z = Z;

		            try {
		                // Transform the point from the camera frame to the map frame
		                map_point = tf_buffer.transform(camera_point, "map");

		                // Extract and print the real-world coordinates
		                float real_x = map_point.point.x;
		                float real_y = map_point.point.y;
		                float real_z = map_point.point.z;

		                ROS_INFO("Red cube %d real-world position in map frame: [x: %f, y: %f, z: %f]",
		                         i, real_x, real_y, real_z);

		                // Optionally, store the cube's position for navigation
		                geometry_msgs::PoseStamped cube_pose;
		                cube_pose.header.frame_id = "map";
		                cube_pose.header.stamp = ros::Time::now();
		                cube_pose.pose.position.x = real_x;
		                cube_pose.pose.position.y = real_y;
		                cube_pose.pose.position.z = real_z;
		                cube_pose.pose.orientation.w = 1.0; // Default orientation
		                detected_cubes.push_back(cube_pose);

		            } catch (tf2::TransformException &ex) {
		                ROS_WARN("Transform failed for red cube %d: %s", i, ex.what());
		            }
		        } else {
		            ROS_WARN("Invalid depth value for cube %d at pixel (%d, %d)", i, x_int, y_int);
		        }
		    }
		}

		// Publish the red mask for visualization
		sensor_msgs::ImagePtr vis_msg = cv_bridge::CvImage(msg->header, "bgr8", red_mask).toImageMsg();
		mask_pub.publish(vis_msg);
	}




    struct TagInfo {
        int id;
        float x, y; // Coordinates of the tag in some reference frame
    };

    std::map<int, TagInfo> tagInfos; // Maps tag ID to its information


    /**
     * Finds the tag ID closest to the given (x, y) coordinates.
     *
     * cubeX The x-coordinate of the cube.
     * cubeY The y-coordinate of the cube.
     */
    int findClosestTagId(float cubeX, float cubeY) {
        int closestId = -1;
        float minDistance = std::numeric_limits<float>::max();
        for (std::map<int, TagInfo>::const_iterator it = tagInfos.begin(); it != tagInfos.end(); ++it) {
            const int& id = it->first;
            const TagInfo& info = it->second;
            // Calculate the Euclidean distance between the cube and the tag
            float distance = std::hypot(cubeX - info.x, cubeY - info.y);
            if (distance < minDistance) {
                // Update the closest tag ID and distance
                minDistance = distance;
                closestId = id;
            }
        }
        return closestId;
    }

	/**
	 * Rotates the robot 360 degrees counterclockwise.
	 *
	 * This function uses a fixed loop rate and publishes Twist messages to the
	 * mobile base controller to perform the rotation. The duration of the
	 * rotation is calculated based on the desired angular velocity.
	 */
	void perform360Rotation() {
		ros::Rate rate(10); // 10 Hz loop rate
		geometry_msgs::Twist twist_msg;

		// Set rotational velocity (radians per second)
		twist_msg.linear.x = 0.0;   
		twist_msg.angular.z = 1.0; // Rotate counterclockwise (1 rad/s)

		// Calculate the duration for a full rotation (360 degrees = 2π radians)
		double rotation_duration = 2 * M_PI / twist_msg.angular.z;
		ros::Time start_time = ros::Time::now();

		// Publish rotation commands for the calculated duration
		ROS_INFO("[Node B] Starting 360-degree rotation.");
		while (ros::ok() && (ros::Time::now() - start_time).toSec() < rotation_duration) {
		    cmd_vel_publisher.publish(twist_msg);
		    rate.sleep();
		}

		// Stop the robot after completing the rotation
		twist_msg.angular.z = 0.0;
		cmd_vel_publisher.publish(twist_msg);
		ROS_INFO("[Node B] 360-degree rotation completed.");
	}


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
        try {  // CV_8UC1
            //latest_depth_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            latest_depth_image = cv_ptr->image;
            int height = cv_ptr->image.rows;
            int width = cv_ptr->image.cols;
            ROS_INFO("[Node B] Received depth image.");
            depthInit = true;

            ROS_INFO_STREAM("encoding " << msg->encoding);

            ROS_INFO_STREAM("msg dim: (h: " << msg->height << ", w: " << msg->width << ")");
            ROS_INFO_STREAM("Mat depth dim: (h: "<< latest_depth_image.rows << " , w: " << latest_depth_image.cols << " )" );

            // this prints numbers
            /*for(int i = 0; i < latest_depth_image.rows; i++) {     // 480, height = rows
                for(int j = 0; j < latest_depth_image.cols; j++) {   // 640  width = cols
                    float value = latest_depth_image.at<float>(i, j);
                    ROS_INFO("Pixel at (%d, %d): %.3f", i, j, value);
                }
            }*/

            //ROS_INFO_STREAM("msg ")
        } catch (cv_bridge::Exception& e) {
            //ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }



    void cameraInfoCallback (const sensor_msgs::CameraInfo::ConstPtr& msg) {
        // Access intrinsic parameters
        // Camera matrix (K) elements
        fx = msg->K[0]; // focal length x
        fy = msg->K[4]; // focal length y
        cx = msg->K[2]; // optical center x
        cy = msg->K[5]; // optical center y

        intrinsicParamInit = true;
        
        // Distortion coefficients
        //std::vector<double> distortion_coeffs = msg->D;
    }


	void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
		publishFeedback("The robot is scanning for AprilTags.");
		std::string target_frame = "map";
		std::string source_frame = msg->header.frame_id;

		for (const auto& detection : msg->detections) {
		    TagInfo info;
		    info.id = detection.id[0]; // Assuming each detection has exactly one ID
		    info.x = detection.pose.pose.pose.position.x; // Example: X coordinate
		    info.y = detection.pose.pose.pose.position.y; // Example: Y coordinate
		    tagInfos[info.id] = info;
		}

		tf2_ros::Buffer tfBuffer;
		tf2_ros::TransformListener tfListener(tfBuffer);

		while (!tfBuffer.canTransform(target_frame, source_frame, ros::Time(0))) {
		    ros::Duration(0.5).sleep();
		}

		geometry_msgs::TransformStamped transformed = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
		geometry_msgs::PoseStamped pos_in;
		geometry_msgs::PoseStamped pos_out;

		for (int i = 0; i < msg->detections.size(); ++i) {
		    pos_in.header.frame_id = msg->detections.at(i).pose.header.frame_id;
		    pos_in.pose.position.x = msg->detections.at(i).pose.pose.pose.position.x;
		    pos_in.pose.position.y = msg->detections.at(i).pose.pose.pose.position.y;
		    pos_in.pose.position.z = msg->detections.at(i).pose.pose.pose.position.z;
		    pos_in.pose.orientation.x = msg->detections.at(i).pose.pose.pose.orientation.x;
		    pos_in.pose.orientation.y = msg->detections.at(i).pose.pose.pose.orientation.y;
		    pos_in.pose.orientation.z = msg->detections.at(i).pose.pose.pose.orientation.z;
		    pos_in.pose.orientation.w = msg->detections.at(i).pose.pose.pose.orientation.w;

		    tf2::doTransform(pos_in, pos_out, transformed);

		    // Get the ID of the detected object
		    int detected_id = msg->detections.at(i).id[0];

		    // Check if the ID is in the received_tags list
		    if (std::find(received_tags.begin(), received_tags.end(), detected_id) != received_tags.end()) {
		        ROS_INFO_STREAM("Detected ID " << detected_id << " is in the received list.");
		    } else {
		        ROS_WARN_STREAM("Detected ID " << detected_id << " is NOT in the received list.");
		    }

		    // Check if this tag ID has been seen before
		    if (seen_tags.find(detected_id) != seen_tags.end()) {
		        ROS_INFO_STREAM("The robot has already found AprilTag with ID: " << detected_id);
		    } else {
		        // First time seeing this tag ID
		        seen_tags.insert(detected_id);
		        ROS_INFO_STREAM("The robot has found a new AprilTag with ID: " << detected_id);
		    }

		    // Original pose
		    const auto& pp = pos_in.pose.position;
		    const auto& po = pos_in.pose.orientation;

		    // Transformed pose
		    const auto& mp = pos_out.pose.position;
		    const auto& mo = pos_out.pose.orientation;

		    ROS_INFO_STREAM("Original pose:    ( " << pp.x << " , " << pp.y << " , " << pp.z << " )");
		    ROS_INFO_STREAM("Transformed pose: ( " << mp.x << " , " << mp.y << " , " << mp.z << " )");
		}

		// Publish feedback about all previously found AprilTag IDs
		std::ostringstream oss;
		oss << "The robot has already found AprilTags with IDs: ";
		for (int id : seen_tags) {
		    oss << id << " ";
		}
		publishFeedback(oss.str());
	}


/*
ROS_INFO_STREAM("Orientation (quaternion):\n"
                        "  x: " << q.x << "\n"
                        "  y: " << q.y << "\n"
                        "  z: " << q.z << "\n"
                        "  w: " << q.w);
*/
        
      



    /**
     * Define navigation goals for the robot.
     * The goals are defined as geometry_msgs::PoseStamped messages.
     */
    void defineNavigationGoals() {
        // Define the first goal
        geometry_msgs::PoseStamped goal1;
        goal1.header.frame_id = "map";
        goal1.header.stamp = ros::Time::now();
        goal1.pose.position.x = 5.0;
        goal1.pose.position.y = 0.0;
        goal1.pose.orientation.w = 1.0;

        // Define the second goal
        geometry_msgs::PoseStamped goal2 = goal1;
        goal2.pose.position.x = 10.5;
        goal2.pose.position.y = -3.7;

        // Define the third goal
        geometry_msgs::PoseStamped goal3 = goal1;
        goal3.pose.position.x = 9.6;
        goal3.pose.position.y = 0.7;

        // Orient the third goal to face the opposite direction
        double yaw = M_PI;
        geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(yaw);
        goal3.pose.orientation = quaternion;

        // Add the goals to the vector
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

		    // Perform 360-degree rotation
		    perform360Rotation();

		    // Adjust head angle (optional)
		    adjustHeadAngle(-0.5);
		} else {
		    ROS_WARN("[Node B] Failed to reach goal. Retrying...");
		    publishFeedback("The robot failed to reach its goal. Retrying...");
		}
	}


	/**
	 * Adjusts the head tilt angle of the robot.
	 * The tilt angle in radians to move the head.
	 * true if the movement was successful, false if it failed or timed out.
	 */
	bool adjustHeadAngle(double tilt_angle_radians) {
		// Create an action client to control the head
		HeadClient head_client("head_controller/follow_joint_trajectory", true);
		head_client.waitForServer();
		
		// Create a goal to move the head
		control_msgs::FollowJointTrajectoryGoal goal;
		goal.trajectory.joint_names.push_back("head_tilt_joint"); 
		trajectory_msgs::JointTrajectoryPoint point;
		point.positions.push_back(tilt_angle_radians);
		point.time_from_start = ros::Duration(1);
		goal.trajectory.points.push_back(point);
		
		// Send the goal and wait for result with timeout
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
        ROS_INFO("[Node B] Feedback: Current position (x: %f, y: %f)",
                 feedback->base_position.pose.position.x,
                 feedback->base_position.pose.position.y);
    }

    /**
     * Sends the detected cube positions to Node A using the
     */
    void sendCubePositionsToNodeA() {
        assignment_1::SendCubePositions srv;
        geometry_msgs::PoseArray cube_positions;
        cube_positions.header.frame_id = "map";
        cube_positions.header.stamp = ros::Time::now();

        // Iterate over the detected cubes and add their positions to the message
        for (const auto& cube_pose : detected_cubes) {
            cube_positions.poses.push_back(cube_pose.pose);
            ROS_INFO("Cube at (x: %f, y: %f)", cube_pose.pose.position.x, cube_pose.pose.position.y);
        }

        // Set the request message
        srv.request.positions = cube_positions;

        // Call the service and check the response
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
