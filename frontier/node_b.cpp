#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "visualization_msgs/Marker.h"
#include "assignment_1/frontier_utils.hpp" // Ensure this matches your include path
#include <vector>
#include <queue>
#include <string>

class FrontierExplorer {
public:
    FrontierExplorer(ros::NodeHandle& nh)
    : tf_listener_() {
        // Subscriptions
        map_subscriber_ = nh.subscribe("/map", 10, &FrontierExplorer::mapCallback, this);
        apriltag_subscriber_ = nh.subscribe("/tag_detections", 10, &FrontierExplorer::apriltagCallback, this);

        // Publishers
        goal_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/exploration_goal", 10);
        feedback_publisher_ = nh.advertise<std_msgs::String>("/node_b/feedback", 10);
        marker_publisher_ = nh.advertise<visualization_msgs::Marker>("/frontier_markers", 10);

        ROS_INFO("Node B Initialized: Waiting for Map...");
    }

    void explore() {
        if (!map_received_) {
            publishFeedback("Waiting for map...");
            return;
        }

        if (goal_sent_) {
            publishFeedback("Exploring...");
            return;
        }

        // Detect and process frontiers
        detectAndProcessFrontiers(map_);

        if (frontierRegions_.empty()) {
            publishFeedback("Exploration complete. No frontiers left.");
            ROS_INFO("No frontiers left to explore.");
            return;
        }

        // Select the best frontier
        frontierRegion best_frontier = selectBestFrontier();

        // Publish visualization markers for the frontiers
        publishFrontiers(frontierRegions_);

        // Send goal to the selected frontier
        sendGoalToFrontier(best_frontier);
    }

private:
    // ROS Components
    ros::Subscriber map_subscriber_, apriltag_subscriber_;
    ros::Publisher goal_publisher_, feedback_publisher_, marker_publisher_;
    tf::TransformListener tf_listener_;
    tf::StampedTransform tf_transform_;

    nav_msgs::OccupancyGrid map_;
    bool map_received_ = false;
    bool goal_sent_ = false;

    // Frontier data
    std::vector<cell> frontierCellGrid_;
    std::vector<frontierRegion> frontierRegions_;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        map_ = *msg;
        map_received_ = true;
        ROS_INFO("Map received: %d x %d", map_.info.width, map_.info.height);
    }

    void apriltagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
        if (msg->detections.empty()) {
            ROS_INFO_THROTTLE(1, "No Apriltags detected...");
            return;
        }

        for (const auto& detection : msg->detections) {
            ROS_INFO("Apriltag ID detected: %d", detection.id[0]);
            publishFeedback("Apriltag detected with ID: " + std::to_string(detection.id[0]));

            // Transform Apriltag pose to map frame
            geometry_msgs::PoseStamped tag_pose_in_map = transformPose(detection.pose.pose.pose);
            ROS_INFO("Apriltag position (map frame): x: %f, y: %f, z: %f",
                     tag_pose_in_map.pose.position.x,
                     tag_pose_in_map.pose.position.y,
                     tag_pose_in_map.pose.position.z);
        }
    }

    geometry_msgs::PoseStamped transformPose(const geometry_msgs::Pose& tag_pose_camera) {
        geometry_msgs::PoseStamped camera_pose, map_pose;
        camera_pose.header.frame_id = "camera_frame";
        camera_pose.pose = tag_pose_camera;

        try {
            tf_listener_.lookupTransform("map", "camera_frame", ros::Time(0), tf_transform_);
            tf::Stamped<tf::Pose> camera_tf, map_tf;
            tf::poseMsgToTF(camera_pose.pose, camera_tf);
            map_tf.setData(tf_transform_ * camera_tf);
            tf::poseTFToMsg(map_tf, map_pose.pose);
        } catch (tf::TransformException& ex) {
            ROS_WARN("Failed to transform pose: %s", ex.what());
        }

        map_pose.header.frame_id = "map";
        return map_pose;
    }

    void detectAndProcessFrontiers(const nav_msgs::OccupancyGrid& map) {
        // Create a non-const copy of map.data
        std::vector<signed char> occupancyGrid = map.data;

        // Preprocess the map
        std::vector<cell> processedMap = preprocessMap(occupancyGrid, map.info.width, map.info.height, 5);

        // Compute frontier cell grid
        frontierCellGrid_ = computeFrontierCellGrid(processedMap, map.info.width, map.info.height);

        // Compute frontier regions
        frontierRegions_ = computeFrontierRegions(frontierCellGrid_, map.info.width, map.info.height,
                                                  map.info.resolution, map.info.origin.position.x,
                                                  map.info.origin.position.y, 10); // Region size threshold = 10
    }

    frontierRegion selectBestFrontier() {
        // Robot's current position
        double robot_x = 0.0, robot_y = 0.0;
        try {
            tf_listener_.lookupTransform("map", "base_link", ros::Time(0), tf_transform_);
            robot_x = tf_transform_.getOrigin().x();
            robot_y = tf_transform_.getOrigin().y();
        } catch (tf::TransformException& ex) {
            ROS_WARN("Failed to get robot position: %s", ex.what());
        }

        // Select the best frontier
        return selectFrontier(frontierRegions_, 0, robot_x, robot_y);
    }

    void sendGoalToFrontier(const frontierRegion& frontier) {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = frontier.x;
        goal.pose.position.y = frontier.y;
        goal.pose.orientation.w = 1.0;

        ROS_INFO("Sending goal to frontier: x: %f, y: %f", frontier.x, frontier.y);

        // Publish the goal
        goal_publisher_.publish(goal);
        goal_sent_ = true;
    }

    void publishFrontiers(const std::vector<frontierRegion>& regions) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.g = 1.0;
        marker.color.a = 1.0;

        for (const auto& region : regions) {
            geometry_msgs::Point p;
            p.x = region.x;
            p.y = region.y;
            marker.points.push_back(p);
        }

        marker_publisher_.publish(marker);
    }

    void publishFeedback(const std::string& message) {
        std_msgs::String feedback_msg;
        feedback_msg.data = message;
        feedback_publisher_.publish(feedback_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_b");
    ros::NodeHandle nh;

    FrontierExplorer explorer(nh);

    ros::Rate rate(1);
    while (ros::ok()) {
        explorer.explore();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

