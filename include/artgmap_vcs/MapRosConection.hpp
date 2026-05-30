#pragma once
/* * -----------------------------------------------------------------------------
 * Copyright (c) 2025 Gilberto Ramos Valenzuela
 * Artgslam Visualizer Project
 * -----------------------------------------------------------------------------
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"      // Changed from PointCloud2
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <SFML/Graphics.hpp>
#include "artgmap_vcs/AStar.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <string>
#include <vector>
#include <mutex>   
#include <fstream>
#include <iostream>
#include <cmath>
#include "Pose.hpp"

class MapRosConection : public rclcpp::Node {
public:
    MapRosConection(AStar& astarRef);

    // Getters for sensors - returns the accumulated points for SFML to draw
    const std::vector<geometry_msgs::msg::Point32>& getSonarPoints() const;
    const std::vector<geometry_msgs::msg::Point32>& getKinnectPoints() const;
    
    // Returns the vector of points parsed from the LaserScan
    const std::vector<geometry_msgs::msg::Point32>& getLidarPoints() const;

    geometry_msgs::msg::Point32& getWmrPose() ;

    double getLinearVelocity() const { return current_linear_velocity; }
    double getAngularVelocity() const { return current_angular_velocity; }
    float getLastDeltaTime() const { return last_dt; }

    void setExecutePathActivity(bool activity){ isPathExecuting = activity;};
    void resetNavigation();
    
    void pointCallback(const geometry_msgs::msg::Point32::SharedPtr msg);
    void setPath(const std::vector<sf::Vector2f>& path);

    const Pose& getPose() const {return lastPose;}
    bool getHasNewPose() const { return hasNewPose; }
    void resetHasNewPose() { hasNewPose = false; }
    bool isRobotMoving() const {return moving;}

private:
    AStar& astar;

    // Velocity scaling and state
    int linear, angular;
    double l_scale, a_scale;
    double current_linear_velocity = 0.0;
    double current_angular_velocity = 0.0;
    float last_dt = 0.0;
    rclcpp::Time last_joy_time; 
    geometry_msgs::msg::Pose2D target_position;
    bool moving = false;
    // Persistent Sensors - these hold the data to be rendered
    std::vector<geometry_msgs::msg::Point32> sonarPoints;
    std::vector<geometry_msgs::msg::Point32> kinnectPoints;
    
    // This vector will now be cleared and repopulated each time a new scan arrives
    std::vector<geometry_msgs::msg::Point32> lidarPoints; 

    // Mutex for Thread Safety between ROS callback and Main SFML Thread
    mutable std::mutex lidarMutex;

    // Robot pose and navigation state
    geometry_msgs::msg::Point32 wmrPose;
    rclcpp::Time lastUpdate;
    bool isPathExecuting = false;
    sf::Vector2f desiredPosition;
    std::vector<sf::Vector2f> currentPath;

    // ROS 2 Communication
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr target_position_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ram_pub;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr time_pub;
    
    rclcpp::Subscription<geometry_msgs::msg::Point32>::SharedPtr sonar_sub;
    rclcpp::Subscription<geometry_msgs::msg::Point32>::SharedPtr kinnect_sub;
    
    // Changed to LaserScan for direct Lidar support
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;

    rclcpp::Subscription<geometry_msgs::msg::Point32>::SharedPtr wmrpose_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr qcar_pose_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;

    rclcpp::TimerBase::SharedPtr ram_timer;


    Pose currentPose;
    bool hasNewPose = false;
    Pose lastPose;
    
    // Callbacks
    void sonarPointReceiver(const geometry_msgs::msg::Point32::SharedPtr sonar);
    
    // Updated signature to handle LaserScan data
    void lidarPointReceiver(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    
    void wmrPoseReceiver(const geometry_msgs::msg::Point32::SharedPtr pose);
    void qcarPoseReceiver(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void joint_sateCallback(const sensor_msgs::msg::JointState::SharedPtr joint);

    long getMemoryUsageKB();
    void publishMemoryUsage();
    float t_update = 0.0f;
    float d = 0.0f;
    size_t pathSize = 0; 
    size_t index = 0;    
};