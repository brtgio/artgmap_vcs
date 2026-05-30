#include "artgmap_vcs/MapRosConection.hpp"
#include <tf2/utils.h>
#include <iostream>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

MapRosConection::MapRosConection(AStar& astarRef)
: rclcpp::Node("map_ros_connection"), astar(astarRef)
{
    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    target_position_pub = this->create_publisher<geometry_msgs::msg::Pose2D>("target_a_star_desired_position", 10);
    ram_pub = this->create_publisher<std_msgs::msg::Int32>("ram_usage_kb", 10);
    time_pub = this->create_publisher<builtin_interfaces::msg::Time>("time_topic", 10);

    sonar_sub = this->create_subscription<geometry_msgs::msg::Point32>(
        "sonarRawdata_bag", 10, 
        std::bind(&MapRosConection::sonarPointReceiver, this, std::placeholders::_1));

    kinnect_sub = this->create_subscription<geometry_msgs::msg::Point32>(
        "/lidar_point_inertial", 10, 
        std::bind(&MapRosConection::pointCallback, this, std::placeholders::_1));

    wmrpose_sub = this->create_subscription<geometry_msgs::msg::Point32>(
        "wmrPose", 10, 
        std::bind(&MapRosConection::wmrPoseReceiver, this, std::placeholders::_1));

    qcar_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/qcar_pose_ekf", 10,
        std::bind(&MapRosConection::qcarPoseReceiver, this, std::placeholders::_1));

    lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 
        rclcpp::SensorDataQoS(),
        std::bind(&MapRosConection::lidarPointReceiver, this, std::placeholders::_1));

    joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "/qcar2_joint",  // topic name
      10,              // queue size
      std::bind(&MapRosConection::joint_sateCallback, this, std::placeholders::_1)
    );

    last_joy_time = this->get_clock()->now();

    ram_timer = this->create_wall_timer(
        std::chrono::seconds(1), 
        std::bind(&MapRosConection::publishMemoryUsage, this));
}

const std::vector<geometry_msgs::msg::Point32>& MapRosConection::getSonarPoints() const
{
    return sonarPoints;
}

const std::vector<geometry_msgs::msg::Point32>& MapRosConection::getKinnectPoints() const
{
    return kinnectPoints;
}

const std::vector<geometry_msgs::msg::Point32>& MapRosConection::getLidarPoints() const
{
    std::lock_guard<std::mutex> lock(lidarMutex);
    return lidarPoints;
}

 geometry_msgs::msg::Point32& MapRosConection::getWmrPose() 
{
    return wmrPose;
    
}

void MapRosConection::sonarPointReceiver(const geometry_msgs::msg::Point32::SharedPtr msg)
{
    sonarPoints.push_back(*msg);
}

void MapRosConection::wmrPoseReceiver(const geometry_msgs::msg::Point32::SharedPtr msg)
{
}

void MapRosConection::qcarPoseReceiver(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{

    this->currentPose.x = msg->pose.position.x;
    this->currentPose.y = msg->pose.position.y;

    this->currentPose.theta =
        tf2::getYaw(msg->pose.orientation);
        // Debug: Show converted values
    
    lastPose.x = this->currentPose.x;
    lastPose.y = this->currentPose.y;
    lastPose.theta  = this->currentPose.theta;

    this->hasNewPose = true;


}

inline long MapRosConection::getMemoryUsageKB() {
    std::ifstream status("/proc/self/status");
    std::string line;
    while (std::getline(status, line)) {
        if (line.rfind("VmRSS:", 0) == 0) {
            long kb;
            sscanf(line.c_str(), "VmRSS: %ld kB", &kb);
            return kb;
        }
    }
    return -1;
}

inline void MapRosConection::publishMemoryUsage() {
    std_msgs::msg::Int32 msg;
    msg.data = static_cast<int>(getMemoryUsageKB());
    ram_pub->publish(msg);
}

void MapRosConection::pointCallback(const geometry_msgs::msg::Point32::SharedPtr msg)
{
    kinnectPoints.push_back(*msg);
}

void MapRosConection::resetNavigation()
{
    index = 0;
    isPathExecuting = false;
    currentPath.clear();
    t_update = 0.0f;
    d = 0.0f;
}

void MapRosConection::setPath(const std::vector<sf::Vector2f>& path){
    currentPath = path;
}

void MapRosConection::lidarPointReceiver(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    float x_odom, y_odom, theta_odom;
    {
        std::lock_guard<std::mutex> lock(lidarMutex);
        x_odom = this->currentPose.x;
        y_odom = this->currentPose.y;
        theta_odom = this->currentPose.theta;
    }

    std::lock_guard<std::mutex> lock(lidarMutex);
    lidarPoints.clear(); 
    lidarPoints.reserve(scan->ranges.size() / 20);

    // Keep the exact structural inversion that kept your walls straight
    const float rot_inversion = -1.0f; 

    // Use i += 20 to step through readings
    for (size_t i = 0; i < scan->ranges.size(); i += 20) {
        float range = scan->ranges[i];

        // Enforce the 10 cm range threshold check here
        if (std::isfinite(range) && range >= scan->range_min && range <= scan->range_max && range >= 0.1f) {
            
            float angle = scan->angle_min + (i * scan->angle_increment);

            float lx = range * std::cos(angle);
            float ly = range * std::sin(angle);

            // Restored the exact physical transformation that worked in your best map
            float bx = (rot_inversion * lx);
            float by = (rot_inversion * ly);

            geometry_msgs::msg::Point32 p;
            // Projecting to Global Frame (Odom) using your original working matrix equations
            p.x = (bx * std::cos(theta_odom)) - (by * std::sin(theta_odom)) + x_odom;
            p.y = (bx * std::sin(theta_odom)) + (by * std::cos(theta_odom)) + y_odom;
            p.z = 0.0f;
            
            lidarPoints.push_back(p);
        }
    }
}

void MapRosConection::joint_sateCallback(const sensor_msgs::msg::JointState::SharedPtr joint){
    
    moving = false;  
    
    for(size_t i = 0; i < joint->velocity.size(); i++){
        if(joint->velocity[i] != 0.0){
            moving = true;
            break;
        }
    }
}