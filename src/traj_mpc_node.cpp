#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <traj_mpc/mpc_controller.h>
#include <traj_mpc/trajectory_loader.h>
#include <ros/package.h>
#include <cmath>

namespace traj_mpc {

class TrajMPCNode {
private:
  ros::NodeHandle nh_;
  
  // Subscribers
  ros::Subscriber odom_sub_;
  
  // Publishers
  ros::Publisher setpoint_pos_pub_; // Position control publisher (deprecated)
  ros::Publisher cmd_vel_pub_;     // Velocity control publisher
  ros::Publisher path_pub_;         // Reference trajectory publisher
  
  // Timers
  ros::Timer control_timer_;
  ros::Timer path_publish_timer_;    // Timer for reference trajectory publishing
  
  // Controllers and loaders
  MPCController mpc_controller_;
  TrajectoryLoader trajectory_loader_;
  
  // Parameters
  std::string odom_topic_;
  std::string trajectory_file_;
  std::string setpoint_pos_topic_;  // Position control topic (deprecated)
  std::string cmd_vel_topic_;       // Velocity control topic
  std::string path_topic_;           // Reference trajectory topic
  
  // Control parameters
  double max_velocity_;              // Maximum velocity
  double max_acceleration_;          // Maximum acceleration
  
  // Current state
  nav_msgs::Odometry current_odom_;
  
  // Control state
  enum ControlState { STATE_IDLE, STATE_TRAJECTORY_TRACKING, STATE_COMPLETED } control_state_ = STATE_IDLE;
  
  // Control setpoints
  geometry_msgs::PoseStamped target_pose_;  // Target pose (deprecated for position control)
  
  // Trajectory tracking parameters
  size_t current_waypoint_index_ = 0;
  double waypoint_reached_threshold_ = 0.1;
  
  // Reference path message
  nav_msgs::Path reference_path_;
  
public:
  TrajMPCNode() : mpc_controller_(nh_) {
    // Get parameters from launch file
    nh_.param("odom_topic", odom_topic_, std::string("/odom"));
    nh_.param("trajectory_file", trajectory_file_, std::string("trajectories/example_trajectory.xml"));
    nh_.param("setpoint_pos_topic", setpoint_pos_topic_, std::string("/target_position"));
    nh_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
    nh_.param("path_topic", path_topic_, std::string("/path_exp"));
    
    // Get velocity control parameters
    nh_.param("max_velocity", max_velocity_, 2.0);
    nh_.param("max_acceleration", max_acceleration_, 1.0);
    
    // Validate required parameters
    if (odom_topic_.empty()) {
      ROS_ERROR("odom_topic parameter is empty! Using default: /odom");
      odom_topic_ = "/odom";
    }
    if (cmd_vel_topic_.empty()) {
      ROS_ERROR("cmd_vel_topic parameter is empty! Using default: /cmd_vel");
      cmd_vel_topic_ = "/cmd_vel";
    }
    if (path_topic_.empty()) {
      ROS_ERROR("path_topic parameter is empty! Using default: /path_exp");
      path_topic_ = "/path_exp";
    }
    
    ROS_INFO("=== Topic Configuration ===");
    ROS_INFO("odom_topic: %s", odom_topic_.c_str());
    ROS_INFO("setpoint_pos_topic: %s (deprecated - using velocity control)", setpoint_pos_topic_.c_str());
    ROS_INFO("cmd_vel_topic: %s", cmd_vel_topic_.c_str());
    ROS_INFO("path_topic: %s", path_topic_.c_str());
    ROS_INFO("=========================");
    
    ROS_INFO("=== Velocity Control Parameters ===");
    ROS_INFO("max_velocity: %.2f m/s", max_velocity_);
    ROS_INFO("max_acceleration: %.2f m/s²", max_acceleration_);
    ROS_INFO("=================================");
    
    // Convert to absolute path using package path
    if (trajectory_file_.find("/") != 0) {
      std::string package_path = ros::package::getPath("traj_mpc");
      trajectory_file_ = package_path + "/" + trajectory_file_;
    }
    
    // Load trajectory
    if (!trajectory_loader_.loadFromXml(trajectory_file_)) {
      ROS_ERROR("Failed to load trajectory file: %s", trajectory_file_.c_str());
      ros::shutdown();
      return;
    }
    
    // Output trajectory information
    auto waypoints = trajectory_loader_.getWaypoints();
    ROS_INFO("Successfully loaded trajectory with %zu waypoints:", waypoints.size());
    for (size_t i = 0; i < waypoints.size(); i++) {
      ROS_INFO("Waypoint %zu: x=%.2f, y=%.2f, z=%.2f, qx=%.2f, qy=%.2f, qz=%.2f, qw=%.2f",
               i+1, waypoints[i].x, waypoints[i].y, waypoints[i].z,
               waypoints[i].qx, waypoints[i].qy, waypoints[i].qz, waypoints[i].qw);
    }
    
    // Initialize reference path message
    initializeReferencePath();
    
    // Subscribers
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &TrajMPCNode::odomCallback, this);
    
    // Publishers
    setpoint_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(setpoint_pos_topic_, 10);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 10);
    
    // Timers
    double control_rate;
    nh_.param("control_rate", control_rate, 10.0);
    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate), &TrajMPCNode::controlCallback, this);
    
    // Reference path publish timer (10Hz)
    path_publish_timer_ = nh_.createTimer(ros::Duration(0.1), &TrajMPCNode::publishReferencePath, this);
    
    // Initialize target pose (for reference only)
    target_pose_.header.frame_id = "camera_init";
    target_pose_.pose.position.x = 0.0;
    target_pose_.pose.position.y = 0.0;
    target_pose_.pose.position.z = 0.0;
    target_pose_.pose.orientation.w = 1.0;
    
    ROS_INFO("Traj MPC Node initialized without MAVROS dependencies");
    ROS_INFO("Using velocity control mode with reference trajectory publishing to %s", path_topic_.c_str());
  }
  
  void initializeReferencePath() {
    auto waypoints = trajectory_loader_.getWaypoints();
    reference_path_.header.frame_id = "camera_init";
    reference_path_.poses.clear();
    
    for (size_t i = 0; i < waypoints.size(); i++) {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "camera_init";
      pose.header.stamp = ros::Time::now();
      pose.pose.position.x = waypoints[i].x;
      pose.pose.position.y = waypoints[i].y;
      pose.pose.position.z = waypoints[i].z;
      pose.pose.orientation.x = waypoints[i].qx;
      pose.pose.orientation.y = waypoints[i].qy;
      pose.pose.orientation.z = waypoints[i].qz;
      pose.pose.orientation.w = waypoints[i].qw;
      reference_path_.poses.push_back(pose);
    }
    
    ROS_INFO("Reference path initialized with %zu waypoints", waypoints.size());
  }
  
  void publishReferencePath(const ros::TimerEvent& event) {
    reference_path_.header.stamp = ros::Time::now();
    path_pub_.publish(reference_path_);
  }
  
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    current_odom_ = *odom;
    
    // Update MPC controller state
    mpc_controller_.updateState(odom);
    
    // Start trajectory tracking when first odom message is received
    if (control_state_ == STATE_IDLE) {
      ROS_INFO("First odometry received, starting trajectory tracking...");
      control_state_ = STATE_TRAJECTORY_TRACKING;
    }
  }
  
  void controlCallback(const ros::TimerEvent& event) {
    switch (control_state_) {
      case STATE_IDLE: {
        // Waiting for first odometry message
        ROS_INFO_THROTTLE(5.0, "Waiting for odometry data...");
        break;
      }
      case STATE_TRAJECTORY_TRACKING: {
        // Get current position
        double current_x = current_odom_.pose.pose.position.x;
        double current_y = current_odom_.pose.pose.position.y;
        double current_z = current_odom_.pose.pose.position.z;
        
        // Get waypoints
        auto waypoints = trajectory_loader_.getWaypoints();
        if (waypoints.empty()) {
          ROS_ERROR("No waypoints loaded!");
          break;
        }
        
        size_t total_waypoints = waypoints.size();
        
        // Check if all waypoints are completed
        if (current_waypoint_index_ >= total_waypoints) {
          ROS_INFO("All %zu waypoints completed! Mission finished.", total_waypoints);
          control_state_ = STATE_COMPLETED;
          break;
        }
        
        // Get current target waypoint
        Waypoint target_wp = waypoints[current_waypoint_index_];
        double target_x = target_wp.x;
        double target_y = target_wp.y;
        double target_z = target_wp.z;
        
        // Calculate distance to current target waypoint
        double dx = target_x - current_x;
        double dy = target_y - current_y;
        double dz = target_z - current_z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        
        ROS_INFO_THROTTLE(1.0, "Tracking waypoint %zu/%zu: target=(%.2f, %.2f, %.2f), current=(%.2f, %.2f, %.2f), distance=%.3f m",
                         current_waypoint_index_ + 1, total_waypoints, target_x, target_y, target_z, current_x, current_y, current_z, distance);
        
        // Check if current waypoint is reached
        if (distance < waypoint_reached_threshold_) {
          ROS_INFO("Waypoint %zu reached! Moving to next waypoint...", current_waypoint_index_ + 1);
          current_waypoint_index_++;
          
          // Check if we just completed the last waypoint
          if (current_waypoint_index_ >= total_waypoints) {
            ROS_INFO("All waypoints completed! Mission finished.");
            control_state_ = STATE_COMPLETED;
          }
        } else {
          // ---------------------------
          // VELOCITY CONTROL MODE
          // ---------------------------
          // Calculate desired velocity based on position error
          geometry_msgs::Twist cmd_vel;
          
          // Proportional control for position error
          double Kp = 1.0;
          cmd_vel.linear.x = Kp * dx;
          cmd_vel.linear.y = Kp * dy;
          cmd_vel.linear.z = Kp * dz;
          
          // Limit velocity based on max_velocity parameter
          double vel_magnitude = std::sqrt(
            cmd_vel.linear.x * cmd_vel.linear.x +
            cmd_vel.linear.y * cmd_vel.linear.y +
            cmd_vel.linear.z * cmd_vel.linear.z
          );
          
          if (vel_magnitude > max_velocity_) {
            double scale = max_velocity_ / vel_magnitude;
            cmd_vel.linear.x *= scale;
            cmd_vel.linear.y *= scale;
            cmd_vel.linear.z *= scale;
          }
          
          // Publish velocity control command
          cmd_vel_pub_.publish(cmd_vel);
          
          // ---------------------------
          // POSITION CONTROL MODE (DEPRECATED)
          // ---------------------------
          /*
          // Update target pose for position controller
          target_pose_.header.stamp = ros::Time::now();
          target_pose_.pose.position.x = target_x;
          target_pose_.pose.position.y = target_y;
          target_pose_.pose.position.z = target_z;
          target_pose_.pose.orientation.x = target_wp.qx;
          target_pose_.pose.orientation.y = target_wp.qy;
          target_pose_.pose.orientation.z = target_wp.qz;
          target_pose_.pose.orientation.w = target_wp.qw;
          
          // Publish target position
          setpoint_pos_pub_.publish(target_pose_);
          */
          
          // Compute and publish velocity control using MPC
          mpc_controller_.updateReference(trajectory_loader_.generateReferenceTrajectory(0.1, 10));
          mpc_controller_.computeControl();
        }
        
        break;
      }
      case STATE_COMPLETED: {
        // Mission completed, stop publishing commands
        ROS_INFO_THROTTLE(5.0, "Mission completed. Node will continue running.");
        
        // Publish zero velocity to stop the vehicle
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.linear.y = 0.0;
        stop_cmd.linear.z = 0.0;
        cmd_vel_pub_.publish(stop_cmd);
        break;
      }
    }
  }
};

} // namespace traj_mpc

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_mpc_node");
  traj_mpc::TrajMPCNode node;
  ros::spin();
  return 0;
}
