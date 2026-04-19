#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <traj_mpc/mpc_controller.h>
#include <traj_mpc/trajectory_loader.h>
#include <ros/package.h>

namespace traj_mpc {

class TrajMPCNode {
private:
  ros::NodeHandle nh_;
  
  // Subscribers
  ros::Subscriber odom_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber imu_sub_;
  
  // Publishers
  ros::Publisher setpoint_pos_pub_;
  ros::Publisher setpoint_vel_pub_;
  ros::Publisher cmd_vel_pub_;
  
  // Service clients
  ros::ServiceClient set_mode_client_;
  ros::ServiceClient arming_client_;
  
  // Timers
  ros::Timer control_timer_;
  ros::Timer offboard_timer_;
  
  // Controllers and loaders
  MPCController mpc_controller_;
  TrajectoryLoader trajectory_loader_;
  
  // Parameters
  std::string odom_topic_;
  std::string trajectory_file_;
  
  // MAVROS topic names (configurable via launch file)
  std::string state_topic_;
  std::string battery_topic_;
  std::string imu_topic_;
  std::string setpoint_pos_topic_;
  std::string setpoint_vel_topic_;
  std::string cmd_vel_topic_;
  std::string set_mode_service_;
  std::string arming_service_;
  
  // PX4 state
  mavros_msgs::State current_state_;
  sensor_msgs::BatteryState battery_state_;
  sensor_msgs::Imu imu_data_;
  nav_msgs::Odometry current_odom_;
  
  // Control state
  enum ControlState { STATE_IDLE, STATE_OFFBOARD_SETUP, STATE_ARMING, STATE_TAKEOFF, STATE_HOVERING, STATE_TRAJECTORY_TRACKING, STATE_LANDING } control_state_ = STATE_IDLE;
  
  bool offboard_active_ = false;
  bool armed_ = false;
  ros::Time last_command_time_;
  double command_timeout_ = 1.0; // seconds
  
  // Control setpoints
  geometry_msgs::PoseStamped target_pose_;
  geometry_msgs::Twist target_velocity_;
  
  // Takeoff parameters
  double takeoff_height_ = 1.0; // meters
  double takeoff_timeout_ = 10.0; // seconds
  ros::Time takeoff_start_time_;
  
  // Hover parameters
  double hover_duration_ = 2.0; // seconds
  ros::Time hover_start_time_;
  
  // Trajectory tracking parameters
  size_t current_waypoint_index_ = 0; // Current waypoint index (0 to N-1)
  double waypoint_reached_threshold_ = 0.3; // meters
  double landing_error_threshold_ = 0.2; // meters
  

  
public:
  TrajMPCNode() : mpc_controller_(nh_) {
    // Get parameters from launch file
    nh_.param("odom_topic", odom_topic_, std::string("/mavros/local_position/odom"));
    nh_.param("trajectory_file", trajectory_file_, std::string("trajectories/example_trajectory.xml"));
    nh_.param("command_timeout", command_timeout_, 1.0);
    
    // Get configurable MAVROS topic names (with defaults)
    nh_.param("state_topic", state_topic_, std::string("/mavros/state"));
    nh_.param("battery_topic", battery_topic_, std::string("/mavros/battery"));
    nh_.param("imu_topic", imu_topic_, std::string("/mavros/imu/data"));
    nh_.param("setpoint_pos_topic", setpoint_pos_topic_, std::string("/mavros/setpoint_position/local"));
    nh_.param("setpoint_vel_topic", setpoint_vel_topic_, std::string("/mavros/setpoint_velocity/cmd_vel"));
    nh_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/mavros/setpoint_velocity/cmd_vel_unstamped"));
    nh_.param("set_mode_service", set_mode_service_, std::string("/mavros/set_mode"));
    nh_.param("arming_service", arming_service_, std::string("/mavros/cmd/arming"));
    
    // Validate required parameters
    if (odom_topic_.empty()) {
      ROS_ERROR("odom_topic parameter is empty! Using default: /mavros/local_position/odom");
      odom_topic_ = "/mavros/local_position/odom";
    }
    if (cmd_vel_topic_.empty()) {
      ROS_ERROR("cmd_vel_topic parameter is empty! Using default: /mavros/setpoint_velocity/cmd_vel_unstamped");
      cmd_vel_topic_ = "/mavros/setpoint_velocity/cmd_vel_unstamped";
    }
    if (setpoint_pos_topic_.empty()) {
      ROS_ERROR("setpoint_pos_topic parameter is empty! Using default: /mavros/setpoint_position/local");
      setpoint_pos_topic_ = "/mavros/setpoint_position/local";
    }
    
    ROS_INFO("=== Topic Configuration ===");
    ROS_INFO("odom_topic: %s", odom_topic_.c_str());
    ROS_INFO("state_topic: %s", state_topic_.c_str());
    ROS_INFO("battery_topic: %s", battery_topic_.c_str());
    ROS_INFO("imu_topic: %s", imu_topic_.c_str());
    ROS_INFO("setpoint_pos_topic: %s", setpoint_pos_topic_.c_str());
    ROS_INFO("setpoint_vel_topic: %s", setpoint_vel_topic_.c_str());
    ROS_INFO("cmd_vel_topic: %s", cmd_vel_topic_.c_str());
    ROS_INFO("set_mode_service: %s", set_mode_service_.c_str());
    ROS_INFO("arming_service: %s", arming_service_.c_str());
    ROS_INFO("=========================");
    
    // Convert to absolute path using package path
    if (trajectory_file_.find("/") != 0) { // If not absolute path
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
    
    // Subscribers
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &TrajMPCNode::odomCallback, this);
    state_sub_ = nh_.subscribe(state_topic_, 10, &TrajMPCNode::stateCallback, this);
    battery_sub_ = nh_.subscribe(battery_topic_, 10, &TrajMPCNode::batteryCallback, this);
    imu_sub_ = nh_.subscribe(imu_topic_, 10, &TrajMPCNode::imuCallback, this);
    
    // Publishers
    setpoint_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(setpoint_pos_topic_, 10);
    setpoint_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(setpoint_vel_topic_, 10);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);
    
    // Service clients
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(set_mode_service_);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(arming_service_);
    
    // Timers
    double control_rate;
    nh_.param("control_rate", control_rate, 10.0);
    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate), &TrajMPCNode::controlCallback, this);
    
    // Offboard setpoint timer (at least 2Hz as required by PX4)
    offboard_timer_ = nh_.createTimer(ros::Duration(0.05), &TrajMPCNode::publishOffboardSetpoints, this);
    
    // Initialize target pose with current position
    target_pose_.header.frame_id = "map";
    target_pose_.pose.position.x = 0.0;
    target_pose_.pose.position.y = 0.0;
    target_pose_.pose.position.z = 0.0;
    target_pose_.pose.orientation.w = 1.0;
    
    ROS_INFO("Traj MPC Node initialized");
  }
  
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    current_odom_ = *odom;
    mpc_controller_.updateState(odom);
  }
  
  void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
    
    // Update local state variables
    offboard_active_ = (current_state_.mode == "OFFBOARD");
    armed_ = current_state_.armed;
    
    // Log state changes
    static mavros_msgs::State last_state;
    if (current_state_.mode != last_state.mode) {
      ROS_INFO("Mode changed: %s -> %s", last_state.mode.c_str(), current_state_.mode.c_str());
    }
    if (current_state_.armed != last_state.armed) {
      ROS_INFO("Arming state changed: %s -> %s", last_state.armed ? "armed" : "disarmed", current_state_.armed ? "armed" : "disarmed");
    }
    last_state = current_state_;
  }
  
  void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
    battery_state_ = *msg;
    
    // Check battery level
    if (battery_state_.percentage < 0.2) {
      ROS_WARN("Low battery: %.2f%%", battery_state_.percentage * 100.0);
    }
  }
  
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    imu_data_ = *msg;
  }
  
  bool setOffboardMode() {
    mavros_msgs::SetMode set_mode_srv;
    set_mode_srv.request.custom_mode = "OFFBOARD";
    
    if (set_mode_client_.call(set_mode_srv) && set_mode_srv.response.mode_sent) {
      ROS_INFO("Offboard mode enabled");
      offboard_active_ = true;
      return true;
    } else {
      ROS_ERROR("Failed to set offboard mode");
      return false;
    }
  }
  
  bool armVehicle() {
    mavros_msgs::CommandBool arm_srv;
    arm_srv.request.value = true;
    
    if (arming_client_.call(arm_srv) && arm_srv.response.success) {
      ROS_INFO("Vehicle armed");
      return true;
    } else {
      ROS_ERROR("Failed to arm vehicle");
      return false;
    }
  }
  
  void publishOffboardSetpoints(const ros::TimerEvent& event) {
    // Always publish setpoints, even before offboard mode is active
    // This is required by PX4 to maintain offboard mode
    
    // Check command timeout
    if (ros::Time::now() - last_command_time_ > ros::Duration(command_timeout_)) {
      ROS_WARN("Command timeout, sending hold position");
      // Send current position as setpoint to hold position
      target_pose_.pose = current_odom_.pose.pose;
    }
    
    // Publish position setpoint
    target_pose_.header.stamp = ros::Time::now();
    setpoint_pos_pub_.publish(target_pose_);
  }
  
  void controlCallback(const ros::TimerEvent& event) {
    // Update last command time
    last_command_time_ = ros::Time::now();
    
    // State machine for control flow
    switch (control_state_) {
      case STATE_IDLE: {
        // Start the control sequence
        ROS_INFO("Starting control sequence...");
        control_state_ = STATE_OFFBOARD_SETUP;
        break;
      }
      case STATE_OFFBOARD_SETUP: {
        // Set offboard mode
        static ros::Time start_time = ros::Time::now();
        static int set_mode_attempts = 0;
        const int MAX_ATTEMPTS = 5;
        const double TIMEOUT = 10.0; // seconds
        
        // Wait a bit before first attempt to ensure setpoints are being published
        if (ros::Time::now() - start_time < ros::Duration(2.0)) {
          ROS_INFO_THROTTLE(1.0, "Waiting for setpoints to be published before switching to offboard mode...");
          break;
        }
        
        if (!offboard_active_) {
          if (set_mode_attempts < MAX_ATTEMPTS) {
            if (setOffboardMode()) {
              ROS_INFO("Offboard mode set requested, waiting for confirmation... (Attempt %d/%d)", 
                      set_mode_attempts + 1, MAX_ATTEMPTS);
              set_mode_attempts++;
            } else {
              ROS_WARN("Failed to request offboard mode, retrying... (Attempt %d/%d)", 
                      set_mode_attempts + 1, MAX_ATTEMPTS);
              set_mode_attempts++;
            }
          } else {
            // Max attempts reached
            ROS_ERROR("Failed to switch to offboard mode after %d attempts, timeout after %.1f seconds", 
                     MAX_ATTEMPTS, TIMEOUT);
            ros::shutdown();
            return;
          }
        } else {
          // Offboard mode confirmed
          ROS_INFO("Offboard mode confirmed, proceeding to arming...");
          control_state_ = STATE_ARMING;
        }
        
        // Check for timeout
        if (ros::Time::now() - start_time > ros::Duration(TIMEOUT)) {
          ROS_ERROR("Timeout waiting for offboard mode confirmation after %.1f seconds", TIMEOUT);
          ros::shutdown();
          return;
        }
        break;
      }
      case STATE_ARMING: {
        // Arm the vehicle
        if (!armed_) {
          if (armVehicle()) {
            ROS_INFO("Vehicle armed successfully, waiting for arm confirmation...");
          } else {
            ROS_WARN("Failed to arm vehicle, retrying...");
          }
        } else {
          // Arming confirmed
          ROS_INFO("Vehicle armed, proceeding to takeoff...");
          // Set takeoff target position
          target_pose_.pose.position.x = current_odom_.pose.pose.position.x;
          target_pose_.pose.position.y = current_odom_.pose.pose.position.y;
          target_pose_.pose.position.z = takeoff_height_;
          target_pose_.pose.orientation = current_odom_.pose.pose.orientation;
          
          takeoff_start_time_ = ros::Time::now();
          control_state_ = STATE_TAKEOFF;
        }
        break;
      }
      case STATE_TAKEOFF: {
        // Check if reached takeoff height
        double current_height = current_odom_.pose.pose.position.z;
        double height_error = fabs(current_height - takeoff_height_);
        
        // Update target pose to current position with target height
        target_pose_.pose.position.x = current_odom_.pose.pose.position.x;
        target_pose_.pose.position.y = current_odom_.pose.pose.position.y;
        target_pose_.pose.position.z = takeoff_height_;
        target_pose_.pose.orientation = current_odom_.pose.pose.orientation;
        
        if (height_error < 0.1) {
          // Reached takeoff height
          ROS_INFO("Reached takeoff height (%.2f m), proceeding to hover...", current_height);
          hover_start_time_ = ros::Time::now();
          control_state_ = STATE_HOVERING;
        } else if (ros::Time::now() - takeoff_start_time_ > ros::Duration(takeoff_timeout_)) {
          // Takeoff timeout
          ROS_ERROR("Takeoff timeout, proceeding to hover...");
          hover_start_time_ = ros::Time::now();
          control_state_ = STATE_HOVERING;
        } else {
          // Continue takeoff
          ROS_INFO_THROTTLE(1.0, "Taking off: current height = %.2f m, target = %.2f m, error = %.2f m", 
                          current_height, takeoff_height_, height_error);
        }
        break;
      }
      case STATE_HOVERING: {
        // Hover for specified duration
        if (ros::Time::now() - hover_start_time_ > ros::Duration(hover_duration_)) {
          // Hover complete, start trajectory tracking
          ROS_INFO("Hover complete, starting trajectory tracking...");
          control_state_ = STATE_TRAJECTORY_TRACKING;
        } else {
          // Continue hovering
          ROS_INFO_THROTTLE(1.0, "Hovering...");
        }
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
        size_t N = total_waypoints; // Total number of waypoints
        
        // Check if all waypoints are completed
        if (current_waypoint_index_ >= N) {
          // All waypoints completed, check landing condition
          Waypoint last_waypoint = waypoints.back();
          double last_x_error = fabs(last_waypoint.x - current_x);
          double last_y_error = fabs(last_waypoint.y - current_y);
          double last_z_error = fabs(last_waypoint.z - current_z);
          double last_max_error = std::sqrt(last_x_error * last_x_error + last_y_error * last_y_error + last_z_error * last_z_error);
          
          ROS_INFO("All %zu waypoints reached. Last waypoint error: %.3f m, landing threshold: %.2f m", 
                   N, last_max_error, landing_error_threshold_);
          
          if (last_max_error < landing_error_threshold_) {
            ROS_INFO("Trajectory completed! Starting landing sequence...");
            control_state_ = STATE_LANDING;
          } else {
            // Still not close enough to last waypoint, keep moving toward it
            target_pose_.pose.position.x = last_waypoint.x;
            target_pose_.pose.position.y = last_waypoint.y;
            target_pose_.pose.position.z = last_waypoint.z;
            ROS_INFO_THROTTLE(1.0, "Moving to last waypoint: error = %.3f m", last_max_error);
          }
          break;
        }
        
        // Get current target waypoint (waypoint index starts from 0)
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
                         current_waypoint_index_ + 1, N, target_x, target_y, target_z, current_x, current_y, current_z, distance);
        
        // Check if current waypoint is reached
        if (distance < waypoint_reached_threshold_) {
          ROS_INFO("Waypoint %zu reached! Moving to next waypoint...", current_waypoint_index_ + 1);
          current_waypoint_index_++;
          
          // Check if we just completed the last waypoint
          if (current_waypoint_index_ >= N) {
            ROS_INFO("All waypoints completed! Preparing for landing...");
          }
        } else {
          // Update target pose for position controller
          target_pose_.pose.position.x = target_x;
          target_pose_.pose.position.y = target_y;
          target_pose_.pose.position.z = target_z;
        }
        
        break;
      }
      case STATE_LANDING: {
        // Landing logic
        double current_z = current_odom_.pose.pose.position.z;
        double landing_altitude = 0.1; // Land at 0.1m height
        
        if (current_z > landing_altitude) {
          // Descend at a moderate rate
          geometry_msgs::Twist cmd_vel;
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y = 0.0;
          cmd_vel.linear.z = -0.4; // 0.4 m/s descent rate (faster)
          
          // Publish the command multiple times to ensure it's received
          for (int i = 0; i < 5; i++) {
            cmd_vel_pub_.publish(cmd_vel);
            ros::Duration(0.02).sleep(); // Small delay between publishes
          }
          
          // Also publish position setpoint to ensure OFFBOARD mode is maintained
          geometry_msgs::PoseStamped landing_pose;
          landing_pose.header.stamp = ros::Time::now();
          landing_pose.pose.position.x = current_odom_.pose.pose.position.x;
          landing_pose.pose.position.y = current_odom_.pose.pose.position.y;
          landing_pose.pose.position.z = 0.0; // Land at ground level
          landing_pose.pose.orientation = current_odom_.pose.pose.orientation;
          setpoint_pos_pub_.publish(landing_pose);
          
          ROS_INFO("Landing: current altitude = %.2f m, descending at 0.4 m/s", current_z);
        } else {
          // Landing complete
          ROS_INFO("Landing completed successfully! Final altitude: %.2f m", current_z);
          
          // Disarm the vehicle
          mavros_msgs::CommandBool arm_cmd;
          arm_cmd.request.value = false;
          if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("Vehicle disarmed successfully");
          } else {
            ROS_WARN("Failed to disarm vehicle");
          }
          
          // Shutdown node
          ros::shutdown();
          return;
        }
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