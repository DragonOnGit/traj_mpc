#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <traj_mpc/mpc_controller.h>
#include <traj_mpc/trajectory_loader.h>
#include <ros/package.h>
#include <cmath>
#include <algorithm>

namespace traj_mpc {

class TrajMPCNode {
private:
  ros::NodeHandle nh_;

  ros::Subscriber odom_sub_;

  ros::Publisher cmd_vel_pub_;
  ros::Publisher path_pub_;
  ros::Publisher position_exp_pub_;

  ros::Timer control_timer_;
  ros::Timer path_publish_timer_;

  MPCController mpc_controller_;
  TrajectoryLoader trajectory_loader_;

  std::string odom_topic_;
  std::string trajectory_file_;
  std::string cmd_vel_topic_;
  std::string path_topic_;
  std::string position_exp_topic_;

  nav_msgs::Odometry current_odom_;

  enum ControlState { STATE_IDLE, STATE_TRAJECTORY_TRACKING, STATE_COMPLETED } control_state_ = STATE_IDLE;

  size_t current_waypoint_index_ = 0;
  double waypoint_reached_threshold_ = 0.25;

  nav_msgs::Path reference_path_;

  nav_msgs::Odometry expected_position_;
  size_t last_published_waypoint_index_ = 0;

  bool odom_received_ = false;

public:
  TrajMPCNode() : mpc_controller_(nh_) {
    std::string ns = ros::this_node::getName();
    nh_.param(ns + "/odom_topic", odom_topic_, std::string("/odom"));
    nh_.param(ns + "/trajectory_file", trajectory_file_, std::string("trajectories/example_trajectory.xml"));
    nh_.param(ns + "/cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
    nh_.param(ns + "/path_topic", path_topic_, std::string("/path_exp"));
    nh_.param(ns + "/position_exp_topic", position_exp_topic_, std::string("/position_exp"));

    if (odom_topic_.empty()) {
      ROS_ERROR("odom_topic parameter is empty! Using default: /odom");
      odom_topic_ = "/odom";
    }
    if (cmd_vel_topic_.empty()) {
      ROS_ERROR("cmd_vel_topic parameter is empty! Using default: /cmd_vel");
      cmd_vel_topic_ = "/cmd_vel";
    }

    ROS_INFO("=== Topic Configuration ===");
    ROS_INFO("odom_topic: %s", odom_topic_.c_str());
    ROS_INFO("cmd_vel_topic: %s", cmd_vel_topic_.c_str());
    ROS_INFO("path_topic: %s", path_topic_.c_str());
    ROS_INFO("position_exp_topic: %s", position_exp_topic_.c_str());
    ROS_INFO("=========================");

    if (trajectory_file_.find("/") != 0) {
      std::string package_path = ros::package::getPath("traj_mpc");
      trajectory_file_ = package_path + "/" + trajectory_file_;
    }

    if (!trajectory_loader_.loadFromXml(trajectory_file_)) {
      ROS_ERROR("Failed to load trajectory file: %s", trajectory_file_.c_str());
      ros::shutdown();
      return;
    }

    auto waypoints = trajectory_loader_.getWaypoints();
    ROS_INFO("Successfully loaded trajectory with %zu waypoints:", waypoints.size());
    for (size_t i = 0; i < waypoints.size(); i++) {
      ROS_INFO("Waypoint %zu: x=%.2f, y=%.2f, z=%.2f, qx=%.2f, qy=%.2f, qz=%.2f, qw=%.2f",
               i + 1, waypoints[i].x, waypoints[i].y, waypoints[i].z,
               waypoints[i].qx, waypoints[i].qy, waypoints[i].qz, waypoints[i].qw);
    }

    initializeReferencePath();

    odom_sub_ = nh_.subscribe(odom_topic_, 10, &TrajMPCNode::odomCallback, this);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 10);
    position_exp_pub_ = nh_.advertise<nav_msgs::Odometry>(position_exp_topic_, 10);

    double control_rate;
    nh_.param(ns + "/control_rate", control_rate, 10.0);
    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate), &TrajMPCNode::controlCallback, this);

    path_publish_timer_ = nh_.createTimer(ros::Duration(0.1), &TrajMPCNode::publishReferencePath, this);

    initializeExpectedPosition();

    const MPCParams& p = mpc_controller_.getParams();
    ROS_INFO("=== MPC Weight Parameters ===");
    ROS_INFO("P(pos): [%.2f, %.2f, %.2f]", p.weight_pos_x, p.weight_pos_y, p.weight_pos_z);
    ROS_INFO("Q(vel): [%.2f, %.2f, %.2f]", p.weight_vel_x, p.weight_vel_y, p.weight_vel_z);
    ROS_INFO("R(acc): [%.2f, %.2f, %.2f]", p.weight_acc_x, p.weight_acc_y, p.weight_acc_z);
    ROS_INFO("max_vel: [%.2f, %.2f, %.2f]", p.max_vel_x, p.max_vel_y, p.max_vel_z);
    ROS_INFO("max_acc: [%.2f, %.2f, %.2f]", p.max_acc_x, p.max_acc_y, p.max_acc_z);
    ROS_INFO("gravity: %.2f, in_model: %s", p.gravity, p.need_gravity_compensation ? "true" : "false");
    ROS_INFO("============================");

    ROS_INFO("Traj MPC Node initialized (First-Order Mass Point Model)");
  }

  void initializeExpectedPosition() {
    auto waypoints = trajectory_loader_.getWaypoints();
    if (!waypoints.empty()) {
      Waypoint first_wp = waypoints[0];
      expected_position_.header.frame_id = "camera_init";
      expected_position_.header.stamp = ros::Time::now();
      expected_position_.pose.pose.position.x = first_wp.x;
      expected_position_.pose.pose.position.y = first_wp.y;
      expected_position_.pose.pose.position.z = first_wp.z;
      expected_position_.pose.pose.orientation.x = first_wp.qx;
      expected_position_.pose.pose.orientation.y = first_wp.qy;
      expected_position_.pose.pose.orientation.z = first_wp.qz;
      expected_position_.pose.pose.orientation.w = first_wp.qw;
      position_exp_pub_.publish(expected_position_);
      last_published_waypoint_index_ = 0;
      ROS_INFO("Initial expected position: x=%.2f, y=%.2f, z=%.2f",
               expected_position_.pose.pose.position.x,
               expected_position_.pose.pose.position.y,
               expected_position_.pose.pose.position.z);
    }
  }

  void publishExpectedPosition(size_t waypoint_index) {
    auto waypoints = trajectory_loader_.getWaypoints();
    if (waypoint_index >= waypoints.size()) {
      return;
    }

    Waypoint wp = waypoints[waypoint_index];
    expected_position_.header.frame_id = "camera_init";
    expected_position_.header.stamp = ros::Time::now();
    expected_position_.pose.pose.position.x = wp.x;
    expected_position_.pose.pose.position.y = wp.y;
    expected_position_.pose.pose.position.z = wp.z;
    expected_position_.pose.pose.orientation.x = wp.qx;
    expected_position_.pose.pose.orientation.y = wp.qy;
    expected_position_.pose.pose.orientation.z = wp.qz;
    expected_position_.pose.pose.orientation.w = wp.qw;

    position_exp_pub_.publish(expected_position_);
    last_published_waypoint_index_ = waypoint_index;
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
  }

  void publishReferencePath(const ros::TimerEvent& event) {
    reference_path_.header.stamp = ros::Time::now();
    path_pub_.publish(reference_path_);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    current_odom_ = *odom;
    mpc_controller_.updateState(odom);
    odom_received_ = true;

    if (control_state_ == STATE_IDLE) {
      ROS_INFO("First odometry received, starting trajectory tracking...");
      control_state_ = STATE_TRAJECTORY_TRACKING;
    }
  }

  void controlCallback(const ros::TimerEvent& event) {
    switch (control_state_) {
      case STATE_IDLE: {
        ROS_INFO_THROTTLE(5.0, "Waiting for odometry data...");
        break;
      }
      case STATE_TRAJECTORY_TRACKING: {
        if (!odom_received_) {
          break;
        }

        double current_x = current_odom_.pose.pose.position.x;
        double current_y = current_odom_.pose.pose.position.y;
        double current_z = current_odom_.pose.pose.position.z;
        double current_vx = current_odom_.twist.twist.linear.x;
        double current_vy = current_odom_.twist.twist.linear.y;
        double current_vz = current_odom_.twist.twist.linear.z;

        auto waypoints = trajectory_loader_.getWaypoints();
        if (waypoints.empty()) {
          ROS_ERROR("No waypoints loaded!");
          break;
        }

        size_t total_waypoints = waypoints.size();

        if (current_waypoint_index_ >= total_waypoints) {
          ROS_INFO("All %zu waypoints completed! Mission finished.", total_waypoints);
          control_state_ = STATE_COMPLETED;
          break;
        }

        Waypoint target_wp = waypoints[current_waypoint_index_];
        double target_x = target_wp.x;
        double target_y = target_wp.y;
        double target_z = target_wp.z;

        double error_x = target_x - current_x;
        double error_y = target_y - current_y;
        double error_z = target_z - current_z;
        double distance = std::sqrt(error_x * error_x + error_y * error_y + error_z * error_z);

        if (distance < waypoint_reached_threshold_) {
          ROS_INFO("=== Waypoint %zu/%zu REACHED ===", current_waypoint_index_ + 1, total_waypoints);
          ROS_INFO("  target=(%.3f, %.3f, %.3f), current=(%.3f, %.3f, %.3f), dist=%.4f m",
                   target_x, target_y, target_z, current_x, current_y, current_z, distance);
          current_waypoint_index_++;

          if (current_waypoint_index_ >= total_waypoints) {
            ROS_INFO("All waypoints completed! Mission finished.");
            control_state_ = STATE_COMPLETED;
          } else {
            publishExpectedPosition(current_waypoint_index_);
          }
        } else {
          if (last_published_waypoint_index_ != current_waypoint_index_) {
            publishExpectedPosition(current_waypoint_index_);
          } else {
            expected_position_.header.stamp = ros::Time::now();
            position_exp_pub_.publish(expected_position_);
          }

          mpc_controller_.updateReference(
              trajectory_loader_.generateReferenceTrajectory(
                  static_cast<int>(current_waypoint_index_),
                  mpc_controller_.getParams().horizon));

          Eigen::Vector3d acc_cmd = mpc_controller_.computeAcceleration();

          double dt = mpc_controller_.getParams().dt;
          Eigen::Vector3d vel_current(current_vx, current_vy, current_vz);
          Eigen::Vector3d vel_cmd = vel_current + acc_cmd * dt;

          double max_vel_x = mpc_controller_.getParams().max_vel_x;
          double max_vel_y = mpc_controller_.getParams().max_vel_y;
          double max_vel_z = mpc_controller_.getParams().max_vel_z;
          vel_cmd(0) = std::max(-max_vel_x, std::min(vel_cmd(0), max_vel_x));
          vel_cmd(1) = std::max(-max_vel_y, std::min(vel_cmd(1), max_vel_y));
          vel_cmd(2) = std::max(-max_vel_z, std::min(vel_cmd(2), max_vel_z));

          geometry_msgs::Twist cmd_vel;
          cmd_vel.linear.x = vel_cmd(0);
          cmd_vel.linear.y = vel_cmd(1);
          cmd_vel.linear.z = vel_cmd(2);
          cmd_vel_pub_.publish(cmd_vel);

          const MPCParams& p = mpc_controller_.getParams();
          ROS_INFO("[WP %zu/%zu] pos_err=(%+.3f,%+.3f,%+.3f) dist=%.3f",
                   current_waypoint_index_ + 1, total_waypoints,
                   error_x, error_y, error_z, distance);
          ROS_INFO("  cur_pos=(%.3f,%.3f,%.3f) tgt_pos=(%.3f,%.3f,%.3f)",
                   current_x, current_y, current_z, target_x, target_y, target_z);
          ROS_INFO("  cur_vel=(%+.3f,%+.3f,%+.3f)",
                   current_vx, current_vy, current_vz);
          ROS_INFO("  mpc_acc=(%+.3f,%+.3f,%+.3f)",
                   acc_cmd(0), acc_cmd(1), acc_cmd(2));
          ROS_INFO("  cmd_vel=(%+.3f,%+.3f,%+.3f)",
                   vel_cmd(0), vel_cmd(1), vel_cmd(2));
          ROS_INFO("  W: P=[%.1f,%.1f,%.1f] Q=[%.1f,%.1f,%.1f] R=[%.2f,%.2f,%.2f]",
                   p.weight_pos_x, p.weight_pos_y, p.weight_pos_z,
                   p.weight_vel_x, p.weight_vel_y, p.weight_vel_z,
                   p.weight_acc_x, p.weight_acc_y, p.weight_acc_z);
          ROS_INFO("  lim: v_max=[%.1f,%.1f,%.1f] a_max=[%.1f,%.1f,%.1f] g=%.2f(%s)",
                   p.max_vel_x, p.max_vel_y, p.max_vel_z,
                   p.max_acc_x, p.max_acc_y, p.max_acc_z,
                   p.gravity, p.need_gravity_compensation ? "in_model" : "off");
        }

        break;
      }
      case STATE_COMPLETED: {
        ROS_INFO_THROTTLE(5.0, "Mission completed. Node will continue running.");

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
