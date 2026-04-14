#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <traj_mpc/mpc_controller.h>
#include <traj_mpc/trajectory_loader.h>
#include <ros/package.h>

namespace traj_mpc {

class TrajMPCNode {
private:
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Timer control_timer_;
  
  MPCController mpc_controller_;
  TrajectoryLoader trajectory_loader_;
  
  std::string odom_topic_;
  std::string trajectory_file_;
  
public:
  TrajMPCNode() : mpc_controller_(nh_) {
    // Get parameters
    nh_.param("odom_topic", odom_topic_, std::string("/Odometry"));
    nh_.param("trajectory_file", trajectory_file_, std::string("trajectories/example_trajectory.xml"));
    
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
    
    // Subscribe to odometry
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &TrajMPCNode::odomCallback, this);
    
    // Create control timer
    double control_rate;
    nh_.param("control_rate", control_rate, 10.0);
    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate), &TrajMPCNode::controlCallback, this);
    
    ROS_INFO("Traj MPC Node initialized");
  }
  
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    mpc_controller_.updateState(odom);
  }
  
  void controlCallback(const ros::TimerEvent& event) {
    // Generate reference trajectory
    double dt;
    int horizon;
    nh_.param("dt", dt, 0.1);
    nh_.param("horizon", horizon, 10);
    
    Eigen::MatrixXd ref = trajectory_loader_.generateReferenceTrajectory(dt, horizon);
    mpc_controller_.updateReference(ref);
    
    // Compute and publish control
    mpc_controller_.computeControl();
  }
};

} // namespace traj_mpc

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_mpc_node");
  traj_mpc::TrajMPCNode node;
  ros::spin();
  return 0;
}