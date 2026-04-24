#include <traj_mpc/mpc_controller.h>

namespace traj_mpc {

MPCController::MPCController(ros::NodeHandle& nh) {
  // Initialize parameters from ROS params
  nh.param("horizon", horizon_, 10);
  nh.param("dt", dt_, 0.1);
  nh.param("weight_pos", weight_pos_, 10.0);
  nh.param("weight_vel", weight_vel_, 1.0);
  nh.param("weight_acc", weight_acc_, 0.1);
  nh.param("max_vel", max_vel_, 2.0);
  nh.param("max_acc", max_acc_, 1.0);
  
  // Initialize state
  current_state_ = Eigen::VectorXd::Zero(6); // [x, y, z, vx, vy, vz]
  
  // Initialize reference trajectory
  reference_trajectory_ = Eigen::MatrixXd::Zero(6, horizon_);
  
  // Initialize publisher for velocity control
  std::string cmd_vel_topic;
  nh.param("cmd_vel_topic", cmd_vel_topic, std::string("/cmd_vel"));
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 10);
}

MPCController::~MPCController() {}

void MPCController::updateState(const nav_msgs::Odometry::ConstPtr& odom) {
  // Update position
  current_state_(0) = odom->pose.pose.position.x;
  current_state_(1) = odom->pose.pose.position.y;
  current_state_(2) = odom->pose.pose.position.z;
  
  // Update velocity
  current_state_(3) = odom->twist.twist.linear.x;
  current_state_(4) = odom->twist.twist.linear.y;
  current_state_(5) = odom->twist.twist.linear.z;
}

void MPCController::updateReference(const Eigen::MatrixXd& ref) {
  reference_trajectory_ = ref;
}

Eigen::VectorXd MPCController::solveMPC() {
  // Simple MPC solver implementation
  // In a real application, use a proper QP solver
  
  // Calculate desired velocity based on position error
  Eigen::VectorXd desired_vel = Eigen::VectorXd::Zero(3);
  
  if (reference_trajectory_.cols() > 0) {
    Eigen::VectorXd ref_pos = reference_trajectory_.block(0, 0, 3, 1);
    Eigen::VectorXd current_pos = current_state_.head(3);
    
    // Proportional control for position error
    double Kp = 1.0;
    desired_vel = Kp * (ref_pos - current_pos);
    
    // Limit velocity
    for (int i = 0; i < 3; i++) {
      if (desired_vel(i) > max_vel_) desired_vel(i) = max_vel_;
      if (desired_vel(i) < -max_vel_) desired_vel(i) = -max_vel_;
    }
  }
  
  return desired_vel;
}

void MPCController::computeControl() {
  // Solve MPC
  Eigen::VectorXd desired_vel = solveMPC();
  
  // Publish control command
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = desired_vel(0);
  cmd_vel.linear.y = desired_vel(1);
  cmd_vel.linear.z = desired_vel(2);
  
  cmd_vel_pub_.publish(cmd_vel);
}

} // namespace traj_mpc