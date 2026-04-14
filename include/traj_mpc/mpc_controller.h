#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

namespace traj_mpc {

class MPCController {
private:
  // MPC parameters
  int horizon_;
  double dt_;
  double weight_pos_;
  double weight_vel_;
  double weight_acc_;
  double max_vel_;
  double max_acc_;
  
  // State and reference
  Eigen::VectorXd current_state_;
  Eigen::MatrixXd reference_trajectory_;
  
  // ROS publisher
  ros::Publisher cmd_vel_pub_;
  
  // Solve MPC problem
  Eigen::VectorXd solveMPC();
  
public:
  MPCController(ros::NodeHandle& nh);
  ~MPCController();
  
  // Update state and reference
  void updateState(const nav_msgs::Odometry::ConstPtr& odom);
  void updateReference(const Eigen::MatrixXd& ref);
  
  // Compute and publish control command
  void computeControl();
  
  // Set MPC parameters
  void setHorizon(int horizon) { horizon_ = horizon; }
  void setDt(double dt) { dt_ = dt; }
  void setWeightPos(double weight) { weight_pos_ = weight; }
  void setWeightVel(double weight) { weight_vel_ = weight; }
  void setWeightAcc(double weight) { weight_acc_ = weight; }
  void setMaxVel(double max_vel) { max_vel_ = max_vel; }
  void setMaxAcc(double max_acc) { max_acc_ = max_acc; }
};

} // namespace traj_mpc

#endif // MPC_CONTROLLER_H