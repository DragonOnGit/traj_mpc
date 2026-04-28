#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

namespace traj_mpc {

struct MPCParams {
  int horizon;
  double dt;

  // Per-axis position error weights (P matrix diagonal)
  double weight_pos_x;
  double weight_pos_y;
  double weight_pos_z;

  // Per-axis velocity weights (Q matrix diagonal)
  double weight_vel_x;
  double weight_vel_y;
  double weight_vel_z;

  // Per-axis acceleration weights (R matrix diagonal)
  double weight_acc_x;
  double weight_acc_y;
  double weight_acc_z;

  // Per-axis velocity constraints
  double max_vel_x;
  double max_vel_y;
  double max_vel_z;

  // Per-axis acceleration constraints
  double max_acc_x;
  double max_acc_y;
  double max_acc_z;

  // Second-order model damping ratio per axis
  double damping_x;
  double damping_y;
  double damping_z;

  // Second-order model natural frequency per axis
  double omega_x;
  double omega_y;
  double omega_z;
};

class MPCController {
private:
  MPCParams params_;

  // State vector: [x, y, z, vx, vy, vz] (6-dim)
  Eigen::VectorXd current_state_;

  // Reference trajectory: 6 x horizon matrix
  Eigen::MatrixXd reference_trajectory_;

  // Weight matrices
  Eigen::Matrix3d P_; // Position error weight
  Eigen::Matrix3d Q_; // Velocity weight
  Eigen::Matrix3d R_; // Acceleration weight

  // State space matrices for second-order model per axis
  // x_dot = A * x + B * u
  // where x = [pos, vel], u = [acc]
  Eigen::Matrix2d A_axis_;
  Eigen::Vector2d B_axis_;

  // ROS publisher
  ros::Publisher cmd_vel_pub_;

  // Build state space matrices from second-order model parameters
  void buildStateSpaceModel();

  // Build weight matrices from per-axis parameters
  void buildWeightMatrices();

  // Solve QP problem: min 0.5 * v^T H v + f^T v  s.t. v_lb <= v <= v_ub
  Eigen::VectorXd solveQP(const Eigen::MatrixXd& H, const Eigen::VectorXd& f,
                           const Eigen::VectorXd& v_lb, const Eigen::VectorXd& v_ub);

  // Solve MPC using QP optimization
  Eigen::VectorXd solveMPC();

public:
  MPCController(ros::NodeHandle& nh);
  ~MPCController();

  void updateState(const nav_msgs::Odometry::ConstPtr& odom);
  void updateReference(const Eigen::MatrixXd& ref);
  void computeControl();

  void setParams(const MPCParams& params) { params_ = params; buildWeightMatrices(); buildStateSpaceModel(); }
  const MPCParams& getParams() const { return params_; }
};

} // namespace traj_mpc

#endif // MPC_CONTROLLER_H
