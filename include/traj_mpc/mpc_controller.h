#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace traj_mpc {

struct MPCParams {
  int horizon;
  double dt;

  double weight_pos_x, weight_pos_y, weight_pos_z;
  double weight_vel_x, weight_vel_y, weight_vel_z;
  double weight_acc_x, weight_acc_y, weight_acc_z;

  double max_vel_x, max_vel_y, max_vel_z;
  double max_acc_x, max_acc_y, max_acc_z;

  double gravity;
  bool need_gravity_compensation;
};

class MPCController {
private:
  MPCParams params_;

  Eigen::VectorXd current_state_;
  Eigen::MatrixXd reference_trajectory_;

  Eigen::Matrix3d P_;
  Eigen::Matrix3d Q_;
  Eigen::Matrix3d R_;

  Eigen::MatrixXd A_full_;
  Eigen::MatrixXd B_full_;

  Eigen::VectorXd d_;

  Eigen::MatrixXd F_;
  Eigen::MatrixXd G_;
  Eigen::VectorXd D_vec_;

  bool prediction_matrices_valid_;

  void buildWeightMatrices();
  void buildStateSpaceModel();
  void buildPredictionMatrices();

  Eigen::VectorXd solveQP(const Eigen::MatrixXd& H,
                           const Eigen::VectorXd& f,
                           const Eigen::VectorXd& u_lb,
                           const Eigen::VectorXd& u_ub);

  Eigen::VectorXd solveMPC();

public:
  MPCController(ros::NodeHandle& nh);
  ~MPCController();

  void updateState(const nav_msgs::Odometry::ConstPtr& odom);
  void updateReference(const Eigen::MatrixXd& ref);
  Eigen::Vector3d computeAcceleration();

  void setParams(const MPCParams& params) {
    params_ = params;
    buildWeightMatrices();
    buildStateSpaceModel();
    prediction_matrices_valid_ = false;
  }
  const MPCParams& getParams() const { return params_; }
};

} // namespace traj_mpc

#endif // MPC_CONTROLLER_H
