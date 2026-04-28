#include <traj_mpc/mpc_controller.h>
#include <algorithm>

namespace traj_mpc {

MPCController::MPCController(ros::NodeHandle& nh)
    : prediction_matrices_valid_(false) {
  std::string ns = ros::this_node::getName();

  nh.param(ns + "/horizon", params_.horizon, 10);
  nh.param(ns + "/dt", params_.dt, 0.1);

  nh.param(ns + "/weight_pos_x", params_.weight_pos_x, 10.0);
  nh.param(ns + "/weight_pos_y", params_.weight_pos_y, 10.0);
  nh.param(ns + "/weight_pos_z", params_.weight_pos_z, 10.0);

  nh.param(ns + "/weight_vel_x", params_.weight_vel_x, 1.0);
  nh.param(ns + "/weight_vel_y", params_.weight_vel_y, 1.0);
  nh.param(ns + "/weight_vel_z", params_.weight_vel_z, 1.0);

  nh.param(ns + "/weight_acc_x", params_.weight_acc_x, 0.1);
  nh.param(ns + "/weight_acc_y", params_.weight_acc_y, 0.1);
  nh.param(ns + "/weight_acc_z", params_.weight_acc_z, 0.1);

  nh.param(ns + "/max_vel_x", params_.max_vel_x, 2.0);
  nh.param(ns + "/max_vel_y", params_.max_vel_y, 2.0);
  nh.param(ns + "/max_vel_z", params_.max_vel_z, 2.0);

  nh.param(ns + "/max_acc_x", params_.max_acc_x, 3.0);
  nh.param(ns + "/max_acc_y", params_.max_acc_y, 3.0);
  nh.param(ns + "/max_acc_z", params_.max_acc_z, 3.0);

  nh.param(ns + "/gravity", params_.gravity, 9.81);
  nh.param(ns + "/need_gravity_compensation", params_.need_gravity_compensation, false);

  current_state_ = Eigen::VectorXd::Zero(6);
  reference_trajectory_ = Eigen::MatrixXd::Zero(6, params_.horizon);

  buildWeightMatrices();
  buildStateSpaceModel();
  buildPredictionMatrices();

  ROS_INFO("=== MPC Parameters (First-Order Mass Point Model) ===");
  ROS_INFO("horizon: %d, dt: %.3f", params_.horizon, params_.dt);
  ROS_INFO("weight_pos: [%.2f, %.2f, %.2f]",
           params_.weight_pos_x, params_.weight_pos_y, params_.weight_pos_z);
  ROS_INFO("weight_vel: [%.2f, %.2f, %.2f]",
           params_.weight_vel_x, params_.weight_vel_y, params_.weight_vel_z);
  ROS_INFO("weight_acc: [%.2f, %.2f, %.2f]",
           params_.weight_acc_x, params_.weight_acc_y, params_.weight_acc_z);
  ROS_INFO("max_vel: [%.2f, %.2f, %.2f]",
           params_.max_vel_x, params_.max_vel_y, params_.max_vel_z);
  ROS_INFO("max_acc: [%.2f, %.2f, %.2f]",
           params_.max_acc_x, params_.max_acc_y, params_.max_acc_z);
  ROS_INFO("gravity: %.2f, need_gravity_compensation: %s",
           params_.gravity, params_.need_gravity_compensation ? "true" : "false");
  ROS_INFO("=====================================================");
}

MPCController::~MPCController() {}

void MPCController::buildWeightMatrices() {
  P_ = Eigen::Matrix3d::Zero();
  P_.diagonal() << params_.weight_pos_x, params_.weight_pos_y, params_.weight_pos_z;

  Q_ = Eigen::Matrix3d::Zero();
  Q_.diagonal() << params_.weight_vel_x, params_.weight_vel_y, params_.weight_vel_z;

  R_ = Eigen::Matrix3d::Zero();
  R_.diagonal() << params_.weight_acc_x, params_.weight_acc_y, params_.weight_acc_z;
}

void MPCController::buildStateSpaceModel() {
  double dt = params_.dt;

  A_axis_ << 1.0, dt,
             0.0, 1.0;

  B_axis_ << 0.5 * dt * dt,
             dt;

  A_full_ = Eigen::MatrixXd::Zero(6, 6);
  B_full_ = Eigen::MatrixXd::Zero(6, 3);

  for (int i = 0; i < 3; i++) {
    A_full_.block(2 * i, 2 * i, 2, 2) = A_axis_;
    B_full_.block(2 * i, i, 2, 1) = B_axis_;
  }

  prediction_matrices_valid_ = false;
}

void MPCController::buildPredictionMatrices() {
  int N = params_.horizon;
  int n_state = 6;
  int n_input = 3;

  std::vector<Eigen::MatrixXd> A_pows(N + 1);
  A_pows[0] = Eigen::MatrixXd::Identity(n_state, n_state);
  for (int k = 1; k <= N; k++) {
    A_pows[k] = A_pows[k - 1] * A_full_;
  }

  F_ = Eigen::MatrixXd(n_state * N, n_state);
  G_ = Eigen::MatrixXd::Zero(n_state * N, n_input * N);

  for (int k = 0; k < N; k++) {
    F_.block(n_state * k, 0, n_state, n_state) = A_pows[k + 1];

    for (int i = 0; i <= k; i++) {
      G_.block(n_state * k, n_input * i, n_state, n_input) =
          A_pows[k - i] * B_full_;
    }
  }

  prediction_matrices_valid_ = true;
}

Eigen::VectorXd MPCController::solveQP(const Eigen::MatrixXd& H,
                                        const Eigen::VectorXd& f,
                                        const Eigen::VectorXd& v_lb,
                                        const Eigen::VectorXd& v_ub) {
  int n = H.rows();
  Eigen::VectorXd v = -0.5 * H.ldlt().solve(f);

  for (int i = 0; i < n; i++) {
    v(i) = std::max(v_lb(i), std::min(v(i), v_ub(i)));
  }

  double alpha = 1.0 / (H.diagonal().maxCoeff() + 1e-6);
  int max_iter = 300;
  double tol = 1e-8;

  for (int iter = 0; iter < max_iter; iter++) {
    Eigen::VectorXd grad = H * v + f;
    Eigen::VectorXd v_new = v - alpha * grad;

    for (int i = 0; i < n; i++) {
      v_new(i) = std::max(v_lb(i), std::min(v_new(i), v_ub(i)));
    }

    double diff = (v_new - v).norm();
    v = v_new;

    if (diff < tol) {
      break;
    }
  }

  return v;
}

Eigen::VectorXd MPCController::solveMPC() {
  int N = params_.horizon;
  int n_state = 6;
  int n_input = 3;

  if (reference_trajectory_.cols() < 1) {
    return Eigen::VectorXd::Zero(n_input);
  }

  if (!prediction_matrices_valid_) {
    buildPredictionMatrices();
  }

  Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(n_state * N, n_state * N);
  for (int k = 0; k < N; k++) {
    Eigen::Matrix<double, 6, 6> Q_step = Eigen::Matrix<double, 6, 6>::Zero();
    Q_step.block<3, 3>(0, 0) = P_;
    Q_step.block<3, 3>(3, 3) = Q_;
    Q_bar.block(n_state * k, n_state * k, n_state, n_state) = Q_step;
  }

  Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(n_input * N, n_input * N);
  for (int k = 0; k < N; k++) {
    R_bar.block(n_input * k, n_input * k, n_input, n_input) = R_;
  }

  Eigen::VectorXd X_ref = Eigen::VectorXd::Zero(n_state * N);
  for (int k = 0; k < N; k++) {
    int ref_idx = std::min(k, (int)reference_trajectory_.cols() - 1);
    X_ref.segment(n_state * k, 3) = reference_trajectory_.block(0, ref_idx, 3, 1);
    X_ref.segment(n_state * k + 3, 3) = reference_trajectory_.block(3, ref_idx, 3, 1);
  }

  Eigen::VectorXd x0 = current_state_;
  Eigen::MatrixXd H = 2.0 * (G_.transpose() * Q_bar * G_ + R_bar);
  Eigen::VectorXd f = 2.0 * G_.transpose() * Q_bar * (F_ * x0 - X_ref);

  Eigen::VectorXd u_lb(n_input * N);
  Eigen::VectorXd u_ub(n_input * N);
  for (int k = 0; k < N; k++) {
    u_lb.segment(n_input * k, 3) << -params_.max_acc_x, -params_.max_acc_y, -params_.max_acc_z;
    u_ub.segment(n_input * k, 3) << params_.max_acc_x, params_.max_acc_y, params_.max_acc_z;
  }

  Eigen::Vector3d vel_current = current_state_.tail(3);
  Eigen::Vector3d vel_lb, vel_ub;
  vel_lb << -params_.max_vel_x, -params_.max_vel_y, -params_.max_vel_z;
  vel_ub << params_.max_vel_x, params_.max_vel_y, params_.max_vel_z;

  for (int i = 0; i < 3; i++) {
    double acc_from_vel_lb = (vel_lb(i) - vel_current(i)) / params_.dt;
    double acc_from_vel_ub = (vel_ub(i) - vel_current(i)) / params_.dt;
    u_lb(i) = std::max(u_lb(i), acc_from_vel_lb);
    u_ub(i) = std::min(u_ub(i), acc_from_vel_ub);
  }

  Eigen::VectorXd U_opt = solveQP(H, f, u_lb, u_ub);

  return U_opt.head(n_input);
}

void MPCController::updateState(const nav_msgs::Odometry::ConstPtr& odom) {
  current_state_(0) = odom->pose.pose.position.x;
  current_state_(1) = odom->pose.pose.position.y;
  current_state_(2) = odom->pose.pose.position.z;
  current_state_(3) = odom->twist.twist.linear.x;
  current_state_(4) = odom->twist.twist.linear.y;
  current_state_(5) = odom->twist.twist.linear.z;
}

void MPCController::updateReference(const Eigen::MatrixXd& ref) {
  reference_trajectory_ = ref;
}

Eigen::Vector3d MPCController::computeAcceleration() {
  Eigen::VectorXd acc = solveMPC();
  return acc.head(3);
}

} // namespace traj_mpc
