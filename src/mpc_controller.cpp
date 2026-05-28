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
  ROS_INFO("gravity: %.2f, gravity_in_model: %s",
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

  // State: x = [px, py, pz, vx, vy, vz] (position-first ordering)
  // Input: u = [ax, ay, az]
  //
  // p(k+1) = p(k) + v(k)*dt + 0.5*a(k)*dt^2
  // v(k+1) = v(k) + a(k)*dt
  //
  // A_full = [I_3   dt*I_3]     B_full = [0.5*dt^2 * I_3]
  //          [0_3   I_3   ]              [dt * I_3       ]

  A_full_ = Eigen::MatrixXd::Zero(6, 6);
  A_full_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  A_full_.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
  A_full_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

  B_full_ = Eigen::MatrixXd::Zero(6, 3);
  B_full_.block<3, 3>(0, 0) = 0.5 * dt * dt * Eigen::Matrix3d::Identity();
  B_full_.block<3, 3>(3, 0) = dt * Eigen::Matrix3d::Identity();

  // Gravity disturbance vector (acts in -Z direction)
  // When need_gravity_compensation=true, the velocity controller does NOT
  // compensate for gravity, so we must include it as a known disturbance:
  //   p_z(k+1) = p_z(k) + v_z(k)*dt + 0.5*a_z(k)*dt^2 - 0.5*g*dt^2
  //   v_z(k+1) = v_z(k) + a_z(k)*dt - g*dt
  d_ = Eigen::VectorXd::Zero(6);
  if (params_.need_gravity_compensation) {
    d_(2) = -0.5 * params_.gravity * dt * dt;
    d_(5) = -params_.gravity * dt;
  }

  prediction_matrices_valid_ = false;
}

void MPCController::buildPredictionMatrices() {
  int N = params_.horizon;
  int n_state = 6;
  int n_input = 3;

  // Precompute powers of A: A^0, A^1, ..., A^N
  std::vector<Eigen::MatrixXd> A_pows(N + 1);
  A_pows[0] = Eigen::MatrixXd::Identity(n_state, n_state);
  for (int k = 1; k <= N; k++) {
    A_pows[k] = A_pows[k - 1] * A_full_;
  }

  // Build prediction matrices:
  // X_pred = F * x0 + G * U + D_vec
  // where X_pred = [x(1); x(2); ...; x(N)]
  //       U = [u(0); u(1); ...; u(N-1)]
  F_ = Eigen::MatrixXd(n_state * N, n_state);
  G_ = Eigen::MatrixXd::Zero(n_state * N, n_input * N);
  D_vec_ = Eigen::VectorXd::Zero(n_state * N);

  for (int k = 0; k < N; k++) {
    // F(k) = A^{k+1}
    F_.block(n_state * k, 0, n_state, n_state) = A_pows[k + 1];

    // G(k, i) = A^{k-i} * B  for i = 0, ..., k
    for (int i = 0; i <= k; i++) {
      G_.block(n_state * k, n_input * i, n_state, n_input) =
          A_pows[k - i] * B_full_;
    }

    // D_vec(k) = sum_{j=0}^{k} A^j * d  (accumulated disturbance)
    Eigen::VectorXd D_k = Eigen::VectorXd::Zero(n_state);
    for (int j = 0; j <= k; j++) {
      D_k += A_pows[j] * d_;
    }
    D_vec_.segment(n_state * k, n_state) = D_k;
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

  // Build block-diagonal weight matrices for the full horizon
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

  // Build reference vector for the full horizon
  // Reference format: [px, py, pz, vx, vy, vz] per step (matches state ordering)
  Eigen::VectorXd X_ref = Eigen::VectorXd::Zero(n_state * N);
  for (int k = 0; k < N; k++) {
    int ref_idx = std::min(k, (int)reference_trajectory_.cols() - 1);
    X_ref.segment(n_state * k, 3) = reference_trajectory_.block(0, ref_idx, 3, 1);
    X_ref.segment(n_state * k + 3, 3) = reference_trajectory_.block(3, ref_idx, 3, 1);
  }

  // QP formulation with gravity disturbance:
  // X_pred = F*x0 + G*U + D_vec
  // J = (X_pred - X_ref)^T Q_bar (X_pred - X_ref) + U^T R_bar U
  //   = U^T (G^T Q_bar G + R_bar) U + 2*(F*x0 + D_vec - X_ref)^T Q_bar G U + const
  //
  // min 0.5*U^T H U + f^T U
  // H = 2*(G^T Q_bar G + R_bar)
  // f = 2*G^T Q_bar (F*x0 + D_vec - X_ref)
  Eigen::VectorXd x0 = current_state_;
  Eigen::MatrixXd H = 2.0 * (G_.transpose() * Q_bar * G_ + R_bar);
  Eigen::VectorXd f = 2.0 * G_.transpose() * Q_bar * (F_ * x0 + D_vec_ - X_ref);

  // Acceleration constraints (box constraints for all steps)
  Eigen::VectorXd u_lb(n_input * N);
  Eigen::VectorXd u_ub(n_input * N);
  for (int k = 0; k < N; k++) {
    u_lb.segment(n_input * k, 3) << -params_.max_acc_x, -params_.max_acc_y, -params_.max_acc_z;
    u_ub.segment(n_input * k, 3) << params_.max_acc_x, params_.max_acc_y, params_.max_acc_z;
  }

  // Velocity constraints: tighten first-step acceleration bounds
  // |v(k+1)| <= max_vel => |v_current + a(0)*dt| <= max_vel
  // => (vel_lb - v_current)/dt <= a(0) <= (vel_ub - v_current)/dt
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
