#include <traj_mpc/mpc_controller.h>
#include <algorithm>

namespace traj_mpc {

MPCController::MPCController(ros::NodeHandle& nh) {
  std::string ns = ros::this_node::getName();

  // Load per-axis MPC parameters with namespace
  nh.param(ns + "/horizon", params_.horizon, 10);
  nh.param(ns + "/dt", params_.dt, 0.1);

  // Per-axis position error weights
  nh.param(ns + "/weight_pos_x", params_.weight_pos_x, 10.0);
  nh.param(ns + "/weight_pos_y", params_.weight_pos_y, 10.0);
  nh.param(ns + "/weight_pos_z", params_.weight_pos_z, 10.0);

  // Per-axis velocity weights
  nh.param(ns + "/weight_vel_x", params_.weight_vel_x, 1.0);
  nh.param(ns + "/weight_vel_y", params_.weight_vel_y, 1.0);
  nh.param(ns + "/weight_vel_z", params_.weight_vel_z, 1.0);

  // Per-axis acceleration weights
  nh.param(ns + "/weight_acc_x", params_.weight_acc_x, 0.1);
  nh.param(ns + "/weight_acc_y", params_.weight_acc_y, 0.1);
  nh.param(ns + "/weight_acc_z", params_.weight_acc_z, 0.1);

  // Per-axis velocity constraints
  nh.param(ns + "/max_vel_x", params_.max_vel_x, 2.0);
  nh.param(ns + "/max_vel_y", params_.max_vel_y, 2.0);
  nh.param(ns + "/max_vel_z", params_.max_vel_z, 2.0);

  // Per-axis acceleration constraints
  nh.param(ns + "/max_acc_x", params_.max_acc_x, 1.0);
  nh.param(ns + "/max_acc_y", params_.max_acc_y, 1.0);
  nh.param(ns + "/max_acc_z", params_.max_acc_z, 1.0);

  // Second-order model damping ratio per axis
  nh.param(ns + "/damping_x", params_.damping_x, 0.8);
  nh.param(ns + "/damping_y", params_.damping_y, 0.8);
  nh.param(ns + "/damping_z", params_.damping_z, 0.8);

  // Second-order model natural frequency per axis
  nh.param(ns + "/omega_x", params_.omega_x, 5.0);
  nh.param(ns + "/omega_y", params_.omega_y, 5.0);
  nh.param(ns + "/omega_z", params_.omega_z, 5.0);

  // Initialize state
  current_state_ = Eigen::VectorXd::Zero(6);

  // Initialize reference trajectory
  reference_trajectory_ = Eigen::MatrixXd::Zero(6, params_.horizon);

  // Build weight and state space matrices
  buildWeightMatrices();
  buildStateSpaceModel();

  // Log parameters
  ROS_INFO("=== MPC Parameters ===");
  ROS_INFO("horizon: %d, dt: %.3f", params_.horizon, params_.dt);
  ROS_INFO("weight_pos: [%.2f, %.2f, %.2f]", params_.weight_pos_x, params_.weight_pos_y, params_.weight_pos_z);
  ROS_INFO("weight_vel: [%.2f, %.2f, %.2f]", params_.weight_vel_x, params_.weight_vel_y, params_.weight_vel_z);
  ROS_INFO("weight_acc: [%.2f, %.2f, %.2f]", params_.weight_acc_x, params_.weight_acc_y, params_.weight_acc_z);
  ROS_INFO("max_vel: [%.2f, %.2f, %.2f]", params_.max_vel_x, params_.max_vel_y, params_.max_vel_z);
  ROS_INFO("max_acc: [%.2f, %.2f, %.2f]", params_.max_acc_x, params_.max_acc_y, params_.max_acc_z);
  ROS_INFO("damping: [%.2f, %.2f, %.2f]", params_.damping_x, params_.damping_y, params_.damping_z);
  ROS_INFO("omega: [%.2f, %.2f, %.2f]", params_.omega_x, params_.omega_y, params_.omega_z);
  ROS_INFO("======================");

  // Initialize publisher
  std::string cmd_vel_topic;
  nh.param(ns + "/cmd_vel_topic", cmd_vel_topic, std::string("/cmd_vel"));
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 10);
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
  // Second-order system: x'' + 2*zeta*omega*x' + omega^2*x = omega^2*u
  // State: [pos, vel], Input: [acc]
  // Discrete state space (zero-order hold):
  //   A_axis = [1, dt; 0, 1] (simplified for small dt)
  //   B_axis = [0.5*dt^2; dt]
  //
  // With damping: A_axis = [1, dt; -omega^2*dt, 1-2*zeta*omega*dt]
  //               B_axis = [0; omega^2*dt]
  //
  // Using average damping model:
  double dt = params_.dt;
  double zeta = (params_.damping_x + params_.damping_y + params_.damping_z) / 3.0;
  double omega = (params_.omega_x + params_.omega_y + params_.omega_z) / 3.0;

  A_axis_ << 1.0, dt,
             -omega * omega * dt, 1.0 - 2.0 * zeta * omega * dt;

  B_axis_ << 0.0,
             omega * omega * dt;
}

Eigen::VectorXd MPCController::solveQP(const Eigen::MatrixXd& H,
                                         const Eigen::VectorXd& f,
                                         const Eigen::VectorXd& v_lb,
                                         const Eigen::VectorXd& v_ub) {
  // Projected gradient descent QP solver with box constraints
  // Solves: min 0.5 * v^T H v + f^T v   s.t. v_lb <= v <= v_ub
  //
  // KKT conditions for box-constrained QP:
  //   H*v + f + mu_lb - mu_ub = 0
  //   mu_lb >= 0, mu_ub >= 0
  //   mu_lb^T (v_lb - v) = 0
  //   mu_ub^T (v - v_ub) = 0

  int n = H.rows();
  Eigen::VectorXd v = -0.5 * H.ldlt().solve(f);

  // Project onto box constraints
  for (int i = 0; i < n; i++) {
    v(i) = std::max(v_lb(i), std::min(v(i), v_ub(i)));
  }

  // Iterative projected gradient refinement
  double alpha = 1.0 / (H.diagonal().maxCoeff() + 1e-6);
  int max_iter = 200;
  double tol = 1e-8;

  for (int iter = 0; iter < max_iter; iter++) {
    Eigen::VectorXd grad = H * v + f;
    Eigen::VectorXd v_new = v - alpha * grad;

    // Project onto box constraints
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
  // MPC with QP optimization
  //
  // Second-order discrete model per axis:
  //   [p(k+1)]   [1      dt  ] [p(k)]   [0.5*dt^2] [a(k)]
  //   [v(k+1)] = [-w^2*dt 1-2*z*w*dt] [v(k)] + [w^2*dt  ] [a(k)]
  //
  // For 3 axes, state x = [px, py, pz, vx, vy, vz], input u = [ax, ay, az]
  //
  // Objective: J = sum_{k=0}^{N-1} [ e(k)^T P e(k) + v(k)^T Q v(k) + a(k)^T R a(k) ]
  //   where e(k) = p_ref(k) - p(k) is position error
  //         v(k) is velocity
  //         a(k) is acceleration (control input)
  //
  // Reformulated as QP in terms of velocity v:
  //   J = e^T P e + v^T Q v
  //   where e = p_ref - p_current - v*dt (position error after applying velocity v)
  //
  // Expanding: J = (p_ref - p_current - v*dt)^T P (p_ref - p_current - v*dt) + v^T Q v
  //          = v^T (dt^2 P + Q) v - 2*dt*(p_ref - p_current)^T P v + const
  //
  // This gives QP: min 0.5*v^T H v + f^T v
  //   H = 2*(dt^2 * P + Q)
  //   f = -2*dt*P*(p_ref - p_current)

  int N = params_.horizon;
  double dt = params_.dt;

  if (reference_trajectory_.cols() < 1) {
    return Eigen::VectorXd::Zero(3);
  }

  // Current position and velocity
  Eigen::Vector3d pos_current = current_state_.head(3);
  Eigen::Vector3d vel_current = current_state_.tail(3);

  // Reference position at first step
  Eigen::Vector3d pos_ref = reference_trajectory_.block(0, 0, 3, 1);

  // Position error
  Eigen::Vector3d e0 = pos_ref - pos_current;

  // --- Single-step QP formulation ---
  // J = e^T P e + v^T Q v
  // e = p_ref - p_current - v*dt  (predicted position error after applying v for dt)
  // J = (e0 - v*dt)^T P (e0 - v*dt) + v^T Q v
  //   = v^T (dt^2 P + Q) v - 2*dt*e0^T P v + e0^T P e0
  //
  // QP: min 0.5*v^T H v + f^T v
  // H = 2*(dt^2 * P + Q)
  // f = -2*dt*P*e0

  Eigen::Matrix3d H = 2.0 * (dt * dt * P_ + Q_);
  Eigen::Vector3d f = -2.0 * dt * P_ * e0;

  // Velocity constraints (box constraints)
  Eigen::Vector3d v_lb, v_ub;
  v_lb << -params_.max_vel_x, -params_.max_vel_y, -params_.max_vel_z;
  v_ub <<  params_.max_vel_x,  params_.max_vel_y,  params_.max_vel_z;

  // Acceleration constraints: |a| <= max_acc => |v - v_current|/dt <= max_acc
  // => v_current - max_acc*dt <= v <= v_current + max_acc*dt
  Eigen::Vector3d acc_lb, acc_ub;
  acc_lb << -params_.max_acc_x, -params_.max_acc_y, -params_.max_acc_z;
  acc_ub <<  params_.max_acc_x,  params_.max_acc_y,  params_.max_acc_z;

  Eigen::Vector3d v_acc_lb = vel_current + acc_lb * dt;
  Eigen::Vector3d v_acc_ub = vel_current + acc_ub * dt;

  // Combine constraints: intersection of velocity and acceleration limits
  for (int i = 0; i < 3; i++) {
    v_lb(i) = std::max(v_lb(i), v_acc_lb(i));
    v_ub(i) = std::min(v_ub(i), v_acc_ub(i));
  }

  // Solve QP
  Eigen::VectorXd v_opt = solveQP(H, f, v_lb, v_ub);

  // --- Multi-step MPC with prediction horizon ---
  // Extend to full horizon using second-order model
  // State prediction: x(k+1) = A*x(k) + B*u(k)
  // where x = [p, v] per axis, u = a (acceleration)
  // and v(k+1) = v(k) + a(k)*dt, p(k+1) = p(k) + v(k)*dt + 0.5*a(k)*dt^2

  if (N > 1) {
    // Build full horizon QP
    // Decision variables: [v_0, v_1, ..., v_{N-1}] (3N variables)
    // For each step k: e(k) = p_ref(k) - p(k)
    //   p(k) = p_current + sum_{i=0}^{k-1} v_i * dt
    //   e(k) = p_ref(k) - p_current - dt * sum_{i=0}^{k-1} v_i

    int n_vars = 3 * N;
    Eigen::MatrixXd H_full = Eigen::MatrixXd::Zero(n_vars, n_vars);
    Eigen::VectorXd f_full = Eigen::VectorXd::Zero(n_vars);

    // Build H and f for multi-step
    // J = sum_{k=0}^{N-1} [ e(k)^T P e(k) + v(k)^T Q v(k) ]
    // e(k) = p_ref(k) - p_current - dt * (v_0 + v_1 + ... + v_{k-1}) - 0.5*dt*v_k
    // Simplified: e(k) ≈ p_ref(k) - p_current - dt * sum_{i=0}^{k} v_i

    for (int k = 0; k < N; k++) {
      Eigen::Vector3d ref_k = reference_trajectory_.block(0, std::min(k, (int)reference_trajectory_.cols() - 1), 3, 1);

      // e(k) = ref_k - p_current - dt * sum_{i=0}^{k} v_i
      // Contribution to J from step k:
      //   e(k)^T P e(k) = (ref_k - p_current - dt*sum_{i=0}^{k} v_i)^T P (...)
      //   v(k)^T Q v(k)

      // Gradient of e(k) w.r.t. v_j for j <= k: -dt * P
      // Hessian contribution from e(k)^T P e(k):
      //   For i,j <= k: dt^2 * P (block at positions 3i:3i+3, 3j:3j+3)

      for (int i = 0; i <= k; i++) {
        for (int j = 0; j <= k; j++) {
          H_full.block(3 * i, 3 * j, 3, 3) += dt * dt * P_;
        }
        // Linear term from e(k): -dt * (ref_k - p_current)^T P
        f_full.segment(3 * i, 3) += -dt * P_ * (ref_k - pos_current);
      }

      // Add velocity weight
      H_full.block(3 * k, 3 * k, 3, 3) += Q_;
    }

    // Scale by 2 for QP form: min 0.5*v^T H v + f^T v
    H_full *= 2.0;
    f_full *= 2.0;

    // Box constraints for all steps
    Eigen::VectorXd v_lb_full = Eigen::VectorXd::Zero(n_vars);
    Eigen::VectorXd v_ub_full = Eigen::VectorXd::Zero(n_vars);
    for (int k = 0; k < N; k++) {
      v_lb_full.segment(3 * k, 3) = v_lb;
      v_ub_full.segment(3 * k, 3) = v_ub;
    }

    // Solve full horizon QP
    Eigen::VectorXd v_full = solveQP(H_full, f_full, v_lb_full, v_ub_full);

    // Extract first step velocity
    v_opt = v_full.head(3);
  }

  return v_opt;
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

void MPCController::computeControl() {
  Eigen::VectorXd desired_vel = solveMPC();

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = desired_vel(0);
  cmd_vel.linear.y = desired_vel(1);
  cmd_vel.linear.z = desired_vel(2);

  cmd_vel_pub_.publish(cmd_vel);
}

} // namespace traj_mpc
