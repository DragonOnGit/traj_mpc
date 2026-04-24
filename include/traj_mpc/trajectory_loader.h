#ifndef TRAJECTORY_LOADER_H
#define TRAJECTORY_LOADER_H

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace traj_mpc {

struct Waypoint {
  double x, y, z;
  double qx, qy, qz, qw;
};

class TrajectoryLoader {
private:
  std::vector<Waypoint> waypoints_;
  double current_time_ = 0.0;
  
public:
  TrajectoryLoader();
  ~TrajectoryLoader();
  
  // Load trajectory from XML file
  bool loadFromXml(const std::string& file_path);
  
  // Generate reference trajectory for MPC
  Eigen::MatrixXd generateReferenceTrajectory(double dt, int horizon);
  
  // Reset trajectory time
  void resetTrajectoryTime();
  
  // Update trajectory time
  void updateTrajectoryTime(double dt);
  
  // Get current trajectory time
  double getCurrentTime() const { return current_time_; }
  
  // Get waypoints
  const std::vector<Waypoint>& getWaypoints() const { return waypoints_; }
};

} // namespace traj_mpc

#endif // TRAJECTORY_LOADER_H