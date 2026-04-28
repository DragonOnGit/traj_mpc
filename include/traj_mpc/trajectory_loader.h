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

  bool loadFromXml(const std::string& file_path);

  Eigen::MatrixXd generateReferenceTrajectory(int current_waypoint, int horizon);

  Eigen::MatrixXd generateReferenceTrajectory(double dt, int horizon);

  void resetTrajectoryTime();

  void updateTrajectoryTime(double dt);

  double getCurrentTime() const { return current_time_; }

  const std::vector<Waypoint>& getWaypoints() const { return waypoints_; }
};

} // namespace traj_mpc

#endif // TRAJECTORY_LOADER_H
