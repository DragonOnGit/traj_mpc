#include <traj_mpc/trajectory_loader.h>
#include <tinyxml2.h>
#include <iostream>

namespace traj_mpc {

TrajectoryLoader::TrajectoryLoader() {}

TrajectoryLoader::~TrajectoryLoader() {}

bool TrajectoryLoader::loadFromXml(const std::string& file_path) {
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(file_path.c_str()) != tinyxml2::XML_SUCCESS) {
    std::cerr << "Failed to load trajectory file: " << file_path << std::endl;
    return false;
  }
  
  tinyxml2::XMLElement* root = doc.FirstChildElement("trajectory");
  if (!root) {
    std::cerr << "Invalid trajectory file format" << std::endl;
    return false;
  }
  
  waypoints_.clear();
  
  for (tinyxml2::XMLElement* point_elem = root->FirstChildElement("point"); 
       point_elem; 
       point_elem = point_elem->NextSiblingElement("point")) {
    Waypoint wp;
    
    // Read position
    if (point_elem->QueryDoubleAttribute("x", &wp.x) != tinyxml2::XML_SUCCESS) {
      std::cerr << "Missing x attribute in point" << std::endl;
      return false;
    }
    if (point_elem->QueryDoubleAttribute("y", &wp.y) != tinyxml2::XML_SUCCESS) {
      std::cerr << "Missing y attribute in point" << std::endl;
      return false;
    }
    if (point_elem->QueryDoubleAttribute("z", &wp.z) != tinyxml2::XML_SUCCESS) {
      std::cerr << "Missing z attribute in point" << std::endl;
      return false;
    }
    
    // Read orientation quaternion
    if (point_elem->QueryDoubleAttribute("qx", &wp.qx) != tinyxml2::XML_SUCCESS) {
      std::cerr << "Missing qx attribute in point" << std::endl;
      return false;
    }
    if (point_elem->QueryDoubleAttribute("qy", &wp.qy) != tinyxml2::XML_SUCCESS) {
      std::cerr << "Missing qy attribute in point" << std::endl;
      return false;
    }
    if (point_elem->QueryDoubleAttribute("qz", &wp.qz) != tinyxml2::XML_SUCCESS) {
      std::cerr << "Missing qz attribute in point" << std::endl;
      return false;
    }
    if (point_elem->QueryDoubleAttribute("qw", &wp.qw) != tinyxml2::XML_SUCCESS) {
      std::cerr << "Missing qw attribute in point" << std::endl;
      return false;
    }
    
    waypoints_.push_back(wp);
  }
  
  if (waypoints_.empty()) {
    std::cerr << "No waypoints found in trajectory file" << std::endl;
    return false;
  }
  
  return true;
}

Eigen::MatrixXd TrajectoryLoader::generateReferenceTrajectory(double dt, int horizon) {
  Eigen::MatrixXd ref(6, horizon);
  
  if (waypoints_.empty()) {
    return ref;
  }
  
  // Simple linear interpolation between waypoints
  for (int i = 0; i < horizon; i++) {
    double t = current_time_ + i * dt;
    double total_time = (waypoints_.size() - 1) * 1.0; // Assume 1 second per waypoint
    
    if (t >= total_time) {
      // Hold last waypoint
      Waypoint wp = waypoints_.back();
      ref(0, i) = wp.x;
      ref(1, i) = wp.y;
      ref(2, i) = wp.z;
      ref(3, i) = 0.0;
      ref(4, i) = 0.0;
      ref(5, i) = 0.0;
    } else {
      // Interpolate between waypoints
      int wp_idx = static_cast<int>(t);
      double alpha = t - wp_idx;
      
      Waypoint wp1 = waypoints_[wp_idx];
      Waypoint wp2 = waypoints_[wp_idx + 1];
      
      ref(0, i) = wp1.x + alpha * (wp2.x - wp1.x);
      ref(1, i) = wp1.y + alpha * (wp2.y - wp1.y);
      ref(2, i) = wp1.z + alpha * (wp2.z - wp1.z);
      ref(3, i) = (wp2.x - wp1.x); // Simple velocity estimate
      ref(4, i) = (wp2.y - wp1.y);
      ref(5, i) = (wp2.z - wp1.z);
    }
  }
  
  return ref;
}

void TrajectoryLoader::resetTrajectoryTime() {
  current_time_ = 0.0;
}

void TrajectoryLoader::updateTrajectoryTime(double dt) {
  current_time_ += dt;
}

} // namespace traj_mpc