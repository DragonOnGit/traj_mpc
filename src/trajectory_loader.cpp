#include <traj_mpc/trajectory_loader.h>
#include <tinyxml/tinyxml.h>
#include <iostream>

namespace traj_mpc {

TrajectoryLoader::TrajectoryLoader() {}

TrajectoryLoader::~TrajectoryLoader() {}

bool TrajectoryLoader::loadFromXml(const std::string& file_path) {
  TiXmlDocument doc(file_path.c_str());
  if (!doc.LoadFile()) {
    std::cerr << "Failed to load trajectory file: " << file_path << std::endl;
    return false;
  }
  
  TiXmlElement* root = doc.FirstChildElement("trajectory");
  if (!root) {
    std::cerr << "Invalid trajectory file format" << std::endl;
    return false;
  }
  
  waypoints_.clear();
  
  for (TiXmlElement* point_elem = root->FirstChildElement("point"); 
       point_elem; 
       point_elem = point_elem->NextSiblingElement("point")) {
    Waypoint wp;
    
    // Read position
    if (point_elem->Attribute("x")) {
      wp.x = atof(point_elem->Attribute("x"));
    } else {
      std::cerr << "Missing x attribute in point" << std::endl;
      return false;
    }
    if (point_elem->Attribute("y")) {
      wp.y = atof(point_elem->Attribute("y"));
    } else {
      std::cerr << "Missing y attribute in point" << std::endl;
      return false;
    }
    if (point_elem->Attribute("z")) {
      wp.z = atof(point_elem->Attribute("z"));
    } else {
      std::cerr << "Missing z attribute in point" << std::endl;
      return false;
    }
    
    // Read orientation quaternion
    if (point_elem->Attribute("qx")) {
      wp.qx = atof(point_elem->Attribute("qx"));
    } else {
      std::cerr << "Missing qx attribute in point" << std::endl;
      return false;
    }
    if (point_elem->Attribute("qy")) {
      wp.qy = atof(point_elem->Attribute("qy"));
    } else {
      std::cerr << "Missing qy attribute in point" << std::endl;
      return false;
    }
    if (point_elem->Attribute("qz")) {
      wp.qz = atof(point_elem->Attribute("qz"));
    } else {
      std::cerr << "Missing qz attribute in point" << std::endl;
      return false;
    }
    if (point_elem->Attribute("qw")) {
      wp.qw = atof(point_elem->Attribute("qw"));
    } else {
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
    double t = i * dt;
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

} // namespace traj_mpc