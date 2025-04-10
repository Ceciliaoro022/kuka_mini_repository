#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"  // Recursive Newton-Euler Algorithm
#include <iostream>

int main(int argc, char ** argv)
{
  using namespace pinocchio;
  
  std::string urdf_filename = "/home/amir/ros2_ws/src/mini_control/description/urdf/rrbot.urdf";
  
  Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);
  std::cout << "Model name: " << model.name << std::endl;
  
  // Create data required by the algorithms
  Data data(model);

  // Use a random configuration or a specific one
  Eigen::VectorXd q(model.nv);
  q << 3.36229, 0.250251;
  std::cout << "q: " << q.transpose() << std::endl;
  
    
  // Create zero velocity and acceleration vectors
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
  
  // Ensure gravity is properly set (default is 0,0,-9.81)
  pinocchio::SE3::Vector3 gravity(0, 0, -9.81);
  model.gravity.linear(gravity);
  
  // Compute gravity torques using the Recursive Newton-Euler Algorithm
  // When velocity and acceleration are zero, RNEA gives only gravity terms
  Eigen::VectorXd tau_g = rnea(model, data, q, v, a);
  
  // Print gravity torques
  std::cout << "\nGravity torques:" << std::endl;
  for (int i = 1; i < model.njoints; ++i) {
    std::cout << std::setw(24) << std::left << model.names[i] 
              << ": " << tau_g(i-1) << " N·m" << std::endl;
  }
  
  return 0;
}