#include "mini_control/gravity_compensation.h"

GravityCompensationNode::GravityCompensationNode() : Node("gravity_compensation")
{

  this->declare_parameter("urdf_path", "rrbot.urdf");
  std::string urdf_path = this->get_parameter("urdf_path").as_string();
  
  pinocchio::urdf::buildModel(urdf_path, model_);
  RCLCPP_INFO(this->get_logger(), "Model loaded: %s", model_.name.c_str());
  
  data_ = pinocchio::Data(model_);
  
  pinocchio::SE3::Vector3 gravity(0, 0, -9.81);
  model_.gravity.linear(gravity);
  
  // joint state subscriber
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, 
    std::bind(&GravityCompensationNode::joint_state_callback, this, std::placeholders::_1));
  
  //gravity torque publisher
  gravity_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/gravity_torques", 10);
    
  RCLCPP_INFO(this->get_logger(), "Gravity compensation node started");
}

void GravityCompensationNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  
  Eigen::VectorXd q(model_.nv);
  for (size_t i = 0; i < msg->position.size(); ++i) {
    q(i) = msg->position[i];
  }
  
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model_.nv);
  
  Eigen::VectorXd tau_g = pinocchio::rnea(model_, data_, q, v, a);
  
  auto torque_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  torque_msg->data.resize(tau_g.size());
  
  for (int i = 0; i < tau_g.size(); ++i) {
    torque_msg->data[i] = tau_g(i);
  }
  
  gravity_torque_pub_->publish(std::move(torque_msg));
  
  
}

std::string GravityCompensationNode::eigen_vector_to_string(const Eigen::VectorXd& vec) {
  std::ostringstream oss;
  for (int i = 0; i < vec.size(); ++i) {
    oss << vec(i);
    if (i < vec.size() - 1) oss << ", ";
  }
  return oss.str();
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GravityCompensationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}