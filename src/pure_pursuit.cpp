#include "pure_pursuit/pure_pursuit.hpp"

namespace hl_controllers {

PurePursuit::PurePursuit() : Node("pure_pursuit") {
  this->declare_parameter<double>("ld_gain", 1.0);
  this->declare_parameter<double>("min_ld", 0.5);
  this->declare_parameter<double>("car_wheel_base", 0.44);
  this->declare_parameter<int>("controller_freq", 10);
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("base_frame", "base_link");

  ld_gain_ = this->get_parameter("ld_gain").as_double();
  min_ld_ = this->get_parameter("min_ld").as_double();
  car_wheel_base_ = this->get_parameter("car_wheel_base").as_double();
  controller_freq_ = this->get_parameter("controller_freq").as_int();
  map_frame_ = this->get_parameter("map_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();

  parms_clk_ = this->add_on_set_parameters_callback(
      std::bind(&PurePursuit::params_callback_, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 1, std::bind(&PurePursuit::odom_callback_, this, _1));
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 1, std::bind(&PurePursuit::path_callback_, this, _1));
}

rcl_interfaces::msg::SetParametersResult PurePursuit::params_callback_(
    const std::vector<rclcpp::Parameter> &parameters) {
  RCLCPP_INFO(this->get_logger(), "Updating Node Parameters:");
  for (const auto &parm : parameters) {
    RCLCPP_INFO(this->get_logger(), "%s: %s", parm.get_name().c_str(),
                parm.as_string().c_str());
  }
  rcl_interfaces::msg::SetParametersResult res;
  res.reason = "Success";
  res.successful = true;
  return res;
}

void PurePursuit::path_callback_(const nav_msgs::msg::Path::SharedPtr msg) {
  path_ = msg->poses;
  got_path_ = true;
  path_done_ = false;
  point_idx_ = 0;
  double start_end_dist =
      distance(path_[0].pose.position, path_.back().pose.position);
  RCLCPP_INFO(this->get_logger(), "Start to End Distance: %f", start_end_dist);
  RCLCPP_INFO(this->get_logger(), "Min lookup distance: %f", min_ld_);
  if (start_end_dist <= min_ld_) {
    loop_ = true;
    RCLCPP_INFO(this->get_logger(), "Is Loop: True");
  }
}

void PurePursuit::odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg) {
  car_speed_ = msg->twist.twist.linear.x;
  ld_ = std::max(ld_gain_ * car_speed_, min_ld_);
}

PurePursuit::~PurePursuit() {}

} // namespace hl_controllers