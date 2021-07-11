#ifndef __PURE_PURSUIT_ROS_H__
#define __PURE_PURSUIT_ROS_H__
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string>
#include <vector>
using std::placeholders::_1;
namespace hl_controllers {

template <typename T1, typename T2> double distance(T1 pt1, T2 pt2) {
  return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) +
              pow(pt1.z - pt2.z, 2));
}

class PurePursuit : public rclcpp::Node {
public:
  PurePursuit();
  ~PurePursuit();

private:
  double ld_gain_;
  double ld_;
  double min_ld_;
  double car_wheel_base_;
  double alpha_;
  double car_speed_;
  int controller_freq_;
  int point_idx_;
  int last_p_idx_;
  double last_dist_ = std::numeric_limits<double>::infinity();
  bool got_path_ = false;
  bool path_done_ = true;
  bool loop_ = false;
  std::string map_frame_ = "map";
  std::string base_frame_ = "base_link";
  builtin_interfaces::msg::Time last_msg_time_;
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  geometry_msgs::msg::PoseStamped target_point_;
  geometry_msgs::msg::PoseStamped lookahead_p;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  void path_callback_(const nav_msgs::msg::Path::SharedPtr msg);
  void odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg);

  OnSetParametersCallbackHandle::SharedPtr parms_clk_;
  rcl_interfaces::msg::SetParametersResult
  params_callback_(const std::vector<rclcpp::Parameter> &parameters);
};
} // namespace hl_controllers
#endif // __PURE_PURSUIT_ROS_H__