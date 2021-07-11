#include "pure_pursuit/pure_pursuit.hpp"

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hl_controllers::PurePursuit>());
  rclcpp::shutdown();
  return 0;
}