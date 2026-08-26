#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "zed_gait_analysis_recorder/gait_recorder_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<zed_gait::GaitRecorderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
