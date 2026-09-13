#include <csignal>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "robot_sdk_driver/robot_sdk_node.hpp"

static std::shared_ptr<robot_sdk_driver::RobotSdkNode> g_node;

void signalHandler(int) {
  if (g_node) {
    RCLCPP_INFO(g_node->get_logger(), "Received shutdown signal, stopping robot...");
    g_node->shutdownRobot();
  }
  rclcpp::shutdown();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<robot_sdk_driver::RobotSdkNode>();
  g_node = node;

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  rclcpp::spin(node);

  node->shutdownRobot();
  rclcpp::shutdown();
  return 0;
}
