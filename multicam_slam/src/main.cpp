#include <rclcpp/rclcpp.hpp>
#include "multistereo_planning_cloud/multistereo_planning_cloud_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // IMPORTANT: do not override arguments or parameters here.
  rclcpp::NodeOptions options;

  auto node = std::make_shared<multicam_depth::MultiStereoPlanningCloudNode>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
