#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rover_lidar_cloud_fusion/lidar_cloud_fusion_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rover_lidar_cloud_fusion::LidarCloudFusionNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
