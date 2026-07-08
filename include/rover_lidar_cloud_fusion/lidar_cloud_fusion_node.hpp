#ifndef ROVER_LIDAR_CLOUD_FUSION__LIDAR_CLOUD_FUSION_NODE_HPP_
#define ROVER_LIDAR_CLOUD_FUSION__LIDAR_CLOUD_FUSION_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "laser_geometry/laser_geometry.hpp"

namespace rover_lidar_cloud_fusion
{

class LidarCloudFusionNode : public rclcpp::Node
{
public:
  explicit LidarCloudFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declareParameters();
  void loadParameters();
  void setupRosInterfaces();

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void publishMergedCloud();

  std::shared_ptr<sensor_msgs::msg::PointCloud2> transformCloudToTargetFrame(
    const sensor_msgs::msg::PointCloud2 & cloud);

  bool isRecent(const rclcpp::Time & stamp) const;

  void appendCloudXYZ(
    const sensor_msgs::msg::PointCloud2 & input,
    sensor_msgs::PointCloud2Iterator<float> & out_x,
    sensor_msgs::PointCloud2Iterator<float> & out_y,
    sensor_msgs::PointCloud2Iterator<float> & out_z);

private:
  std::string scan_topic_;
  std::string cloud_topic_;
  std::string output_topic_;
  std::string target_frame_;

  double publish_rate_;
  double max_data_age_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  laser_geometry::LaserProjection laser_projector_;

  mutable std::mutex data_mutex_;

  std::shared_ptr<sensor_msgs::msg::PointCloud2> latest_scan_cloud_;
  std::shared_ptr<sensor_msgs::msg::PointCloud2> latest_input_cloud_;

  rclcpp::Time latest_scan_time_;
  rclcpp::Time latest_cloud_time_;
};

}  // namespace rover_lidar_cloud_fusion

#endif  // ROVER_LIDAR_CLOUD_FUSION__LIDAR_CLOUD_FUSION_NODE_HPP_
