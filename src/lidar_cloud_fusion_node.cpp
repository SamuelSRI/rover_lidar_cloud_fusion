#include "rover_lidar_cloud_fusion/lidar_cloud_fusion_node.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

using namespace std::chrono_literals;

namespace rover_lidar_cloud_fusion
{

LidarCloudFusionNode::LidarCloudFusionNode(const rclcpp::NodeOptions & options)
: Node("lidar_cloud_fusion_node", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  latest_scan_time_(0, 0, this->get_clock()->get_clock_type()),
  latest_cloud_time_(0, 0, this->get_clock()->get_clock_type())
{
  declareParameters();
  loadParameters();
  setupRosInterfaces();

  RCLCPP_INFO(this->get_logger(), "lidar_cloud_fusion_node started");
  RCLCPP_INFO(this->get_logger(), "scan_topic: %s", scan_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "cloud_topic: %s", cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "target_frame: %s", target_frame_.c_str());
}

void LidarCloudFusionNode::declareParameters()
{
  this->declare_parameter<std::string>("scan_topic", "/scan");
  this->declare_parameter<std::string>("cloud_topic", "/front_cloud");
  this->declare_parameter<std::string>("output_topic", "/merged_cloud");
  this->declare_parameter<std::string>("target_frame", "base_link");

  this->declare_parameter<double>("publish_rate", 10.0);
  this->declare_parameter<double>("max_data_age", 0.5);
}

void LidarCloudFusionNode::loadParameters()
{
  scan_topic_ = this->get_parameter("scan_topic").as_string();
  cloud_topic_ = this->get_parameter("cloud_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();
  target_frame_ = this->get_parameter("target_frame").as_string();

  publish_rate_ = this->get_parameter("publish_rate").as_double();
  max_data_age_ = this->get_parameter("max_data_age").as_double();

  if (publish_rate_ <= 0.0) {
    throw std::runtime_error("publish_rate must be > 0.0");
  }

  if (max_data_age_ <= 0.0) {
    throw std::runtime_error("max_data_age must be > 0.0");
  }
}

void LidarCloudFusionNode::setupRosInterfaces()
{
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&LidarCloudFusionNode::scanCallback, this, std::placeholders::_1));

  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&LidarCloudFusionNode::cloudCallback, this, std::placeholders::_1));

  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic_,
    rclcpp::SensorDataQoS());

  const auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);

  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
    std::bind(&LidarCloudFusionNode::publishMergedCloud, this));
}

void LidarCloudFusionNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 scan_cloud;

  try {
    laser_projector_.projectLaser(*msg, scan_cloud);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Failed to convert LaserScan to PointCloud2: %s",
      e.what());
    return;
  }

  auto transformed_cloud = transformCloudToTargetFrame(scan_cloud);
  if (!transformed_cloud) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_scan_cloud_ = transformed_cloud;
  latest_scan_time_ = this->now();
}

void LidarCloudFusionNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto transformed_cloud = transformCloudToTargetFrame(*msg);
  if (!transformed_cloud) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_input_cloud_ = transformed_cloud;
  latest_cloud_time_ = this->now();
}

std::shared_ptr<sensor_msgs::msg::PointCloud2>
LidarCloudFusionNode::transformCloudToTargetFrame(const sensor_msgs::msg::PointCloud2 & cloud)
{
  auto transformed_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

  if (cloud.header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Input cloud has an empty frame_id");
    return nullptr;
  }

  if (cloud.header.frame_id == target_frame_) {
    *transformed_cloud = cloud;
    transformed_cloud->header.stamp = this->now();
    transformed_cloud->header.frame_id = target_frame_;
    return transformed_cloud;
  }

  try {
    geometry_msgs::msg::TransformStamped transform =
      tf_buffer_.lookupTransform(
        target_frame_,
        cloud.header.frame_id,
        tf2::TimePointZero,
        100ms);

    tf2::doTransform(cloud, *transformed_cloud, transform);

    transformed_cloud->header.stamp = this->now();
    transformed_cloud->header.frame_id = target_frame_;

    return transformed_cloud;
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Failed to transform cloud from '%s' to '%s': %s",
      cloud.header.frame_id.c_str(),
      target_frame_.c_str(),
      e.what());
    return nullptr;
  }
}

bool LidarCloudFusionNode::isRecent(const rclcpp::Time & stamp) const
{
  if (stamp.nanoseconds() == 0) {
    return false;
  }

  const double age = (this->now() - stamp).seconds();
  return age <= max_data_age_;
}

void LidarCloudFusionNode::publishMergedCloud()
{
  std::shared_ptr<sensor_msgs::msg::PointCloud2> scan_cloud;
  std::shared_ptr<sensor_msgs::msg::PointCloud2> input_cloud;
  rclcpp::Time scan_time(0, 0, this->get_clock()->get_clock_type());
  rclcpp::Time cloud_time(0, 0, this->get_clock()->get_clock_type());

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    scan_cloud = latest_scan_cloud_;
    input_cloud = latest_input_cloud_;
    scan_time = latest_scan_time_;
    cloud_time = latest_cloud_time_;
  }

  const bool use_scan = scan_cloud && isRecent(scan_time);
  const bool use_cloud = input_cloud && isRecent(cloud_time);

  if (!use_scan && !use_cloud) {
    return;
  }

  size_t total_points = 0;

  if (use_scan) {
    total_points += static_cast<size_t>(scan_cloud->width) * static_cast<size_t>(scan_cloud->height);
  }

  if (use_cloud) {
    total_points += static_cast<size_t>(input_cloud->width) * static_cast<size_t>(input_cloud->height);
  }

  if (total_points == 0) {
    return;
  }

  sensor_msgs::msg::PointCloud2 merged_cloud;
  merged_cloud.header.stamp = this->now();
  merged_cloud.header.frame_id = target_frame_;
  merged_cloud.height = 1;
  merged_cloud.width = static_cast<uint32_t>(total_points);
  merged_cloud.is_bigendian = false;
  merged_cloud.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(merged_cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(total_points);

  sensor_msgs::PointCloud2Iterator<float> out_x(merged_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(merged_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(merged_cloud, "z");

  if (use_scan) {
    appendCloudXYZ(*scan_cloud, out_x, out_y, out_z);
  }

  if (use_cloud) {
    appendCloudXYZ(*input_cloud, out_x, out_y, out_z);
  }

  merged_cloud.width = static_cast<uint32_t>(total_points);
  merged_cloud.height = 1;
  merged_cloud.row_step = merged_cloud.point_step * merged_cloud.width;

  cloud_pub_->publish(merged_cloud);
}

void LidarCloudFusionNode::appendCloudXYZ(
  const sensor_msgs::msg::PointCloud2 & input,
  sensor_msgs::PointCloud2Iterator<float> & out_x,
  sensor_msgs::PointCloud2Iterator<float> & out_y,
  sensor_msgs::PointCloud2Iterator<float> & out_z)
{
  sensor_msgs::PointCloud2ConstIterator<float> in_x(input, "x");
  sensor_msgs::PointCloud2ConstIterator<float> in_y(input, "y");
  sensor_msgs::PointCloud2ConstIterator<float> in_z(input, "z");

  const size_t point_count =
    static_cast<size_t>(input.width) * static_cast<size_t>(input.height);

  for (size_t i = 0; i < point_count; ++i, ++in_x, ++in_y, ++in_z) {
    const float x = *in_x;
    const float y = *in_y;
    const float z = *in_z;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    *out_x = x;
    *out_y = y;
    *out_z = z;

    ++out_x;
    ++out_y;
    ++out_z;
  }
}

}  // namespace rover_lidar_cloud_fusion
