#pragma once

#include <deque>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using ImuMsg = sensor_msgs::msg::Imu;
using JpegImageMsg = sensor_msgs::msg::CompressedImage;

struct MessageGroup
{
    PointCloudMsg::SharedPtr lidar;
    std::vector<ImuMsg::SharedPtr> imu;
    // accumulate all cameras to 1 queue. Check frame id in header to map to correct extrinsics
    std::vector<JpegImageMsg::SharedPtr> cameras;
};

class SynchronizeNode : public rclcpp::Node
{
  public:
    SynchronizeNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~SynchronizeNode();

    void setCallback(std::function<void(MessageGroup &)> callback);

  private:
    void lidar_callback(const PointCloudMsg::SharedPtr msg);
    void imu_callback(const ImuMsg::SharedPtr msg);
    void camera_callback(const JpegImageMsg::SharedPtr msg);
    void syncLoop();
    void defaultSyncedCallback(MessageGroup &msg_group);

    std::function<void(MessageGroup &)> syncedCallback = nullptr;
    bool syncThreadRunning = true;
    std::thread *syncThread;
    int camera_num;
    double lidar_scan_time;
    double camera_lidar_time_offset;
    std::deque<PointCloudMsg::SharedPtr> lidar_buffer;
    std::deque<ImuMsg::SharedPtr> imu_buffer;
    std::deque<JpegImageMsg::SharedPtr> camera_buffer;
    rclcpp::Subscription<ImuMsg>::SharedPtr imu_subscriber;
    rclcpp::Subscription<PointCloudMsg>::SharedPtr lidar_subscriber;
    std::vector<rclcpp::Subscription<JpegImageMsg>::SharedPtr> camera_subscribers;
    std::mutex sync_mutex;
    std::condition_variable syncCv;
};