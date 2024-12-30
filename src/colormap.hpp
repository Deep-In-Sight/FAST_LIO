#pragma once
#include <condition_variable>
#include <memory>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

typedef pcl::PointXYZRGBNormal PointRGBType;
typedef pcl::PointCloud<PointRGBType> PointCloudXYZRGBN;
typedef sensor_msgs::msg::CompressedImage ImageMsg;
typedef sensor_msgs::msg::PointCloud2 PointCloud2Msg;

class ColormapNode : public rclcpp::Node
{
  public:
    using Ptr = std::shared_ptr<ColormapNode>;

    struct ColormapParams
    {
        bool publish_color_en;
        std::string camera_topic;
        std::string pcd_topic;
        double z_filter;
        double time_offset;
        std::map<std::string, std::vector<double>> intrinsics;
        std::map<std::string, Eigen::Vector3d> extrinsics_T_CI; // from imu to camera
        std::map<std::string, Eigen::Quaterniond> extrinsics_R_CI;
        std::map<std::string, Eigen::Vector2d> fov; // horizontal start and end in degs
    };

    struct FrameGroup
    {
        std::vector<ImageMsg::SharedPtr> imgs;
        PointCloudXYZRGBN::Ptr pcd;
    };

    static ColormapNode::Ptr getInstance();
    ColormapNode(const ColormapNode &) = delete;
    ColormapNode &operator=(const ColormapNode &) = delete;

    bool isInitialized()
    {
        return initialized;
    }
    void queuePointCloud(PointCloudXYZRGBN::Ptr &msg);
    ~ColormapNode();

  private:
    void initParameters();
    void printParameters();
    void cameraCallback(ImageMsg::SharedPtr msg);
    void mapSaveCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    FrameGroup sync();
    void mapPinHole(PointCloudXYZRGBN &pcd, ImageMsg &img, PointCloudXYZRGBN &pcd_color);
    void colorizePointCloud(FrameGroup &g);
    void worker();

    ColormapNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    bool running;

    ColormapParams params;
    PointCloudXYZRGBN global_pcd;

    rclcpp::Subscription<ImageMsg>::SharedPtr image_subscriber;
    rclcpp::Publisher<PointCloud2Msg>::SharedPtr color_publisher;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_service;

    std::deque<PointCloudXYZRGBN::Ptr> pointcloud_queue;
    std::deque<ImageMsg::SharedPtr> image_msg_queue;
    std::mutex mtx;
    std::condition_variable cv;
    std::thread *colorize_thread;

    bool initialized = false;
};