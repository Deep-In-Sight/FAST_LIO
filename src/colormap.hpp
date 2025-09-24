#pragma once
#include <condition_variable>
#include <memory>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

typedef pcl::PointXYZRGBNormal PointRGBType;
typedef pcl::PointCloud<PointRGBType> PointCloudXYZRGBN;

class ColormapNode : public rclcpp::Node
{
  public:
    using Ptr = std::shared_ptr<ColormapNode>;

    struct ColormapParams
    {
        bool compressed_image;
        std::string camera_topic;
        std::string pcd_topic;
        double z_filter;
        double time_offset;
        int frame_rate;
        std::map<std::string, std::vector<double>> intrinsics;
        std::map<std::string, std::vector<double>> distortion;
        std::map<std::string, Eigen::Vector3d> extrinsics_T_CI; // from imu to camera
        std::map<std::string, Eigen::Matrix3d> extrinsics_R_CI;
        std::map<std::string, Eigen::Vector2d> fov; // horizontal start and end in degs
    };

    template<typename ImageType>
    struct FrameGroup
    {
        std::vector<typename ImageType::SharedPtr> imgs;
        PointCloudXYZRGBN::Ptr pcd;
    };

    static ColormapNode::Ptr getInstance();
    ColormapNode(const ColormapNode &) = delete;
    ColormapNode &operator=(const ColormapNode &) = delete;

    void queuePointCloud(PointCloudXYZRGBN::Ptr &msg);
    void queueOdometry(nav_msgs::msg::Odometry &odom);
    ~ColormapNode();

  private:
    void initParameters();
    void printParameters();
    double poly_eval(const Eigen::VectorXd &coeffs, double x);
    
    template<typename ImageType>
    void cameraCallback(typename ImageType::SharedPtr msg);
    
    void mapSaveCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    template<typename ImageType>
    FrameGroup<ImageType> sync();
    
    template<typename ImageType>
    void mapPinHole(PointCloudXYZRGBN &pcd, ImageType &img, PointCloudXYZRGBN &pcd_color);
    
    template<typename ImageType>
    void colorizePointCloud(FrameGroup<ImageType> &g);
    
    template<typename ImageType>
    void worker();

    ColormapNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    bool running;
    int camera_mode;

    ColormapParams params;
    PointCloudXYZRGBN global_pcd;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr color_publisher;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_service;

    std::deque<PointCloudXYZRGBN::Ptr> pointcloud_queue;
    std::deque<sensor_msgs::msg::CompressedImage::SharedPtr> compressed_image_queue;
    std::deque<sensor_msgs::msg::Image::SharedPtr> image_queue;
    std::deque<nav_msgs::msg::Odometry> odom_queue;
    std::mutex mtx;
    std::condition_variable cv;
    std::thread *colorize_thread;
};