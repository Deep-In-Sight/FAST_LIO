#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

struct MessageGroup;

class LaserMappingNode : public rclcpp::Node
{
  public:
    LaserMappingNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~LaserMappingNode();

    void enqueueGroup(MessageGroup &group);

  private:
    void init_camera_containers();
    void load_camera_parameters();
    void timer_callback();
    void map_publish_callback();
    void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req,
                           std_srvs::srv::Trigger::Response::SharedPtr res);

  private:
    Eigen::Affine3d T_I2L;
    std::array<Eigen::Affine3d, 3> T_L2Cs;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubColorMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
#ifdef USE_LIVOX
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
#endif

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    std::queue<MessageGroup> syncQueue;
    std::mutex syncQueueMutex;
    std::condition_variable syncQueueCv;

    bool effect_pub_en = false, map_pub_en = false;
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0,
                           aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    FILE *fp;
    std::ofstream fout_pre, fout_out, fout_dbg;
};

int saveEverything();