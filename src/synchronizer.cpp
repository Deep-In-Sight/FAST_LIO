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

#define SYNC_DEBUG 1

template <typename T> void print_timestamp(T msg, const std::string &name)
{
    auto timestamp = rclcpp::Time(msg->header.stamp).seconds();
    std::cout << name << " : " << timestamp << std::endl;
}

void SigHandle(int sig)
{
    std::cout << "catch sig %d" << sig << std::endl;
    rclcpp::shutdown();
}

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
    SynchronizeNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("synchronizer", options)
    {
        // Declare parameters
        this->declare_parameter<int>("common.camera_num", 3);
        this->get_parameter_or<int>("common.camera_num", camera_num, 3);
        this->declare_parameter<double>("common.lidar_scan_time", 0.1);
        this->get_parameter_or<double>("common.lidar_scan_time", lidar_scan_time, 0.1);
        this->declare_parameter<double>("common.camera_lidar_time_offset", 0.0);
        this->get_parameter_or<double>("common.camera_lidar_time_offset", camera_lidar_time_offset, 0.0);

        // create subscribers
        lidar_subscriber = this->create_subscription<PointCloudMsg>(
            "/lidar", rclcpp::SensorDataQoS().keep_all(),
            [this](const PointCloudMsg::SharedPtr msg) { lidar_callback(msg); });
        imu_subscriber = this->create_subscription<ImuMsg>("/imu/data", rclcpp::SensorDataQoS().keep_all(),
                                                           [this](const ImuMsg::SharedPtr msg) { imu_callback(msg); });
        for (int i = 0; i < camera_num; i++)
        {
            auto topic_name = "/camera_" + std::to_string(i) + "_CompressedImage";
            auto camera_sub = this->create_subscription<JpegImageMsg>(
                topic_name, rclcpp::SensorDataQoS().keep_all(),
                [this](const JpegImageMsg::SharedPtr msg) { camera_callback(msg); });
            camera_subscribers.push_back(camera_sub);
        }

        syncThread = new std::thread(&SynchronizeNode::syncLoop, this);
    }

    ~SynchronizeNode()
    {
#if SYNC_DEBUG
        std::cout << "lidar_buffer size: " << lidar_buffer.size() << std::endl;
        std::cout << "imu_buffer size: " << imu_buffer.size() << std::endl;
        std::cout << "camera_buffer size: " << camera_buffer.size() << std::endl;
#endif
        syncThreadRunning = false;
        syncCv.notify_all();
        syncThread->join();
    }

    void lidar_callback(const PointCloudMsg::SharedPtr msg)
    {
        std::unique_lock<std::mutex> lock(sync_mutex);
        lidar_buffer.push_back(msg);
        syncCv.notify_all();
    }

    void imu_callback(const ImuMsg::SharedPtr msg)
    {
        imu_buffer.push_back(msg);
    }

    void camera_callback(const JpegImageMsg::SharedPtr msg)
    {
        // adjust camera timestamp
        auto timestamp = rclcpp::Time(msg->header.stamp).seconds();
        auto timestamp_ns = (int64_t)((timestamp + camera_lidar_time_offset) * 1e9);
        msg->header.stamp = rclcpp::Time(timestamp_ns);
        std::unique_lock<std::mutex> lock(sync_mutex);
        camera_buffer.push_back(msg);
        std::sort(camera_buffer.begin(), camera_buffer.end(),
                  [](const JpegImageMsg::SharedPtr &a, const JpegImageMsg::SharedPtr &b) {
                      return rclcpp::Time(a->header.stamp).seconds() < rclcpp::Time(b->header.stamp).seconds();
                  });
        syncCv.notify_all();
    }

    void setCallback(std::function<void(MessageGroup &)> callback)
    {
        syncedCallback = callback;
    }

    void syncLoop()
    {
        while (syncThreadRunning)
        {
            std::unique_lock<std::mutex> lock(sync_mutex);
            // wait until all buffer has data
            syncCv.wait(lock, [this] {
                bool buffer_ready = !lidar_buffer.empty() && !imu_buffer.empty() && camera_buffer.size() > 20;
                return !syncThreadRunning || buffer_ready;
            });

            auto lidar_end_time = rclcpp::Time(lidar_buffer.front()->header.stamp).seconds();
            auto lidar_begin_time = lidar_end_time - 1.0 / 10.0; // get the scan time from parameter

            // lidar and imu
            auto imu_first_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();
            auto imu_last_time = rclcpp::Time(imu_buffer.back()->header.stamp).seconds();

            // drop early lidar scan until we get IMU readings
            if (lidar_end_time < imu_first_time)
            {
                lidar_buffer.pop_front();
                continue;
            }
            // wait until have enough IMU readings
            if (imu_last_time < lidar_end_time)
            {
                continue;
            }

            // lidar and camera
            double threshold = 0.1 * lidar_scan_time; // allow 10% early, add extra flexibility with the offset
            auto camera_time = rclcpp::Time(camera_buffer.front()->header.stamp).seconds();
            if (std::abs(lidar_end_time - camera_time) > threshold)
            {
                if (camera_time < lidar_end_time)
                {
                    camera_buffer.pop_front(); // drop early camera frame
                }
                else
                {
                    lidar_buffer.pop_front(); // drop early lidar frame
                }
                continue;
            }

            // at this point things should be sync'ed
            MessageGroup msg_group;
            msg_group.lidar = lidar_buffer.front();
            lidar_buffer.pop_front();
            while (!imu_buffer.empty())
            {
                msg_group.imu.push_back(imu_buffer.front());
                imu_buffer.pop_front();
                if (rclcpp::Time(imu_buffer.front()->header.stamp).seconds() > lidar_end_time)
                    break;
            }
            while (!camera_buffer.empty())
            {
                msg_group.cameras.push_back(camera_buffer.front());
                camera_buffer.pop_front();
                if (std::abs(rclcpp::Time(camera_buffer.front()->header.stamp).seconds() - lidar_end_time) > threshold)
                    break;
            }

            if (syncedCallback != nullptr)
                syncedCallback(msg_group);
        }
    }

  private:
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

/**
 * real use:
 * - clone the group and put into a queue
 * - notify waiters
 * - return quickly and let other thread process the group.
 * */
void synchCallback(MessageGroup &group)
{
    std::cout << "synced ========== ";
    print_timestamp(group.lidar, "lidar");
    std::cout << "imu size: " << group.imu.size() << std::endl;
    print_timestamp(group.imu[0], "imu begin");
    print_timestamp(group.imu[group.imu.size() - 1], "imu end");

    for (int i = 0; i < group.cameras.size(); i++)
    {
        std::cout << "camera_" << i << " ";
        print_timestamp(group.cameras[i], "");
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    signal(SIGINT, SigHandle);
    auto synchronizer = std::make_shared<SynchronizeNode>();
    synchronizer->setCallback(synchCallback);
    rclcpp::spin(synchronizer);
    if (rclcpp::ok())
        rclcpp::shutdown();

    return 0;
}