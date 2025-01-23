#include <colormap.hpp>
#include <fmt/ranges.h>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

using namespace std;

auto logger = spdlog::basic_logger_mt("colormap_node", "colormap_node.log", true);

double time_ms(ImageMsg::SharedPtr &msg)
{
    return rclcpp::Time(msg->header.stamp).seconds() * 1000;
}

ColormapNode::Ptr ColormapNode::getInstance()
{
    static ColormapNode::Ptr instance(new ColormapNode());
    return instance;
}

void ColormapNode::queuePointCloud(PointCloudXYZRGBN::Ptr &pcd)
{
    std::lock_guard<std::mutex> lock(mtx);
    double pcd_time_ms = pcd->header.stamp;
    if(!params.publish_color_en || !image_msg_queue.empty() && pcd_time_ms >= time_ms(image_msg_queue.front()))
    {
        pointcloud_queue.push_back(pcd);
        cv.notify_all();
    }
    else
    {
        logger->warn("Dropping early pointcloud");
    }
}

void ColormapNode::initParameters()
{
    this->declare_parameter<bool>("publish.color_en", false);
    this->declare_parameter<string>("camera.topic", "/camera");
    this->declare_parameter<string>("camera.pcd_topic", "/colored_cloud");
    this->declare_parameter<double>("camera.z_filter", 0.0);
    this->declare_parameter<double>("camera.time_offset", 0.0);
    this->declare_parameter<double>("camera.angle_per_pixel", 0.0);
    auto declare_intrinsics_extrinsics = [&](string frame_id) {
        this->declare_parameter<string>(frame_id + ".frame_id", frame_id);
        this->declare_parameter<vector<double>>(frame_id + ".intrinsics", vector<double>());
        this->declare_parameter<vector<double>>(frame_id + ".extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>(frame_id + ".extrinsic_R", vector<double>());
        this->declare_parameter<vector<double>>(frame_id + ".fov", vector<double>());
    };
    declare_intrinsics_extrinsics("camera.front");
    declare_intrinsics_extrinsics("camera.left");
    declare_intrinsics_extrinsics("camera.right");

    bool success = true;
    success &= this->get_parameter_or("publish.color_en", params.publish_color_en, false);
    success &= this->get_parameter("camera.topic", params.camera_topic);
    success &= this->get_parameter("camera.pcd_topic", params.pcd_topic);
    success &= this->get_parameter("camera.z_filter", params.z_filter);
    success &= this->get_parameter("camera.time_offset", params.time_offset);
    success &= this->get_parameter("camera.angle_per_pixel", params.angle_per_pixel);
    auto get_intrinsics_extrinsics = [&](string frame_id) {
        vector<double> T, R, fov, intrinsics;
        string key;
        success &= this->get_parameter(frame_id + ".frame_id", key);
        success &= this->get_parameter(frame_id + ".intrinsics", intrinsics);
        success &= this->get_parameter(frame_id + ".extrinsic_T", T);
        success &= this->get_parameter(frame_id + ".extrinsic_R", R);
        success &= this->get_parameter(frame_id + ".fov", fov);
        success &= T.size() == 3 && R.size() == 9 && fov.size() == 2 && intrinsics.size() == 9;
        success &= (fov[0] < fov[1]);
        if (success)
        {
            Eigen::Matrix3d rot;
            rot << R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8];
            params.extrinsics_T_CI[key] = Eigen::Vector3d(T[0], T[1], T[2]);
            params.extrinsics_R_CI[key] = Eigen::Quaterniond(rot).normalized();
            params.fov[key] = Eigen::Vector2d(fov[0], fov[1]);
            params.intrinsics[key] = intrinsics;
        }
    };
    get_intrinsics_extrinsics("camera.front");
    get_intrinsics_extrinsics("camera.left");
    get_intrinsics_extrinsics("camera.right");

    if (!success)
    {
        throw std::runtime_error("[ColormapNode] Failed to get (or wrong) parameters");
    }
}

void ColormapNode::printParameters()
{
    logger->info("Camera topic: {}", params.camera_topic);
    logger->info("PCD topic: {}", params.pcd_topic);
    logger->info("Time offset: {}", params.time_offset);
    for (const auto& [frame_id, intrinsics] : params.intrinsics) {
        logger->info("Frame ID: {}", frame_id);
        logger->info("Intrinsics: [{}]", fmt::join(intrinsics, ", "));
    }
    for (auto &[frame_id, extrinsics] : params.extrinsics_T_CI)
    {
        logger->info("Frame ID: {}", frame_id);
        logger->info("Extrinsic T: [{}]", fmt::join(extrinsics, ", "));
        // logger->info("Extrinsic R: [{}]", fmt::join(params.extrinsics_R_CI[frame_id], ", "));
    }
}

void ColormapNode::cameraCallback(ImageMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx);
    auto msg_time = rclcpp::Time(msg->header.stamp).seconds();
    auto last_time = image_msg_queue.empty() ? -1e10 : rclcpp::Time(image_msg_queue.back()->header.stamp).seconds();
    image_msg_queue.push_back(msg);
    if (msg_time < last_time)
    {
        logger->warn("out of order");
        std::sort(image_msg_queue.begin(), image_msg_queue.end(),
                  [](auto &a, auto &b) { return rclcpp::Time(a->header.stamp) < rclcpp::Time(b->header.stamp); });
    }
    cv.notify_all();
}

ColormapNode::FrameGroup ColormapNode::sync()
{
    ColormapNode::FrameGroup g;

    g.pcd = pointcloud_queue.front();
    pointcloud_queue.pop_front();
    double pcd_times_ms = g.pcd->header.stamp;
    double threshold = 1.0; // allow 1ms difference
    while (!image_msg_queue.empty())
    {
        auto img_time_ms = time_ms(image_msg_queue.front());
        if (img_time_ms < pcd_times_ms - threshold)
        {
            logger->warn("Dropping stale image");
            image_msg_queue.pop_front();
        }
        else if (img_time_ms < pcd_times_ms + threshold)
        {
            g.imgs.push_back(image_msg_queue.front());
            image_msg_queue.pop_front();
        }
        else
        {
            break;
        }
    }
    if (g.imgs.size() != params.extrinsics_T_CI.size())
    {
        logger->warn("Incomplete frame set {}/{}", g.imgs.size(), params.extrinsics_T_CI.size());
    }

    return g;
}

ColormapNode::FrameGroup ColormapNode::sync_nocam()
{
    ColormapNode::FrameGroup g;

    g.pcd = pointcloud_queue.front();
    pointcloud_queue.pop_front();

    return g;
}

void ColormapNode::mapPinHole(PointCloudXYZRGBN &pcd, ImageMsg &img, PointCloudXYZRGBN &pcd_color)
{
    auto frame_id = img.header.frame_id;
    if (params.extrinsics_T_CI.find(frame_id) == params.extrinsics_T_CI.end())
    {
        logger->warn("frame {} doesn't exist", frame_id);
        return;
    }
    if (params.intrinsics.find(frame_id) == params.intrinsics.end()) 
    {
        logger->warn("Frame {} doesn't exist or intrinsics missing", frame_id);
        return;
    }
    logger->info("Map frame {}", frame_id);

    auto T = params.extrinsics_T_CI[frame_id];
    auto R = params.extrinsics_R_CI[frame_id];
    auto fov = params.fov[frame_id];
    auto intrinsics = params.intrinsics[frame_id];

    auto fx = intrinsics[0];
    auto fy = intrinsics[4];
    auto cx = intrinsics[2];
    auto cy = intrinsics[5];

    // check if img is a sensor_msgs::msg::CompressedImage or sensor_msgs::msg::Image
#ifndef ISAAC_SIM
    cv::Mat img_cv = cv::Mat(img.height, img.width, CV_8UC3, const_cast<uint8_t *>(img.data.data()));
    if (img_cv.empty())
    {
        logger->warn("Failed to decode image");
        return;
    }
#else
    cv::Mat img_cv = cv::imdecode(cv::Mat(img.data), cv::IMREAD_UNCHANGED);
    if (img_cv.empty())
    {
        logger->warn("Failed to decode image");
        return;
    }
#endif

    logger->info("Decoded");
    int mapped = 0;
    pcd_color.clear();
    for (auto &pt : pcd.points)
    {
        Eigen::Vector3d pt_imu(pt.x, pt.y, pt.z);
        Eigen::Vector3d pt_cam = R.conjugate() * (pt_imu - T);
        bool front = pt_cam.z() > 0; // ros cam +z forward
#ifndef ISAAC_SIM
        double r = sqrt(pt_cam.x() * pt_cam.x() + pt_cam.y() * pt_cam.y() + pt_cam.z() * pt_cam.z());
        double theta = atan2(pt_cam.y(), pt_cam.x());
        double elevation = acos(pt_cam.z() / r) * 57.295779513; // rad to deg
        double u = elevation * cos(theta) * pixel_per_angle + cx;
        double v = elevation * sin(theta) * pixel_per_angle + cy;
#else
        double u = fx * pt_cam.x() / pt_cam.z() + cx;
        double v = fy * pt_cam.y() / pt_cam.z() + cy;
#endif
        bool in = u >= 0 && u < img_cv.cols && v >= 0 && v < img_cv.rows;
        double azimuth = -360 * pt.curvature / 100 + 360;    // 0-100ms => 360-0deg
        azimuth = (azimuth < 180) ? azimuth : azimuth - 360; // [0:360] => [-180:180]
        bool fov_in = azimuth >= fov[0] && azimuth <= fov[1];
        if (front && in && fov_in)
        {
            cv::Vec3b color = img_cv.at<cv::Vec3b>(v, u);
            pt.r = color[0];
            pt.g = color[1];
            pt.b = color[2];
            pcd_color.push_back(pt);
            mapped++;
        }
    }
    logger->info("Mapped {} points", mapped); // mapping time is minimal compared to decoding time
}

void ColormapNode::putColor(PointCloudXYZRGBN &pcd, PointCloudXYZRGBN &pcd_color)
{
    int mapped = 0;
    pcd_color.clear();
    cv::Vec3b color(255,255,255);
    for (auto &pt : pcd.points)
    {
        pt.r = color[0];
        pt.g = color[1];
        pt.b = color[2];
        pcd_color.push_back(pt);
        mapped++;
    }
    logger->info("Mapped {} points", mapped); // mapping time is minimal compared to decoding time
}

void filterPointCloud(PointCloudXYZRGBN::Ptr cloud, float z_limit)
{
    pcl::PassThrough<PointCloudXYZRGBN::PointType> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-std::numeric_limits<float>::max(), z_limit); // Keep points with z <= z_limit
    pass.filter(*cloud);
}

void ColormapNode::colorizePointCloud(ColormapNode::FrameGroup &g)
{
    if (g.imgs.empty())
    {
        return;
    }

    PointCloudXYZRGBN::Ptr pcd_color(new PointCloudXYZRGBN);
    PointCloudXYZRGBN sub_pcd;
    for (auto &img : g.imgs)
    {
        mapPinHole(*(g.pcd), *img, sub_pcd);
        *pcd_color += sub_pcd;
    }

    auto pos = g.pcd->sensor_origin_.head<3>();
    auto orient = g.pcd->sensor_orientation_;
    pcl::transformPointCloud(*pcd_color, *pcd_color, pos, orient);
    if (params.z_filter > 0)
    {
        filterPointCloud(pcd_color, params.z_filter);
    }

    global_pcd += *pcd_color;

    PointCloud2Msg pcd_msg;
    pcl::toROSMsg(*pcd_color, pcd_msg);
    pcd_msg.header.stamp = rclcpp::Time(g.pcd->header.stamp * 1e6); // ms to ns
    pcd_msg.header.frame_id = "camera_init";
    color_publisher->publish(pcd_msg);
}

void ColormapNode::colorizePointCloud_nocam(ColormapNode::FrameGroup &g)
{
    PointCloudXYZRGBN::Ptr pcd_color(new PointCloudXYZRGBN);
    PointCloudXYZRGBN sub_pcd;
    putColor(*(g.pcd), sub_pcd);
    *pcd_color += sub_pcd;

    auto pos = g.pcd->sensor_origin_.head<3>();
    auto orient = g.pcd->sensor_orientation_;
    pcl::transformPointCloud(*pcd_color, *pcd_color, pos, orient);
    if (params.z_filter > 0)
    {
        filterPointCloud(pcd_color, params.z_filter);
    }

    global_pcd += *pcd_color;

    PointCloud2Msg pcd_msg;
    pcl::toROSMsg(*pcd_color, pcd_msg);
    pcd_msg.header.stamp = rclcpp::Time(g.pcd->header.stamp * 1e6); // ms to ns
    pcd_msg.header.frame_id = "camera_init";
    color_publisher->publish(pcd_msg);
}

void ColormapNode::worker()
{
    while (running)
    {
        ColormapNode::FrameGroup g;
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock, [&] {
                auto buffer_ready = !pointcloud_queue.empty() && image_msg_queue.size() > 20;
                return !running || buffer_ready;
            });

            g = sync();
        }

        colorizePointCloud(g);
    }
}

void ColormapNode::worker_nocam()
{
    while (running)
    {
        ColormapNode::FrameGroup g;
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock, [&] {
                auto buffer_ready = !pointcloud_queue.empty();
                return !running || buffer_ready;
            });

            g = sync_nocam();
        }

        colorizePointCloud_nocam(g);
    }
}

void ColormapNode::mapSaveCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary("color.pcd", global_pcd);
}

ColormapNode::ColormapNode(const rclcpp::NodeOptions &options) : Node("colormap_node")
{
    logger->flush_on(spdlog::level::info);
    initParameters();
    printParameters();


    auto qos = rclcpp::SensorDataQoS().reliable();

    color_publisher = this->create_publisher<PointCloud2Msg>(params.pcd_topic, qos);
    map_save_service = this->create_service<std_srvs::srv::Trigger>(
        "colormap_save", std::bind(&ColormapNode::mapSaveCallback, this, std::placeholders::_1, std::placeholders::_2));

    if (params.publish_color_en)
    {
        pixel_per_angle = 1 / params.angle_per_pixel;
        image_subscriber = this->create_subscription<ImageMsg>(
            params.camera_topic, qos, std::bind(&ColormapNode::cameraCallback, this, std::placeholders::_1));
    }

    running = true;
    if(params.publish_color_en)
    {
        colorize_thread = new std::thread(&ColormapNode::worker, this);
    }
    else
    {
        colorize_thread = new std::thread(&ColormapNode::worker_nocam, this);
    }
    
    initialized = true;
}

ColormapNode::~ColormapNode()
{
    if (colorize_thread)
    {
        running = false;
        cv.notify_all();
        colorize_thread->join();
        delete colorize_thread;
    }
}