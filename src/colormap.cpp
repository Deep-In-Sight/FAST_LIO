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

// Forward declarations
void filterPointCloud(PointCloudXYZRGBN::Ptr cloud, float z_limit);

template<typename ImageType>
double time_ms(typename ImageType::SharedPtr &msg)
{
    return rclcpp::Time(msg->header.stamp).seconds() * 1000;
}

ColormapNode::Ptr ColormapNode::getInstance()
{
    static ColormapNode::Ptr instance(new ColormapNode());
    return instance;
}

bool ColormapNode::isEnabled()
{
    this->declare_parameter<bool>("camera.enable", false);
    bool enable;
    this->get_parameter("camera.enable", enable);
    return enable;
}

void ColormapNode::queuePointCloud(PointCloudXYZRGBN::Ptr &pcd)
{
    std::lock_guard<std::mutex> lock(mtx);
    pointcloud_queue.push_back(pcd);
    cv.notify_all();
}

void ColormapNode::queueOdometry(nav_msgs::msg::Odometry &odom)
{
    std::lock_guard<std::mutex> lock(mtx);
    odom_queue.push_back(odom);
    cv.notify_all();
}

template<typename ImageType>
void ColormapNode::cameraCallback(typename ImageType::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx);
    auto msg_time = rclcpp::Time(msg->header.stamp).seconds();
    
    if constexpr (std::is_same_v<ImageType, sensor_msgs::msg::CompressedImage>) {
        auto last_time = compressed_image_queue.empty() ? -1e10 : 
            rclcpp::Time(compressed_image_queue.back()->header.stamp).seconds();
        compressed_image_queue.push_back(msg);
        if (msg_time < last_time) {
            logger->warn("out of order");
            std::sort(compressed_image_queue.begin(), compressed_image_queue.end(),
                [](auto &a, auto &b) { return rclcpp::Time(a->header.stamp) < rclcpp::Time(b->header.stamp); });
        }
    } else {
        auto last_time = image_queue.empty() ? -1e10 : 
            rclcpp::Time(image_queue.back()->header.stamp).seconds();
        image_queue.push_back(msg);
        if (msg_time < last_time) {
            logger->warn("out of order");
            std::sort(image_queue.begin(), image_queue.end(),
                [](auto &a, auto &b) { return rclcpp::Time(a->header.stamp) < rclcpp::Time(b->header.stamp); });
        }
    }
    cv.notify_all();
}

template<typename ImageType>
ColormapNode::FrameGroup<ImageType> ColormapNode::sync()
{
    ColormapNode::FrameGroup<ImageType> g;

    if (pointcloud_queue.empty() || odom_queue.empty()) {
        return g;
    }

    // Get the first pointcloud and odometry
    g.pcd = pointcloud_queue.front();
    auto odom = odom_queue.front();

    // Verify timestamps match
    double pcd_time = rclcpp::Time(g.pcd->header.stamp).seconds();
    double odom_time = rclcpp::Time(odom.header.stamp).seconds();
    if (std::abs(pcd_time - odom_time) > 0.001) { // 1ms tolerance
        logger->warn("Timestamp mismatch between pointcloud ({}) and odometry ({})", 
                    pcd_time, odom_time);
        return g;
    }

    // Remove processed messages
    pointcloud_queue.pop_front();
    odom_queue.pop_front();

    // Process images
    double threshold = 17.0; // 17ms
    if constexpr (std::is_same_v<ImageType, sensor_msgs::msg::CompressedImage>) {
        while (!compressed_image_queue.empty()) {
            auto img_time = rclcpp::Time(compressed_image_queue.front()->header.stamp).seconds();
            double diff = pcd_time - img_time;
            
            if (std::abs(diff) < threshold) {
                g.imgs.push_back(compressed_image_queue.front());
                compressed_image_queue.pop_front();
            } else if (diff < 0) {
                break;
            } else {
                logger->warn("Dropping stale image");
                compressed_image_queue.pop_front();
            }
        }
    } else {
        while (!image_queue.empty()) {
            auto img_time = rclcpp::Time(image_queue.front()->header.stamp).seconds();
            double diff = pcd_time - img_time;
            
            if (std::abs(diff) < threshold) {
                g.imgs.push_back(image_queue.front());
                image_queue.pop_front();
            } else if (diff < 0) {
                break;
            } else {
                logger->warn("Dropping stale image");
                image_queue.pop_front();
            }
        }
    }

    if (g.imgs.size() != params.extrinsics_T_CI.size()) {
        logger->warn("Incomplete frame set {}/{}", g.imgs.size(), params.extrinsics_T_CI.size());
    }

    return g;
}

template<typename ImageType>
void ColormapNode::colorizePointCloud(FrameGroup<ImageType> &g)
{
    if (g.imgs.empty()) {
        return;
    }

    PointCloudXYZRGBN::Ptr pcd_color(new PointCloudXYZRGBN);
    PointCloudXYZRGBN sub_pcd;

    Eigen::Vector3f pos = g.pcd->sensor_origin_.template head<3>();
    Eigen::Quaternionf orient = g.pcd->sensor_orientation_;
    for (auto &img : g.imgs) {
        mapPinHole(*(g.pcd), *img, sub_pcd);
        *pcd_color += sub_pcd;

        std::string frame_id = img->header.frame_id;
        auto T = params.extrinsics_T_CI[frame_id];
        auto R = params.extrinsics_R_CI[frame_id];  
        Eigen::Vector3d cam_pos = R * pos.cast<double>() + T;
        Eigen::Quaterniond rot_quat(R);
        Eigen::Quaterniond cam_orient = rot_quat * orient.cast<double>();
    }

    pcl::transformPointCloud(*pcd_color, *pcd_color, pos, orient);
    if (params.z_filter > 0) {
        filterPointCloud(pcd_color, params.z_filter);
    }

    global_pcd += *pcd_color;

    sensor_msgs::msg::PointCloud2 pcd_msg;
    pcl::toROSMsg(*pcd_color, pcd_msg);
    pcd_msg.header.stamp = rclcpp::Time(g.pcd->header.stamp); // ms to ns
    pcd_msg.header.frame_id = "camera_init";
    color_publisher->publish(pcd_msg);
}

template<typename ImageType>
void ColormapNode::worker()
{
    while (running)
    {
        FrameGroup<ImageType> g;
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock, [&] {
                auto buffer_ready = !pointcloud_queue.empty() && !odom_queue.empty() && 
                    ((std::is_same_v<ImageType, sensor_msgs::msg::CompressedImage> && compressed_image_queue.size() > 20) ||
                     (std::is_same_v<ImageType, sensor_msgs::msg::Image> && image_queue.size() > 20));
                return !running || buffer_ready;
            });
            g = sync<ImageType>();
        }

        if (!g.imgs.empty()) {
            colorizePointCloud(g);
        }
    }
}

template<typename ImageType>
void ColormapNode::mapPinHole(PointCloudXYZRGBN &pcd, ImageType &img, PointCloudXYZRGBN &pcd_color)
{
    auto frame_id = img.header.frame_id;
    logger->info("Map frame {}", frame_id);

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

    auto T = params.extrinsics_T_CI[frame_id];
    auto R = params.extrinsics_R_CI[frame_id];
    auto fov = params.fov[frame_id];
    auto intrinsics = params.intrinsics[frame_id];
    auto distortion = params.distortion[frame_id];

    auto fx = intrinsics[0];
    auto fy = intrinsics[4];
    auto cx = intrinsics[2];
    auto cy = intrinsics[5];
    auto k1 = distortion[0];
    auto k2 = distortion[1];
    auto k3 = distortion[2];
    auto k4 = distortion[3];

    // Convert image to OpenCV format
    cv::Mat img_cv;
    if constexpr (std::is_same_v<ImageType, sensor_msgs::msg::CompressedImage>) {
        img_cv = cv::imdecode(cv::Mat(img.data), cv::IMREAD_UNCHANGED);
    } else if constexpr (std::is_same_v<ImageType, sensor_msgs::msg::Image>) {
        if (img.encoding == "rgb8" || img.encoding == "bgr8") {
            img_cv = cv::Mat(img.height, img.width, CV_8UC3, const_cast<uint8_t*>(img.data.data()));
        } else {
            logger->warn("Unsupported image encoding: {}", img.encoding);
            return;
        }
    }

    if (img_cv.empty()) {
        logger->warn("Failed to convert image to OpenCV format");
        return;
    }

    int mapped = 0;
    pcd_color.clear();
    
    // Project points to image plane
    for (auto &pt : pcd.points)
    {
        Eigen::Vector3d pt_imu(pt.x, pt.y, pt.z);
        Eigen::Vector3d pt_cam = R * pt_imu + T;

        if(pt_cam.z() <= 0) // ros cam +z forward
            continue;

        float a = static_cast<float>(pt_cam.x() / pt_cam.z());
        float b = static_cast<float>(pt_cam.y() / pt_cam.z());
        float r_val = std::sqrt(a * a + b * b);
        float x_d = 0.0f, y_d = 0.0f;
        if(r_val < 1e-6) {
            x_d = 0.0f;
            y_d = 0.0f;
        } else {
            float theta = std::atan(r_val);
            float theta_d = theta * (1 + k1 * std::pow(theta,2) +
                                     k2 * std::pow(theta,4) +
                                     k3 * std::pow(theta,6) +
                                     k4 * std::pow(theta,8));
            x_d = (theta_d / r_val) * a;
            y_d = (theta_d / r_val) * b;
        }
        float x = static_cast<float>(fx * x_d + cx);
        float y = static_cast<float>(fy * y_d + cy);

        int ix = static_cast<int>(std::round(x));
        int iy = static_cast<int>(std::round(y));

        bool in = 0 <= ix && ix < img_cv.cols && 0 <= iy && iy < img_cv.rows;

        double azimuth = -360 * pt.curvature / 100 + 360;    // 0-100ms => 360-0deg
        bool fov_in = fov[0] <= azimuth && azimuth <= fov[1];

        if (in && fov_in)
        {
            cv::Vec3b color = img_cv.at<cv::Vec3b>(iy, ix);
            pt.r = color[2];
            pt.g = color[1];
            pt.b = color[0];
            pcd_color.push_back(pt);
            mapped++;
        }
    }
}

// Explicit template instantiations
template void ColormapNode::cameraCallback<sensor_msgs::msg::CompressedImage>(sensor_msgs::msg::CompressedImage::SharedPtr);
template void ColormapNode::cameraCallback<sensor_msgs::msg::Image>(sensor_msgs::msg::Image::SharedPtr);
template ColormapNode::FrameGroup<sensor_msgs::msg::CompressedImage> ColormapNode::sync<sensor_msgs::msg::CompressedImage>();
template ColormapNode::FrameGroup<sensor_msgs::msg::Image> ColormapNode::sync<sensor_msgs::msg::Image>();
template void ColormapNode::colorizePointCloud<sensor_msgs::msg::CompressedImage>(FrameGroup<sensor_msgs::msg::CompressedImage> &);
template void ColormapNode::colorizePointCloud<sensor_msgs::msg::Image>(FrameGroup<sensor_msgs::msg::Image> &);
template void ColormapNode::worker<sensor_msgs::msg::CompressedImage>();
template void ColormapNode::worker<sensor_msgs::msg::Image>();
template void ColormapNode::mapPinHole<sensor_msgs::msg::CompressedImage>(PointCloudXYZRGBN &, sensor_msgs::msg::CompressedImage &, PointCloudXYZRGBN &);
template void ColormapNode::mapPinHole<sensor_msgs::msg::Image>(PointCloudXYZRGBN &, sensor_msgs::msg::Image &, PointCloudXYZRGBN &);

void ColormapNode::initParameters()
{
    this->declare_parameter<bool>("camera.compressed_image", false);
    this->declare_parameter<string>("camera.topic", "/camera");
    this->declare_parameter<string>("camera.pcd_topic", "/colored_cloud");
    this->declare_parameter<double>("camera.z_filter", 0.0);
    this->declare_parameter<double>("camera.time_offset", 0.0);
    this->declare_parameter<int>("camera.frame_rate", 30);
    auto declare_intrinsics_extrinsics = [&](string frame_id) {
        this->declare_parameter<string>(frame_id + ".frame_id", frame_id);
        this->declare_parameter<vector<double>>(frame_id + ".intrinsics", vector<double>());
        this->declare_parameter<vector<double>>(frame_id + ".distortion", vector<double>());
        this->declare_parameter<vector<double>>(frame_id + ".extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>(frame_id + ".extrinsic_R", vector<double>());
        this->declare_parameter<vector<double>>(frame_id + ".fov", vector<double>());
    };
    declare_intrinsics_extrinsics("camera.front");
    declare_intrinsics_extrinsics("camera.left");
    declare_intrinsics_extrinsics("camera.right");

    bool success = true;
    success &= this->get_parameter_or("camera.compressed_image", params.compressed_image, false);
    success &= this->get_parameter("camera.topic", params.camera_topic);
    success &= this->get_parameter("camera.pcd_topic", params.pcd_topic);
    success &= this->get_parameter("camera.z_filter", params.z_filter);
    success &= this->get_parameter("camera.time_offset", params.time_offset);
    success &= this->get_parameter("camera.frame_rate", params.frame_rate);
    auto get_intrinsics_extrinsics = [&](string frame_id) {
        vector<double> T, R, fov, intrinsics, distortion;
        bool front;
        string key;
        success &= this->get_parameter(frame_id + ".frame_id", key);
        success &= this->get_parameter(frame_id + ".intrinsics", intrinsics);
        success &= this->get_parameter(frame_id + ".distortion", distortion);
        success &= this->get_parameter(frame_id + ".extrinsic_T", T);
        success &= this->get_parameter(frame_id + ".extrinsic_R", R);
        success &= this->get_parameter(frame_id + ".fov", fov);
        success &= T.size() == 3 && R.size() == 9 && fov.size() == 2 && intrinsics.size() == 9;
        if(fov[0] > fov[1]){
            // swap
            double temp = fov[0];
            fov[0] = fov[1];
            fov[1] = temp;
        }
        success &= (fov[0] <= fov[1]);
        if (success)
        {
            params.extrinsics_T_CI[key] << T[0], T[1], T[2];
            params.extrinsics_R_CI[key] << R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8];
            params.fov[key] = Eigen::Vector2d(fov[0], fov[1]);
            params.intrinsics[key] = intrinsics;
            params.distortion[key] = distortion;
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
    logger->info("Frame rate: {}", params.frame_rate);
    for (const auto& [frame_id, intrinsics] : params.intrinsics) {
        logger->info("Frame ID: {}", frame_id);
        logger->info("Intrinsics: [{}]", fmt::join(intrinsics, ", "));
    }
    for (const auto& [frame_id, distortion] : params.distortion) {
        logger->info("Frame ID: {}", frame_id);
        logger->info("Distortion: [{}]", fmt::join(distortion, ", "));
    }
    for (auto &[frame_id, extrinsics] : params.extrinsics_T_CI)
    {
        logger->info("Frame ID: {}", frame_id);
        logger->info("Extrinsic T: [{}]", fmt::join(extrinsics, ", "));
        // logger->info("Extrinsic R: [{}]", fmt::join(params.extrinsics_R_CI[frame_id], ", "));
    }
}

double ColormapNode::poly_eval(const Eigen::VectorXd &coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); ++i) {
        result = result * x + coeffs[i];
    }
    return result;
}

void filterPointCloud(PointCloudXYZRGBN::Ptr cloud, float z_limit)
{
    pcl::PassThrough<PointCloudXYZRGBN::PointType> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-std::numeric_limits<float>::max(), z_limit); // Keep points with z <= z_limit
    pass.filter(*cloud);
}

void ColormapNode::mapSaveCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (pcl::io::savePLYFile("color.ply", global_pcd, /*binary=*/true) < 0)
    {
        std::cout << "Error: PLY 파일 저장 실패" << std::endl;
        response->success = false;
        response->message = "PLY File Save Fail";
        return;
    } else {
        std::cout << "PLY 파일 저장 성공" << std::endl;
    }

    response->success = true;
    response->message = "PCL File Save Success";
    return;
}

ColormapNode::ColormapNode(const rclcpp::NodeOptions &options) : Node("colormap_node")
{
    logger->flush_on(spdlog::level::info);
    if(isEnabled())
    {
        initParameters();
        printParameters();
        
        auto qos = rclcpp::QoS(10).keep_all().reliable();

        color_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(params.pcd_topic, qos);
        map_save_service = this->create_service<std_srvs::srv::Trigger>(
            "colormap_save", std::bind(&ColormapNode::mapSaveCallback, this, std::placeholders::_1, std::placeholders::_2));

        if (params.compressed_image) {
            compressed_image_subscriber = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                params.camera_topic, qos, std::bind(&ColormapNode::cameraCallback<sensor_msgs::msg::CompressedImage>, this, std::placeholders::_1));
            running = true;
            colorize_thread = new std::thread(&ColormapNode::worker<sensor_msgs::msg::CompressedImage>, this);
        } else {
            image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
                params.camera_topic, qos, std::bind(&ColormapNode::cameraCallback<sensor_msgs::msg::Image>, this, std::placeholders::_1));
            running = true;
            colorize_thread = new std::thread(&ColormapNode::worker<sensor_msgs::msg::Image>, this);
        }
    }
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