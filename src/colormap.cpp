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
    this->declare_parameter<bool>("publish.color_compressed", false);
    this->declare_parameter<string>("camera.topic", "/camera");
    this->declare_parameter<string>("camera.pcd_topic", "/colored_cloud");
    this->declare_parameter<double>("camera.z_filter", 0.0);
    this->declare_parameter<double>("camera.time_offset", 0.0);
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
    success &= this->get_parameter_or("publish.color_en", params.publish_color_en, false);
    success &= this->get_parameter_or("publish.color_compressed", params.color_compressed, false);
    success &= this->get_parameter("camera.topic", params.camera_topic);
    success &= this->get_parameter("camera.pcd_topic", params.pcd_topic);
    success &= this->get_parameter("camera.z_filter", params.z_filter);
    success &= this->get_parameter("camera.time_offset", params.time_offset);
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
    double threshold = 17.0; //17
    double diff = 0.0;
    while (!image_msg_queue.empty())
    {
        auto img_time_ms = time_ms(image_msg_queue.front());
        diff = pcd_times_ms - img_time_ms;
        if (abs(diff) < threshold)
        {
            g.imgs.push_back(image_msg_queue.front());
            image_msg_queue.pop_front();
        }
        else if (diff < 0)
        {
            break;
        }
        else 
        {
            logger->warn("Dropping stale image");
            image_msg_queue.pop_front();
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
    // print img timestamp

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
    auto distortion = params.distortion[frame_id];

    auto fx = intrinsics[0];
    auto fy = intrinsics[4];
    auto cx = intrinsics[2];
    auto cy = intrinsics[5];
    auto k1 = distortion[0];
    auto k2 = distortion[1];
    auto k3 = distortion[2];
    auto k4 = distortion[3];

    // check if img is a sensor_msgs::msg::CompressedImage or sensor_msgs::msg::Image
    cv::Mat img_cv;
    // if(params.color_compressed)
    // {
    //     img_cv = cv::Mat(img.height, img.width, CV_8UC3, const_cast<uint8_t *>(img.data.data()));
    //     if (img_cv.empty())
    //     {
    //         logger->warn("Failed to decode image");
    //         return;
    //     }
    // }
    // else
    {
        img_cv = cv::imdecode(cv::Mat(img.data), cv::IMREAD_UNCHANGED);
        if (img_cv.empty())
        {
            logger->warn("Failed to decode image");
            return;
        }
    }

    logger->info("Decoded");
    int mapped = 0;
    pcd_color.clear();
    
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
        float x = static_cast<float>(fx * x_d * y_d + cx);
        float y = static_cast<float>(fy * y_d + cy);

        int ix = static_cast<int>(std::round(x));
        int iy = static_cast<int>(std::round(y));

        cv::Vec3b color;

        bool in = 0 <= ix < img_cv.cols && 0 <= iy < img_cv.rows;

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
        std::cout << "No image available" << std::endl;
        return;
    }

    PointCloudXYZRGBN::Ptr pcd_color(new PointCloudXYZRGBN);
    PointCloudXYZRGBN sub_pcd;

    auto pos = g.pcd->sensor_origin_.head<3>();
    auto orient = g.pcd->sensor_orientation_;
    for (auto &img : g.imgs)
    {
        mapPinHole(*(g.pcd), *img, sub_pcd);
        *pcd_color += sub_pcd;

        std::string frame_id = img->header.frame_id;
        std::string cam_timestamp_save = std::to_string(img->header.stamp.sec) + std::to_string(img->header.stamp.nanosec) + "\n";
        auto T = params.extrinsics_T_CI[frame_id];
        auto R = params.extrinsics_R_CI[frame_id];  
        Eigen::Vector3d cam_pos = R * pos.cast<double>() + T.cast<double>();
        Eigen::Quaterniond rot_quat(R);
        Eigen::Quaterniond cam_orient = rot_quat * orient.cast<double>();
        std::string cam_pos_save = std::to_string(cam_pos.x()) + " " + std::to_string(cam_pos.y()) + " " + std::to_string(cam_pos.z()) + "\n";
        std::string cam_orient_save = std::to_string(cam_orient.x()) + " " + std::to_string(cam_orient.y()) + " " + std::to_string(cam_orient.z()) + " " + std::to_string(cam_orient.w()) + "\n";
    
        cam_path_output += frame_id + "\n" + cam_timestamp_save + cam_pos_save + cam_orient_save + "\n";
    }

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
    if (pcl::io::savePLYFile("color.ply", global_pcd, /*binary=*/true) < 0)
    {
        std::cout << "Error: PLY 파일 저장 실패" << std::endl;
        response->success = false;
        response->message = "PLY File Save Fail";
        return;
    } else {
        std::cout << "PLY 파일 저장 성공" << std::endl;
    }

    std::ofstream outfile("cam_pos_orient.txt");
    if(outfile.is_open())
    {
        outfile << cam_path_output;
        outfile.close();
    }
    else
    {
        std::cout << "Error: cam_pos_orient.txt 파일 저장 실패" << std::endl;
        response->success = false;
        response->message = "Cam Pos Orient File Save Fail";
        return;
    }

    response->success = true;
    response->message = "PCL File Save Success";
    return;
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