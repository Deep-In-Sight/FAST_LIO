#include <laserMapping.hpp>
#include <synchronizer.hpp>

void SigHandle(int sig)
{
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, SigHandle);

    auto mappingNode = std::make_shared<LaserMappingNode>();
    auto syncNode = std::make_shared<SynchronizeNode>();
    syncNode->setCallback([mappingNode](MessageGroup &group) { mappingNode->enqueueGroup(group); });

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(mappingNode);
    executor.add_node(syncNode);

    executor.spin();

    if (rclcpp::ok())
        rclcpp::shutdown();

    saveEverything();

    return 0;
}