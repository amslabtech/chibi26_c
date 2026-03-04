#include "obstacle_detector/obstacle_detector.hpp"
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<ObstacleDetector> od = std::make_shared<ObstacleDetector>();
    rclcpp::spin(od);
    rclcpp::shutdown();
    return 0;
}