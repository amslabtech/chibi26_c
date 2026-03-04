#include "local_map_creator/local_map_creator.hpp"

// ===== メイン関数 =====
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalMapCreator>();
    rclcpp::Rate loop_rate(node->getFreq());
    while(rclcpp::ok())
    {
        node->process();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
