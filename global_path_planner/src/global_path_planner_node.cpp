#include "global_path_planner/global_path_planner.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<Astar> astar = std::make_shared<Astar>();
    rclcpp::spin(astar);
    rclcpp::shutdown();

    return 0;
}