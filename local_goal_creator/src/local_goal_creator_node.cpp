#include "local_goal_creator/local_goal_creator.hpp"

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv); // ノードの初期化
    auto node = std::make_shared<LocalGoalCreator>();
    rclcpp::Rate loop_rate(node->getOdomFreq());
    

    while(rclcpp::ok()){
        node->process();
        rclcpp::spin_some(node);   // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }

    return 0;
}