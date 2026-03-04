#include "local_path_planner/local_path_planner.hpp"

//===== メイン関数 =====
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv); // ノードの初期化
    std::shared_ptr<DWAPlanner> dwa = std::make_shared<DWAPlanner>();
    rclcpp::Rate loop_rate(dwa->get_freq()); // 制御周波数の設定   

    while(rclcpp::ok())
    {
        dwa->process();
        rclcpp::spin_some(dwa);   // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
    rclcpp::shutdown();

    return 0;
}