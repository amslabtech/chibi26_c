#ifndef ODOM_MODEL_HPP
#define ODOM_MODEL_HPP

#include <random>

class OdomModel
{
    public:
        OdomModel();    // デフォルトコンストラクタ
        OdomModel(const double ff, const double fr, const double rf, const double rr);  // コンストラクタ
        OdomModel& operator =(const OdomModel& model); // 代入演算子

        void   set_dev(const double length, const double angle); // 標準偏差の設定
        double get_fw_noise();  // 直進に関するノイズの取得
        double get_rot_noise(); // 回転に関するノイズの取得

    private:
        double fw_var_per_fw_;   // ffの分散
        double fw_var_per_rot_;  // frの分散
        double rot_var_per_fw_;  // rfの分散
        double rot_var_per_rot_; // rrの分散

        double fw_dev_;  // 直進に関するノイズ
        double rot_dev_; // 回転に関するノイズ

        // 正規分布
        std::normal_distribution<> std_norm_dist_;
        std::random_device seed_gen_;
        std::default_random_engine engine_;
};

#endif