#include "localizer/odom_model.hpp"

// デフォルトコンストラクタ
OdomModel::OdomModel() {}

// コンストラクタ
OdomModel::OdomModel(const double ff, const double fr, const double rf, const double rr)
    : engine_(seed_gen_()), std_norm_dist_(0.0, 1.0), fw_dev_(0.0), rot_dev_(0.0)
{
    // 各移動パラメータに対する分散（標準偏差の2乗）をあらかじめ計算
    fw_var_per_fw_  = ff * ff;   // 直進1mあたりの直進方向の分散
    fw_var_per_rot_ = fr * fr;   // 回転1radあたりの直進方向の分散
    rot_var_per_fw_ = rf * rf;   // 直進1mあたりの回転方向の分散
    rot_var_per_rot_ = rr * rr;  // 回転1radあたりの回転方向の分散
}

// 代入演算子
OdomModel& OdomModel::operator =(const OdomModel& model)
{
    if (this != &model) {
        this->fw_var_per_fw_   = model.fw_var_per_fw_;
        this->fw_var_per_rot_  = model.fw_var_per_rot_;
        this->rot_var_per_fw_  = model.rot_var_per_fw_;
        this->rot_var_per_rot_ = model.rot_var_per_rot_;
        this->fw_dev_          = model.fw_dev_;
        this->rot_dev_         = model.rot_dev_;
        // 乱数エンジンは各インスタンスで独立して保持
    }
    return *this;
}

// 並進，回転に関する標準偏差の設定
void OdomModel::set_dev(const double length, const double angle)
{
    // 移動距離(length)と回転角(angle)の絶対値に基づき、現在の移動に伴うノイズの標準偏差を算出
    // 分散 = (d1*d1*|dist| + d2*d2*|deg|) の平方根をとる
    fw_dev_  = std::sqrt(fw_var_per_fw_ * std::abs(length) + fw_var_per_rot_ * std::abs(angle));
    rot_dev_ = std::sqrt(rot_var_per_fw_ * std::abs(length) + rot_var_per_rot_ * std::abs(angle));
}

// 直進に関するノイズ（fw_dev_）の取得
double OdomModel::get_fw_noise()
{
    // 平均0、標準偏差fw_dev_の正規分布から乱数を生成
    return fw_dev_ * std_norm_dist_(engine_);
}

// 回転に関するノイズ（rot_dev_）の取得
double OdomModel::get_rot_noise()
{
    // 平均0、標準偏差rot_dev_の正規分布から乱数を生成
    return rot_dev_ * std_norm_dist_(engine_);
}