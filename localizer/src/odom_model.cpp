#include "localizer/odom_model.hpp"

// デフォルトコンストラクタ
OdomModel::OdomModel() {}

// コンストラクタ
OdomModel::OdomModel(const double ff, const double fr, const double rf, const double rr)
    : engine_(seed_gen_()), std_norm_dist_(0.0, 1.0), fw_dev_(0.0), rot_dev_(0.0)
{
    fw_var_per_fw_ /* = */;
    fw_var_per_rot_ /* = */;
    rot_var_per_fw_ /* = */;
    rot_var_per_rot_ /* = */;
}

// 代入演算子
OdomModel& OdomModel::operator =(const OdomModel& model)
{


    return *this;
}

// 並進，回転に関する標準偏差の設定
void OdomModel::set_dev(const double length, const double angle)
{
    
}

// 直進に関するノイズ（fw_dev_）の取得
double OdomModel::get_fw_noise()
{

}

// 回転に関するノイズ（rot_dev_）の取得
double OdomModel::get_rot_noise()
{

}
