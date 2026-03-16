#include "localizer/pose.hpp"

// デフォルトコンストラクタ
Pose::Pose()
{

}

// コンストラクタ
Pose::Pose(const double x, const double y, const double yaw)
{

}

// 代入演算子
Pose& Pose::operator =(const Pose& pose)
{

}

// 複合代入演算子/=
Pose& Pose::operator /=(const double a)
{

}

// setter
void Pose::set(const double x, const double y, const double yaw)
{

}

// パーティクルの移動
// ノイズを加えて，移動させる
void Pose::move(double length, double direction, double rotation, const double fw_noise, const double rot_noise)
{
    
}

// 適切な角度(-M_PI ~ M_PI)に変更
void Pose::normalize_angle()
{

}