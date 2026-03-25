// /home/amsl/ros2_ws/src/chibi26_c/localizer/src/pose.cpp
#include "localizer/pose.hpp"

// デフォルトコンストラクタ
Pose::Pose() : x_(0.0), y_(0.0), yaw_(0.0)
{
}

// コンストラクタ
Pose::Pose(const double x, const double y, const double yaw) : x_(x), y_(y), yaw_(yaw)
{
}

// 代入演算子
Pose& Pose::operator =(const Pose& pose)
{
    if (this != &pose) {
        this->x_ = pose.x_;
        this->y_ = pose.y_;
        this->yaw_ = pose.yaw_;
    }
    return *this;
}

// 複合代入演算子/= (平均計算などで使用)
Pose& Pose::operator /=(const double a)
{
    if (a != 0.0) {
        this->x_ /= a;
        this->y_ /= a;
        this->yaw_ /= a;
    }
    return *this;
}

// setter
void Pose::set(const double x, const double y, const double yaw)
{
    x_ = x;
    y_ = y;
    yaw_ = yaw;
}

// パーティクルの移動
// ノイズを加えて，移動させる
void Pose::move(double length, double direction, double rotation, const double fw_noise, const double rot_noise)
{
    // 移動距離と回転角にノイズを加える
    double dist = length + fw_noise;
    double rot  = rotation + rot_noise;

    // 現在の向き(yaw_)と移動の向き(direction)から、新しい座標を計算
    // directionは機体正面に対する移動方向（通常、直進なら0）
    x_ += dist * std::cos(yaw_ + direction);
    y_ += dist * std::sin(yaw_ + direction);

    // 向きを更新
    yaw_ += rot;

    // 角度を -M_PI ~ M_PI の範囲に正規化
    normalize_angle();
}

// 適切な角度(-M_PI ~ M_PI)に変更
void Pose::normalize_angle()
{
    while (yaw_ > M_PI)  yaw_ -= 2.0 * M_PI;
    while (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;
}