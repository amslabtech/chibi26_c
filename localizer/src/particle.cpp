// /home/amsl/ros2_ws/src/chibi26_c/localizer/src/particle.cpp
#include "localizer/particle.hpp"

#include <algorithm>  // std::max のため
#include <cmath>

// デフォルトコンストラクタ
Particle::Particle() : pose_(0.0, 0.0, 0.0), weight_(0.0)
{
}

// コンストラクタ
Particle::Particle(const double x, const double y, const double yaw, const double weight)
    : pose_(x, y, yaw), weight_(weight)
{
}

// 代入演算子
Particle& Particle::operator =(const Particle& p)
{
    if (this != &p) {
        this->pose_ = p.pose_;
        this->weight_ = p.weight_;
    }
    return *this;
}

// setter
void Particle::set_weight(const double weight)
{
    weight_ = weight;
}

// 尤度関数
double Particle::likelihood(const nav_msgs::msg::OccupancyGrid& map, const sensor_msgs::msg::LaserScan& laser,
        const double sensor_noise_ratio, const int laser_step, const std::vector<double>& ignore_angle_range_list)
{
    double L = 0.0;
    int count = 0;

    for(int i=0; i<(int)laser.ranges.size(); i+=laser_step)
    {
        double angle = laser.angle_min + i * laser.angle_increment;

        // 柱などの除外範囲チェック
        if(is_ignore_angle(angle, ignore_angle_range_list)) continue;

        double measured_dist = laser.ranges[i];
        if(measured_dist < laser.range_min || measured_dist > laser.range_max) continue;

        // 地図上の期待距離を算出
        double expected_dist = calc_dist_to_wall(pose_.x(), pose_.y(), pose_.yaw() + angle, map, laser.range_max, sensor_noise_ratio);

        // ガウス分布（正規分布）による評価
        // センサノイズは距離に比例すると仮定
        double sigma = measured_dist * sensor_noise_ratio;
        L += norm_pdf(measured_dist, expected_dist, sigma);
        count++;
    }

    return (count > 0) ? (L / count) : 0.0;
}

// 柱がある範囲か判定
bool Particle::is_ignore_angle(double angle, const std::vector<double>& ignore_angle_range_list)
{
    for(size_t i=0; i<ignore_angle_range_list.size(); i+=2)
    {
        if(angle >= ignore_angle_range_list[i] && angle <= ignore_angle_range_list[i+1]) return true;
    }
    return false;
}

// 壁までの距離を算出（レイキャスティング）
double Particle::calc_dist_to_wall(double x, double y, const double laser_angle, const nav_msgs::msg::OccupancyGrid& map,
        const double laser_range, const double sensor_noise_ratio)
{
    const double search_step = map.info.resolution;
    const double search_limit = laser_range;

    for(double dist=0.0; dist<search_limit; dist+=search_step)
    {
        double target_x = x + dist * std::cos(laser_angle);
        double target_y = y + dist * std::sin(laser_angle);

        int index = xy_to_grid_index(target_x, target_y, map.info);

        // マップ外または未知領域（-1）の場合
        if(!in_map(index, map.data.size()) || map.data[index] == -1)
        {
            return search_limit * 2.0;
        }

        // 壁（100）を発見した場合
        if(map.data[index] == 100)
        {
            return dist;
        }
    }

    // 壁が見つからなかった場合
    return search_limit * 5.0;
}

// 座標からグリッドのインデックスを返す
int Particle::xy_to_grid_index(const double x, const double y, const nav_msgs::msg::MapMetaData& map_info)
{
    int x_grid = std::floor((x - map_info.origin.position.x) / map_info.resolution);
    int y_grid = std::floor((y - map_info.origin.position.y) / map_info.resolution);

    return y_grid * map_info.width + x_grid;
}

// マップ内か判定
bool Particle::in_map(const int grid_index, const int map_data_size)
{
    return (grid_index >= 0 && grid_index < map_data_size);
}

// 確率密度関数（正規分布）
double Particle::norm_pdf(const double x, const double mean, const double stddev)
{
    // ★修正：stddevが極小・0・負の値になっても安全に動作するよう下限クランプを行う。
    //   - measured_dist がほぼ0のときに sigma = measured_dist * sensor_noise_ratio が0に近づき、
    //     0除算で +inf や NaN になり得る。
    //   - 何らかの異常で stddev が負になった場合、(x-mean)^2 / (2*stddev^2) は計算できても
    //     1/(sqrt(2π)*stddev) が負になり、確率密度が負値になってしまう。
    // → 絶対値を取り、さらに下限値でクランプして安全な値だけを使う。
    constexpr double MIN_STDDEV = 1e-6;
    const double safe_stddev = std::max(std::abs(stddev), MIN_STDDEV);

    const double diff = x - mean;
    return (1.0 / (std::sqrt(2.0 * M_PI) * safe_stddev))
         * std::exp(-(diff * diff) / (2.0 * safe_stddev * safe_stddev));
}