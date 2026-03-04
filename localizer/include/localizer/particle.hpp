#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "localizer/pose.hpp"

class Particle
{
    public:
        Particle(); // デフォルトコンストラクタ
        Particle(const double x, const double y, const double yaw, const double weight); // コンストラクタ
        Particle& operator =(const Particle& p); // 代入演算子

        // accessor
        void set_weight(const double weight);   // 重みのセット
        double weight() const { return weight_; }

        // 尤度関数
        double likelihood(const nav_msgs::msg::OccupancyGrid& map, const sensor_msgs::msg::LaserScan& laser,
            const double sensor_noise_ratio, const int laser_step, const std::vector<double>& ignore_angle_range_list);

        // --- メンバ変数 ---
        Pose pose_;

    private:
        // 柱か判断
        bool is_ignore_angle(double angle, const std::vector<double>& ignore_angle_range_list);

        // 壁までの距離を算出
        double calc_dist_to_wall(double x, double y, const double laser_angle, const nav_msgs::msg::OccupancyGrid& map,
            const double laser_range, const double sensor_noise_ratio);

        // 座標からグリッドのインデックスを返す
        int  xy_to_grid_index(const double x, const double y, const nav_msgs::msg::MapMetaData& map_info);

        // マップ内か判断
        bool in_map(const int grid_index, const int map_data_size);

        // 確率密度関数（正規分布）
        double norm_pdf(const double x, const double mean, const double stddev);

        // --- メンバ変数 ---
        double weight_; // [-]
};

#endif
