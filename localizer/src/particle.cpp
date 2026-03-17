#include "localizer/particle.hpp"

// デフォルトコンストラクタ
Particle::Particle() : pose_(0.0, 0.0, 0.0)
{

}

// コンストラクタ
Particle::Particle(const double x, const double y, const double yaw, const double weight) : pose_(x, y, yaw)
{

}

// 代入演算子
Particle& Particle::operator =(const Particle& p)
{

}

// setter
void Particle::set_weight(const double weight)
{

}

// 尤度関数
// センサ情報からパーティクルの姿勢を尤度を算出
double Particle::likelihood(const nav_msgs::msg::OccupancyGrid& map, const sensor_msgs::msg::LaserScan& laser,
        const double sensor_noise_ratio, const int laser_step, const std::vector<double>& ignore_angle_range_list)
{
        double L = 0.0; // 尤度
        // センサ情報からパーティクルの姿勢を評価

        return L;
}

// 柱がある範囲か判定
bool Particle::is_ignore_angle(double angle, const std::vector<double>& ignore_angle_range_list)
{

}

// 与えられた座標と角度の方向にある壁までの距離を算出
// マップデータが100の場合，距離を返す
// マップデータが-1（未知）の場合，マップ範囲外の場合はsearch_limit * 2.0を返す
// いずれでもない場合は，search_limit * 5.0を返す
double Particle::calc_dist_to_wall(double x, double y, const double laser_angle, const nav_msgs::msg::OccupancyGrid& map,
        const double laser_range, const double sensor_noise_ratio)
{
        // 探索のステップサイズ
        const double search_step = map.info.resolution;
        // 最大探索距離
        const double search_limit = laser_range;

        // 探索
        for(double dist=0.0; dist<search_limit; dist+=search_step)
        {

        }
        
        return search_limit * sensor_noise_ratio * 5.0;
}

// 座標からグリッドのインデックスを返す
int Particle::xy_to_grid_index(const double x, const double y, const nav_msgs::msg::MapMetaData& map_info)
{

}

// マップ内か判定
bool Particle::in_map(const int grid_index, const int map_data_size)
{

}

// 確率密度関数（正規分布）
double Particle::norm_pdf(const double x, const double mean, const double stddev)
{

}
