#include "local_map_creator/local_map_creator.hpp"

// コンストラクタ
#include <cmath>
#include <functional>
#include <limits>
#include <chrono> // std::chrono::milliseconds を使うために追加
using std::placeholders::_1;

LocalMapCreator::LocalMapCreator() : Node("local_map_creator")
{
    // パラメータの取得(hz, map_size, map_reso)
    hz_          = this->declare_parameter<int>("hz", 10);
    map_size_    = this->declare_parameter<double>("map_size", 4);  //1m*1mの世界と仮定
    map_reso_    = this->declare_parameter<double>("map_reso", 0.005 );  //1マスが何mか

    // Sub: /sub_obs_poses
    sub_obs_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "obstacles", 10, std::bind(&LocalMapCreator::obs_poses_callback, this, _1));
    // Pub: /pub_local_map_
    pub_local_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("local_map", 10);  //OccupancyGrid:二次元の格子で座標を表現

    // --- 基本設定 ---
    // マップの基本情報(local_map_)を設定する（header, info, data）
    //   header
    local_map_.header.frame_id = "base_link";
    //   info(width, height, position.x, position.y)  
    int grid_size = static_cast<int>(map_size_ / map_reso_); //マップの1辺が何マスになるか
    local_map_.info.width = grid_size;
    local_map_.info.height = grid_size;
    local_map_.info.resolution = map_reso_;    // 1マスが何メートルかを設定
    // ロボットの初期位置がマップのどこなのか
    local_map_.info.origin.position.x = -map_size_ / 2.0;
    local_map_.info.origin.position.y = -map_size_ / 2.0;
    local_map_.info.origin.position.z = 0.0;
    local_map_.info.origin.orientation.x = 0.0;
    local_map_.info.origin.orientation.y = 0.0;
    local_map_.info.origin.orientation.z = 0.0;
    local_map_.info.origin.orientation.w = 1.0;
    //   data
    // グリッドの総数(width * height)分だけ配列のサイズを確保し、すべて-1(未知)で塗りつぶす
    local_map_.data.assign(grid_size * grid_size, -1);

    //timer
    //using namespace std::chrono_literals -> コード内で 100ms や 1s のように、 ms や s という単位を使うためのもの
    auto period = std::chrono::milliseconds(1000 / hz_);
    timer_ = this->create_wall_timer(
        period,
        std::bind(&LocalMapCreator::process, this));   //この4行で100msごとにprocess()を呼ぶというテンプレみたいな
}

// obs_posesのコールバック関数
void LocalMapCreator::obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    obs_poses_ = *msg;
    //flag_obs_poses_ = true;  //データキタ！の合図
}

// 周期処理の実行間隔を取得する
int LocalMapCreator::getFreq()
{
    return hz_;
}

// 障害物情報が更新された場合、マップを更新する
void LocalMapCreator::process()
{
    /*if(flag_obs_poses_)
    {
        update_map();
        flag_obs_poses_ = false; //更新が終わったらハタ下す
    }*/
    init_map(); //マップ初期化
    update_map();

    // 更新したマップをpublishする
    local_map_.header.stamp = this->now();
    pub_local_map_->publish(local_map_);

}

// 障害物の情報をもとにローカルマップを更新する
void LocalMapCreator::update_map()
{
    for (const auto& pose : obs_poses_.poses)
    {        
        double x = pose.position.x;
        double y = pose.position.y;
        double dist = std::sqrt(x * x + y * y);
        // atan2を使ってx,yから角度(ラジアン)を算出
        double angle = std::atan2(y, x); 

        // --- 1. 原点から障害物までを白く(0)塗るロジック (変更なし) ---
        if (dist > 1e-6)
        {
            double step = map_reso_ * 0.5;
            int n = static_cast<int>(dist / step);

            for (int i = 0; i < n; ++i)
            {
                double t = static_cast<double>(i) / static_cast<double>(n);
                int free_index = xy_to_grid_index(t * x, t * y);

                if (free_index != -1)
                {
                    local_map_.data[free_index] = 0;
                }
            }
        }
    
        // --- 2. 柱判定の追加 (ここだけが追加・変更点) ---
        // 柱の距離(例: 0.45m以内)かつ、指定した角度の範囲内か判定
        if (dist < 0.3) 
        {
            if ((-2.50 <= angle && angle <= -2.20) || 
                ( 2.20 <= angle && angle <=  2.50) || 
                ( 0.65 <= angle && angle <=  3.14) || 
                (-0.95 <= angle && angle <= -0.65))
            {
                // ここが実行される＝柱である
                int pillar_index = xy_to_grid_index(x, y);
                if (pillar_index != -1)
                {
                    local_map_.data[pillar_index] = 0; // 黒(100)ではなく白(0)で上書き
                }
                // このデータの処理はここで終了(下の「黒塗り」へ行かない)
                continue; 
            }
        }

        // --- 3. 障害物の点を黒(100)で塗る (変更なし) ---
        int index = xy_to_grid_index(x, y);
        if (index != -1)
        {
            local_map_.data[index] = 100; 
        }
    }
}

// マップの初期化(すべて「既知(走行可能)」にする)
void LocalMapCreator::init_map()
{
    std::fill(local_map_.data.begin(), local_map_.data.end(), -1);
}

/*  obstacle_detectorで極座標に変換したため、省略。in_mapはxy_to_grid_indexでしょり
// マップ内の場合、trueを返す
bool LocalMapCreator::in_map(const double dist, const double angle)
{
    // 指定された距離と角度がマップの範囲内か判定する
}

// 距離と角度からグリッドのインデックスを返す
int LocalMapCreator::get_grid_index(const double dist, const double angle)
{
}
*/


// 座標からグリッドのインデックスを返す
int LocalMapCreator::xy_to_grid_index(const double x, const double y)
{
    // (X, Y)からマップのマス目(grid_x, grid_y)を計算
    //->現実の座標(m) を「何マス目か」に変換する
    int grid_x = static_cast<int>(std::floor((x - local_map_.info.origin.position.x) / map_reso_));
    int grid_y = static_cast<int>(std::floor((y - local_map_.info.origin.position.y) / map_reso_));

    //in_map処理
    if (grid_x < 0 || grid_x >= static_cast<int>(local_map_.info.width) ||
        grid_y < 0 || grid_y >= static_cast<int>(local_map_.info.height))
        {
            return -1;
        }
        
    //2次元のマス目を「1列の配列の何番目か」に変換する
    return grid_y * local_map_.info.width + grid_x;
}