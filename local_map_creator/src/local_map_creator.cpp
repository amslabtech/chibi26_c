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
    map_size_    = this->declare_parameter<double>("map_size", 50);  //50m*50mの世界と仮定
    map_reso_    = this->declare_parameter<double>("map_reso", 0.05 );  //1マスが何mか

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
    flag_obs_poses_ = true;  //データキタ！の合図
}

// 周期処理の実行間隔を取得する
int LocalMapCreator::getFreq()
{
    return 1000/hz_;
}

// 障害物情報が更新された場合、マップを更新する
void LocalMapCreator::process()
{
    if(flag_obs_poses_)
    {
        update_map();
        flag_obs_poses_ = false; //更新が終わったらハタ下す
    }
}

// 障害物の情報をもとにローカルマップを更新する
void LocalMapCreator::update_map()
{
    // マップを初期化する
    init_map();
    // 障害物の位置を考慮してマップを更新する
    for (const auto& pose : obs_poses_.poses)
    {        
        // ObstacleDetectorで変換済みのX, Y
        double x = pose.position.x;
        double y = pose.position.y;
        
        // 座標からインデックスへ変換 (ここでマップ外の判定も同時に行われる)
        int index = xy_to_grid_index(x, y);
        
        // インデックスが -1(マップ外) でなければ、障害物をプロット
        if (index != -1)
        {
            local_map_.data[index] = 100; 
        }
    }
    // 更新したマップをpublishする
    local_map_.header.stamp = this->now();  //マップのヘッダー情報のタイムスタンプを、現在時刻(this->now)にセット
    pub_local_map_->publish(local_map_);
}

// マップの初期化(すべて「未知」にする)
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