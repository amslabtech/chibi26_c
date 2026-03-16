#include "local_map_creator/local_map_creator.hpp"

// コンストラクタ
LocalMapCreator::LocalMapCreator() : Node("local_map_creater")
{
    // パラメータの取得(hz, map_size, map_reso)

    // Subscriberの設定

    // Publisherの設定

    // --- 基本設定 ---
    // マップの基本情報(local_map_)を設定する（header, info, data）
    //   header
    //   info(width, height, position.x, position.y)
    //   data
}

// obs_posesのコールバック関数
void LocalMapCreator::obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
}

// 周期処理の実行間隔を取得する
int LocalMapCreator::getFreq()
{
}

// 障害物情報が更新された場合、マップを更新する
void LocalMapCreator::process()
{
}

// 障害物の情報をもとにローカルマップを更新する
void LocalMapCreator::update_map()
{
    // マップを初期化する

    // 障害物の位置を考慮してマップを更新する

    // 更新したマップをpublishする
}

// マップの初期化(すべて「未知」にする)
void LocalMapCreator::init_map()
{
}

// マップ内の場合、trueを返す
bool LocalMapCreator::in_map(const double dist, const double angle)
{
    // 指定された距離と角度がマップの範囲内か判定する
}

// 距離と角度からグリッドのインデックスを返す
int LocalMapCreator::get_grid_index(const double dist, const double angle)
{
}

// 座標からグリッドのインデックスを返す
int LocalMapCreator::xy_to_grid_index(const double x, const double y)
{
}
