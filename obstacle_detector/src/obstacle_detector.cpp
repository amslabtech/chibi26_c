#include "obstacle_detector/obstacle_detector.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <chrono> // std::chrono::milliseconds を使うために追加
using std::placeholders::_1;

#include <vector>
#include <algorithm>

ObstacleDetector::ObstacleDetector()
: Node("c_obstacle_detector")
{
   // global変数を定義(yamlファイルからパラメータを読み込めるようにすると，パラメータ調整が楽)
    hz_            = this->declare_parameter<int>("hz", 10);
    laser_step_    = this->declare_parameter<int>("laser_step", 1);
    robot_frame_   = this->declare_parameter<std::string>("robot_frame", "base_link");
    ignore_dist_   = this->declare_parameter<double>("ignore_dist", 200);
    ignore_pillar_ = this->declare_parameter<double>("ignore_pillar", 0.40);

    // Sub: /scan_sub_
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&ObstacleDetector::scan_callback, this, _1));
    // Pub: /obstacle_pub_
    obstacles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("obstacles", 10);

    //timer
    //using namespace std::chrono_literals -> コード内で 100ms や 1s のように、 ms や s という単位を使うためのもの
    auto period = std::chrono::milliseconds(1000 / hz_);
    timer_ = this->create_wall_timer(
        period,
        std::bind(&ObstacleDetector::process, this));   //この4行で100msごとにprocess()を呼ぶというテンプレみたいな
}

//Lidarから障害物の情報を取得
void ObstacleDetector::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    scan_ = *msg;
}

//一定周期で行う処理(obstacle_detectorの処理)
void ObstacleDetector::process()
{
    //scan_obstacle()を呼び出す
    if(!scan_obstacle())
    {
        return;
    }
}

//Lidarから障害物情報を取得し，障害物の座標をpublish　※メッセージの型は自分で決めてください
bool ObstacleDetector::scan_obstacle()
{
    if(!scan_.has_value())  //値が得られなかった場合の処理
    {
        return false;
    }
    // scan_ 変数の中から、実際のLiDARデータ（LaserScan型）を取り出し、scan_dataという名前で参照できるようにする
    const auto & scan_data = scan_.value();  //scan_ ・・・ std::optional 型のメンバ変数。[箱]
                                             //.value　・・・ std::optional 型が持つ専用の機能。[箱の中のデータを取り出す]

    geometry_msgs::msg::PoseArray pose_array;  //PoseArray型の空の変数を作成する（複数の座標をまとめて入れる箱）
    
    pose_array.header.stamp = scan_data.header.stamp; //配信データとLiDARからの受信データの時刻をそろえる
    
    pose_array.header.frame_id = robot_frame_; //基準座標系を、yamlで設定したrobot_frame_に

    // LiDARの計測データの最初から最後まで、laser_step_飛ばしでループを回す。（ranges配列）
    for (size_t i = 0; i < scan_data.ranges.size(); i += laser_step_)
    {
        double range = scan_data.ranges[i];  // i番目の配列に入っている、LiDARから障害物までの距離mを取得
        // LiDARの計測開始角度(angle_min)に、1本あたりの角度間隔(angle_increment) × インデックス(i) を足して、現在のレーザーの「角度[rad]」を計算する
        double angle = scan_data.angle_min + i * scan_data.angle_increment;

        if (is_ignore_scan(range, angle))  // 外れ値かどうか確認する
        {
            continue;  // 外れ値だった場合、これ以降の座標計算はスキップして、次のレーザーの処理（次のi）へ進む
        }

        geometry_msgs::msg::Pose pose;  //// Pose型（位置と向きを持つ型）の変数を作成する。障害物の座標を格納。
        
        // x,yを極座標から直交座標
        pose.position.x = range * std::cos(angle);
        pose.position.y = range * std::sin(angle);
        pose.position.z = 0.0;  //z座標0
        
        // 回転なし
        pose.orientation.w = 1.0;  // w　・・・　3D空間での物体の回転（姿勢）

        // 計算が終わった1点分の座標データ(pose)を、配信用配列(pose_array.poses)の最後尾に追加(push_back)する
        pose_array.poses.push_back(pose);
    }

    obstacles_pub_->publish(pose_array);  //抽出した全障害物の座標が詰まったpose_array配列をPublish

    return true;
    
}


//無視するlidar情報の範囲の決定(lidarがroombaの櫓の中にあり，櫓の４つの柱を障害物として検出してしまうため削除が必要)
bool ObstacleDetector::is_ignore_scan(double range, double angle) const
{
    // LaserScanはinfが来ることがあるので弾く
    if (!std::isfinite(range))
    {
        return true;
    }

    // 遠すぎるものを外れ値として除去
    if (range >= ignore_dist_)
    {
        return true;
    }

    // 角度で除外（4か所）
    // if (
    //     (-2.50 <= angle && angle <= -2.20) || // 右後
    //     ( 2.20 <= angle && angle <=  2.50) || // 左後
    //     ( 0.65 <= angle && angle <=  0.95) || // 左前
    //     (-0.95 <= angle && angle <= -0.65)    // 右前
    // )
    const double delta = 0.0436; // 約2.5度（1.5° + 1°）

    if (
        (-2.50 - delta <= angle && angle <= -2.20 + delta) || // 右後
        ( 2.20 - delta <= angle && angle <=  2.50 + delta) || // 左後
        ( 0.65 - delta <= angle && angle <=  0.95 + delta) || // 左前
        (-0.95 - delta <= angle && angle <= -0.65 + delta)    // 右前
    )
    {
        return true;
    }

    return false;
}
