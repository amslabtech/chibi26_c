#include "local_goal_creator/local_goal_creator.hpp"
#include <cmath>

LocalGoalCreator::LocalGoalCreator() : Node("LocalGoalCreator")
{
    // ### 初期値の設定 ###
    // ループ周期 [Hz]
    this->declare_parameter("hz", 10);
    // １回で更新するインデックス数
    this->declare_parameter("index_step", 1);
    // グローバルパス内におけるローカルゴールのインデックス
    this->declare_parameter("goal_index", 0);
    // 現在位置-ゴール間の距離 [m]
    this->declare_parameter("target_dist_to_goal", 1.5);

    hz_ = this->get_parameter("hz").as_int();
    index_step_ = this->get_parameter("index_step").as_int();
    goal_index_ = this->get_parameter("goal_index").as_int();
    // YAMLの target_dist_to_goal を target_dist_to_goal_ に格納
    target_dist_to_goal_ = this->get_parameter("target_dist_to_goal").as_double();

    // ### pubやsubの定義，tfの統合 ###
    // -- パブリッシャーの作成(PointStamped型) --
    // トピック名: "local_goal"
    local_goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("local_goal", 10);

    // -- サブスクライバーの作成 --
    // トピック名: "global_path"
    // 実行する関数: pathCallback (この後、クラス内に定義する必要がある)
    // グローバルパスを購読
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "global_path", 10, std::bind(&LocalGoalCreator::pathCallback, this, std::placeholders::_1));
    
    // 自己位置を受け取る
    // トピック名: "estimated_pose"
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "estimated_pose", 10, std::bind(&LocalGoalCreator::poseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Local Goal Creator node has started.");
}

void LocalGoalCreator::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)//subのコールバック関数
{
    pose_ = *msg;
}

void LocalGoalCreator::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)//subのコールバック関数
{
    path_ = *msg;
    is_path_ = true;
    // 新しいパスが来たら、追従開始インデックスをリセット
    goal_index_ = 0;
}

int LocalGoalCreator::getOdomFreq()//hzを返す関数（無くてもいい）
{
    return hz_;
}

void LocalGoalCreator::process()//main文ので実行する関数
{
    // ### pathが読み込めた場合にpublishGoal関数を実行 ###
    // パスを受信していない場合は処理をスキップ
    if (!is_path_ || path_.poses.empty()) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for Path...");
        return;
    }

    publishGoal();
}

void LocalGoalCreator::publishGoal()
{
    // ### ゴールまでの距離の計算を行う ###
    // 設定値に応じて，ゴール位置の変更を行う
    // --- ローカルゴールの更新ロジック ---
    // 現在位置と、パス上のターゲット点(goal_index)の距離を計算
    // しきい値より近ければ、次の点へインデックスを進める
    while (goal_index_ < (int)path_.poses.size() - 1) {
        double dist = getDistance();
        if (dist < target_dist_to_goal_) {
            goal_index_ += index_step_;
        } else {
            break;
        }
    }

    // 配列外参照を防ぐガード
    if (goal_index_ >= (int)path_.poses.size()) {
        goal_index_ = (int)path_.poses.size() - 1;
    }

    // --- 2. PointStamped 型のメッセージを作成 ---
    geometry_msgs::msg::PointStamped point_msg;

    // ヘッダー情報のコピー (frame_id を "map" などに合わせる)
    point_msg.header.frame_id = path_.header.frame_id;
    point_msg.header.stamp = this->get_clock()->now();

    // 座標の代入 (PoseStamped の position 部分だけを抽出)
    point_msg.point = path_.poses[goal_index_].pose.position;

    // --- 3. パブリッシュ ---
    local_goal_pub_->publish(point_msg);
}

double LocalGoalCreator::getDistance()//距離計算関数（使わなくても平気）
{
    // 現在のターゲット点と自己位置の直線距離を計算
    double dx = path_.poses[goal_index_].pose.position.x - pose_.pose.position.x;
    double dy = path_.poses[goal_index_].pose.position.y - pose_.pose.position.y;
    
    // std::hypot(x, y) は sqrt(x*x + y*y) と同等
    return std::hypot(dx, dy);
}