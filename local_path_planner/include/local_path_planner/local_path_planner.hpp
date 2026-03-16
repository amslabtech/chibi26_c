/*
DWA: Dynamic Window Approach

速度について以下のようにする
velocity(vel) : 並進速度
yawrate       : 旋回速度
speed         : 速度の総称(vel, yawrate)
*/

#ifndef LOCAL_PATH_PLANNER_A_HPP
#define LOCAL_PATH_PLANNER_A_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "roomba_500driver_meiji/msg/roomba_ctrl.hpp"

// =========================
// 構造体定義
// =========================

// ロボットの状態を表す構造体
struct State
{
    double x;        // [m]
    double y;        // [m]
    double yaw;      // [rad]
    double velocity; // [m/s]
    double yawrate;  // [rad/s]
};

// 動的ウィンドウを表す構造体
struct DynamicWindow
{
    double min_vel;     // [m/s]
    double max_vel;     // [m/s]
    double min_yawrate; // [rad/s]
    double max_yawrate; // [rad/s]
};


// =========================
// DWA_Plannerクラス
// =========================
class DWAPlanner : public rclcpp::Node
{
public:
    DWAPlanner(); // デフォルトコンストラクタ
    void process(); // nodeで実行するメイン関数
    int get_freq(); // hzを返す関数

private:
    // ----- 引数あり関数 -----
    // コールバック関数
    void local_goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    //void timer_callback();

    // その他の関数
    void   roomba_control(const double velocity, const double yawrate);                                         // Roombaの制御入力
    void   move(State& state, const double velocity, const double yawrate);                                     // 予測軌跡作成時における仮想ロボットの移動
    void   visualize_traj(const std::vector<State>& traj, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_path, rclcpp::Time now); // 軌跡を可視化
    double normalize_angle(double angle);                                                                        // 適切な角度(-M_PI ~ M_PI)を返す
    double calc_evaluation(const std::vector<State>& traj);                                                     // 評価関数を計算
    double calc_heading_eval(const std::vector<State>& traj);                                                   // headingの評価関数を計算
    double calc_dist_eval(const std::vector<State>& traj);                                                      // distの評価関数を計算
    double calc_vel_eval(const std::vector<State>& traj);                                                       // velocityの評価関数を計算
    std::vector<State> calc_traj(const double velocity, const double yawrate);                                  // 予測軌跡の作成


    // ----- 引数なし関数 -----
    void calc_dynamic_window();             // Dynamic Windowを計算
    void change_mode();
    bool can_move();                        // ゴールに着くまでTrueを返す
    std::vector<double> calc_final_input(); // 最適な制御入力を計算


    // ----- 変数 -----
    bool   is_visible_;         // パスを可視化するかの設定用
    int    hz_ = 10;                 // ループ周波数
    int    mode_;
    double max_vel_;            // 最高並進速度 [m/s]（計算用）
    double max_vel1_;           // 最高並進速度 [m/s]（平常時）
    double max_vel2_;           // 最高並進速度 [m/s]（減速時）
    double turn_thres_yawrate_; // 旋回中か判断する閾値
    double avoid_thres_vel_;    // 回避中か判断する閾値
    double mode_log_time_;      // logをとる時間区間
    double min_vel_;            // 最低並進速度 [m/s]
    double max_yawrate_;        // 最高旋回速度 [rad/s]（計算用）
    double max_yawrate1_;       // 最高旋回速度 [rad/s]（平常時）
    double max_yawrate2_;       // 最高旋回速度 [rad/s]（減速時）
    double max_accel_;          // 最高並進加速度 [m/s^2]
    double max_dyawrate_;       // 最高旋回加速度 [rad/s^2]
    double vel_reso_;           // 並進速度解像度 [m/s]
    double yawrate_reso_;       // 旋回速度解像度 [rad/s]
    double stop_vel_th_;        // 停止状態か判断する閾値 [m/s^2]
    double stop_yawrate_th_;    // 停止状態か判断する閾値 [rad/s]
    double dt_;                 // 微小時間 [s]
    double predict_time_;       // 軌跡予測時間 [s]
    double predict_time1_;      // 軌跡予測時間 [s]
    double predict_time2_;      // 軌跡予測時間 [s]
    double roomba_radius_;      // Roombaのサイズ(半径) [m]
    double radius_margin_;      // 半径の余白 [m]（計算用）
    double radius_margin1_;     // 半径の余白 [m]（平常時）
    double radius_margin2_;     // 半径の余白 [m]（減速時）
    double goal_tolerance_;     // 目標地点の許容誤差 [m]
    double search_range_;       // 評価関数distで探索する範囲 [m]
    std::string robot_frame_;   // base_link

    // msgの受け取りフラグ
    bool flag_local_goal_ = false;
    bool flag_obs_poses_  = false;

    // mode記録用
    std::vector<double> mode_log_;

    // 重み定数
    double weight_heading_;  //（計算用）
    double weight_heading1_; //（平常時）
    double weight_heading2_; //（減速時）
    double weight_dist_;     //（計算用）
    double weight_dist1_;    //（平常時）
    double weight_dist2_;    //（減速時）
    double weight_vel_;
    
    // 時間計測
    rclcpp::Clock clock_; // 時間計測用
    
    // ----- その他のオブジェクト -----
    State roomba_;
    DynamicWindow dw_;

    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_local_goal_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_obs_poses_;

    // Publisher
    rclcpp::Publisher<roomba_500driver_meiji::msg::RoombaCtrl>::SharedPtr pub_cmd_speed_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_optimal_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_predict_path_;

    // pose関連
    geometry_msgs::msg::PointStamped local_goal_; // local path用の目標位置
    geometry_msgs::msg::PoseArray    obs_poses_;  // 障害物のポーズの配列

    // tf
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 制御入力
    roomba_500driver_meiji::msg::RoombaCtrl cmd_speed_;
};

#endif