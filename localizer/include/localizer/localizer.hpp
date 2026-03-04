#ifndef LOCALIZER_HPP
#define LOCALIZER_HPP

#include <rclcpp/rclcpp.hpp>
#include <functional>   // bind, placeholders
#include <memory>       // SharedPtr
#include <type_traits>
#include <utility>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <random>

#include "localizer/odom_model.hpp"
#include "localizer/particle.hpp"
#include "localizer/pose.hpp"

class Localizer : public rclcpp::Node
{
    public:
        Localizer();        // デフォルトコンストラクタ
        int getOdomFreq();  // 制御周期(hz_)を返す関数
        void initialize();  // パーティクルの初期化
        void process();     // main文のループ内で実行する関数
    private:
        // ----- 関数（引数あり）------
        // コールバック関数
        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        // その他の関数
        double normalize_angle(double angle);                    // 適切な角度(-M_PI ~ M_PI)を返す
        double norm_rv(const double mean, const double stddev);  // ランダム変数生成関数（正規分布）
        void   resampling(const double alpha);                   // リサンプリング（系統サンプリング）

        // ----- 関数（引数なし）------
        void   reset_weight();             // パーティクルの重みの初期化
        void   broadcast_odom_state();     // map座標系からみたodom座標系の位置と姿勢をtfでbroadcast
        void   localize();                 // 自己位置推定
        void   motion_update();            // 動作更新
        void   observation_update();       // 観測更新
        void   estimate_pose();            // 推定位置の決定
        void   normalize_belief();         // 重みの正規化
        void   expansion_resetting();      // 膨張リセット
        void   publish_estimated_pose();   // 推定位置のパブリッシュ
        void   publish_particles();        // パーティクルクラウドのパブリッシュ
        double calc_marginal_likelihood(); // 周辺尤度の算出

        // Subscriber
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;

        // Publisher
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_estimated_pose_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_particle_cloud_;

        // ----- 変数 -----
        // 基本設定
        int       hz_;                 // ループ周波数 [Hz]
        int       particle_num_;       // パーティクルの個数 [-]
        int       max_particle_num_;   // パーティクルの個数 [-]
        int       min_particle_num_;   // パーティクルの個数 [-]
        double    move_dist_th_;       // ロボットの移動開始判断用（スタートからの距離の閾値）[m]    int       reset_counter = 0;   // 連続リセットの回数 [-]
        // 初期ポーズ関連
        double    init_x_;             // 初期位置 [m]
        double    init_y_;             // 初期位置 [m]
        double    init_yaw_;           // 初期姿勢 [rad]
        double    init_x_dev_;         // 初期位置xの標準偏差 [m]
        double    init_y_dev_;         // 初期位置yの標準偏差 [m]
        double    init_yaw_dev_;       // 初期姿勢の標準偏差 [rad]
        // リセット関連
        double    alpha_th_;           // リセットに関する平均尤度の閾値 [-]
        int       reset_counter = 0;   // 連続リセットの回数 [-]
        int       reset_count_limit_;  // 連続リセットの回数の上限 [-]
        double    expansion_x_dev_;    // 膨張リセットの位置xの標準偏差 [m]
        double    expansion_y_dev_;    // 膨張リセットの位置yの標準偏差 [m]
        double    expansion_yaw_dev_;  // 膨張リセットの姿勢の標準偏差 [rad]
        // センサ関連
        int       laser_step_;         // 何本ずつレーザを見るか [-]
        double    sensor_noise_ratio_; // 距離に対するセンサノイズ比 [-]

        Pose      estimated_pose_;     // 推定位置
        OdomModel odom_model_;         // odometryのモデル

        // リスト
        std::vector<Particle> particles_;             // パーティクルクラウド（計算用）
        std::vector<double> ignore_angle_range_list_; // 柱に関する角度範囲の配列 [rad]

        // msg受け取りフラグ
        bool flag_map_     = false;
        bool flag_odom_    = false;
        bool flag_laser_   = false;

        // その他のフラグ
        bool flag_move_ = false; // 機体が動いたか判断用
        bool flag_init_noise_;   // 初期位置にノイズを加えるか
        bool flag_broadcast_;    // tf broadcastをするか
        bool flag_reverse_;      // 初期姿勢を逆にするか
        bool is_visible_;        // パーティクルクラウドをパブリッシュするか

        // OdomModel関連
        double ff_;     // 直進1[m]で生じる道のりのばらつきの標準偏差 [m]
        double fr_;     // 回転1[rad]で生じる道のりのばらつきの標準偏差 [m]
        double rf_;     // 直進1[m]で生じるロボットの向きのばらつきの標準偏差 [rad]
        double rr_;     // 回転1[rad]で生じるロボットの向きのばらつきの標準偏差 [rad]

        // 正規分布用乱数
        std::random_device seed_gen_;
        std::default_random_engine engine_;

        // 各種オブジェクト
        nav_msgs::msg::OccupancyGrid    map_;                // map_serverから受け取るマップ
        nav_msgs::msg::Odometry         last_odom_;          // 最新のodometry
        nav_msgs::msg::Odometry         prev_odom_;          // 1制御周期前のodometry
        sensor_msgs::msg::LaserScan     laser_;              // レーザ値
        geometry_msgs::msg::PoseStamped estimated_pose_msg_; // 推定位置
        geometry_msgs::msg::PoseArray   particle_cloud_msg_; // パーティクルクラウド（パブリッシュ用）
};

#endif