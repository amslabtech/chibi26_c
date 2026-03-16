#ifndef LOCAL_MAP_CREATOR_HPP
#define LOCAL_MAP_CREATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

class LocalMapCreator : public rclcpp::Node
{
    public:
        LocalMapCreator(); // デフォルトコンストラクタ
        void process();
        int getFreq();

    private:
        // ----- 関数（引数あり) ------
        // コールバック関数
        void obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

        // その他の関数
        bool in_map(const double dist, const double angle);         // マップ内か判断
        int  get_grid_index(const double dist, const double angle); // グリッドのインデックスを返す
        int  xy_to_grid_index(const double x, const double y);      // グリッドのインデックスを返す

        // ----- 関数（引数なし）-----
        void init_map();   // マップの初期化
        void update_map(); // マップの更新

        // ----- 変数 -----
        int    hz_;       // ループ周波数 [Hz]
        double map_size_; // マップの一辺の長さ [m]
        double map_reso_; // マップの解像度 [m/cell]

        // msg受け取りフラグ
        bool flag_obs_poses_ = false;

        // Subscriber
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_obs_poses_;

        // Publisher
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_local_map_;

        // 各種オブジェクト
        geometry_msgs::msg::PoseArray obs_poses_; // 障害物のポーズ配列
        nav_msgs::msg::OccupancyGrid  local_map_; // ローカルマップ
};

#endif
