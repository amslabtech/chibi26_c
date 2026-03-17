#ifndef GLOBAL_PATH_PLANNER_HPP
#define GLOBAL_PATH_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <math.h>

// =========================
// 構造体定義
// =========================

// ノード構造体（経路探索で使用）
struct Node_ {
    int    x = 0;
    int    y = 0;
    int    parent_x = -1;
    int    parent_y = -1;
    double f = 0.0;
};

// モーション構造体（移動方向とコストを定義）
struct Motion_ {
    int dx;
    int dy;
    double cost;
};

// =========================
// A*アルゴリズムクラス
// =========================
class Astar : public rclcpp::Node {
public:
    Astar();        // デフォルトコンストラクタ

private:
    // ----- コールバック関数 -----
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);  // マップデータを受信し処理

    // ----- 経路探索関連関数 -----
    void obs_expand(const int index);                // 指定されたインデックスの障害物を拡張
    double make_heuristic(const Node_ node);         // ノードのヒューリスティック値を計算
    Node_ set_way_point(const int phase);            // スタート・ゴールノードの設定
    void create_path(Node_ node);                    // ノードをたどり，パスを作成
    geometry_msgs::msg::PoseStamped node_to_pose(const Node_ node);  // ノード情報を座標へ変換
    void swap_node(const Node_ node, std::vector<Node_>& list1, std::vector<Node_>& list2); // ノードをリスト間で移動
    int check_list(const Node_ target_node, std::vector<Node_>& set); // 指定リスト内のノード検索（インデックスを取得）
    void update_list(const Node_ node);              // 隣接ノードを探索しリストを更新
    void create_neighbor_nodes(const Node_ node, std::vector<Node_>& nodes); // 指定ノードの隣接ノードを作成
    void get_motion(std::vector<Motion_>& motion);   // 動作モデル（移動方向とコスト）を定義
    Motion_ motion(const int dx, const int dy, const int cost);  // モーション構造体を作成
    Node_ get_neighbor_node(const Node_ node, const Motion_ motion); // 指定モーションを適用したノードを取得
    std::tuple<int, int> search_node(const Node_ node);  // 指定ノードの検索（座標取得）
    int search_node_from_list(const Node_ node, std::vector<Node_>& list);  // 指定リスト内のノード検索

    // ----- ノードチェック関数 -----
    bool check_start(const Node_ node);  // 指定ノードがスタートノードか判定
    bool check_goal(const Node_ node);   // 指定ノードがゴールノードか判定
    bool check_same_node(const Node_ n1, const Node_ n2);  // 2つのノードが同一か判定
    bool check_obs(const Node_ node);    // 指定ノードが障害物か判定
    bool check_parent(const int index, const Node_ node); // 指定ノードが親ノードか確認

    // ----- 経路計画関連関数 -----
    void process(); // mapを受け取ったら実行する関数（obs_expaner，planningを実行する）
    void obs_expander();  // マップ全体の障害物を拡張処理
    void planning();      // A*アルゴリズムによる経路計画を実行
    Node_ select_min_f(); // openリスト内で最小のf値を持つノードを選択

    // ----- デバッグ用関数 -----
    void show_node_point(const Node_ node);  // Rviz上でノードをポイントとして可視化
    void show_path(nav_msgs::msg::Path& current_path);  // Rviz上で経路を可視化
    void show_exe_time();  // 経路計画の実行時間を表示

    // =========================
    // ROS関連メンバ変数
    // =========================

    // Subscriber
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_; // マップ購読

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_; // 計算した経路を発行
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_node_point_; // 可視化用ノード位置を発行
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_current_path_; // 現在のパスを発行
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_new_map_; // 拡張後のマップを発行

    // =========================
    // 変数定義
    // =========================

    // 基本設定
    double sleep_time_; // デバッグ用スリープ時間 [s]
    double margin_; // 障害物拡張マージン [m]

    // ノード情報
    Node_ start_node_;  // 開始ノード
    Node_ goal_node_;   // 目標ノード
    std::vector<Node_> open_list_;  // openリスト
    std::vector<Node_> close_list_; // closeリスト

    // マップ情報
    double origin_x_;  // 原点のx座標
    double origin_y_;  // 原点のy座標
    std::vector<double> way_points_x_;  // 経由点(x座標)
    std::vector<double> way_points_y_;  // 経由点(y座標)
    bool map_checker_ = false; // マップ取得の確認フラグ
    bool test_show_; // デバッグ用フラグ

    // マップ詳細
    int height_;  // マップの高さ
    int width_;   // マップの幅
    double resolution_; // マップ解像度
    Node_ origin_;  // マップ原点
    std::vector<std::vector<int>> map_grid_; // グリッドマップ

    // 時間計測
    rclcpp::Clock clock_; // 時間計測用
    rclcpp::Time begin_;  // 開始時間

    // 経路データ
    nav_msgs::msg::Path global_path_; // グローバルパス
    nav_msgs::msg::OccupancyGrid map_; // マップデータ
    nav_msgs::msg::OccupancyGrid new_map_; // 拡張マップ
    geometry_msgs::msg::PointStamped current_node_; // 現在のノード
};

#endif