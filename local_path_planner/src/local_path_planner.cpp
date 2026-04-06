/*
DWA: Dynamic Window Approach

速度について以下のようにする
velocity(vel) : 並進速度
yawrate       : 旋回速度
speed         : 速度の総称(vel, yawrate)
*/

#include "local_path_planner/local_path_planner.hpp"
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

// デフォルトコンストラクタ
// パラメータの宣言と取得
DWAPlanner::DWAPlanner() : Node("local_path_planner"), clock_(RCL_ROS_TIME)
{
    // ###### パラメータの宣言 ######
    this->declare_parameter("is_visible", true);
    this->declare_parameter("hz", 10);
    this->declare_parameter("robot_frame", "base_link");
    this->declare_parameter("max_vel", 2.0);
    this->declare_parameter("max_vel1", 3.0);
    this->declare_parameter("max_vel2", 1.0);
    this->declare_parameter("avoid_thres_vel", 0.1);
    this->declare_parameter("min_vel", 0.0);
    this->declare_parameter("max_yawrate", 0.0);
    this->declare_parameter("max_yawrate1", 1.0);
    this->declare_parameter("max_yawrate2", 0.5);
    this->declare_parameter("turn_thres_yawrate", 0.3);
    this->declare_parameter("max_accel", 1.0);
    this->declare_parameter("max_dyawrate", 1.0);
    this->declare_parameter("vel_reso", 0.01);
    this->declare_parameter("yawrate_reso", 0.01);
    this->declare_parameter("dt", 0.1);
    this->declare_parameter("predict_time1", 3.0);
    this->declare_parameter("predict_time2", 2.0);
    this->declare_parameter("roomba_radius", 0.2);
    this->declare_parameter("radius_margin1", 0.1);
    this->declare_parameter("radius_margin2", 0.05);
    this->declare_parameter("weight_heading1", 0.1);
    this->declare_parameter("weight_dist1", 0.1);
    this->declare_parameter("weight_vel", 0.1);
    this->declare_parameter("goal_tolerance", 0.2);
    this->declare_parameter("search_range", 1.0);
    //3.25篠田追加(yamlで宣言しているが、cppにはなかったもの)
    this->declare_parameter("weight_heading2", 0.1);
    this->declare_parameter("weight_dist2", 0.1);
    this->declare_parameter("mode_log_time", 1.0);
    this->declare_parameter("stop_vel_th", 0.01);
    this->declare_parameter("stop_yawrate_th", 0.01);


    // ###### パラメータの取得 ######
    is_visible_ = this->get_parameter("is_visible").as_bool();
    hz_ = this->get_parameter("hz").as_int();
    robot_frame_ = this->get_parameter("robot_frame").as_string();
    max_vel_ = this->get_parameter("max_vel").as_double();
    max_vel1_ = this->get_parameter("max_vel1").as_double();
    max_vel2_ = this->get_parameter("max_vel2").as_double();
    min_vel_ = this->get_parameter("min_vel").as_double();
    max_yawrate_ = this->get_parameter("max_yawrate").as_double();
    max_yawrate1_ = this->get_parameter("max_yawrate1").as_double();
    max_yawrate2_ = this->get_parameter("max_yawrate2").as_double();
    max_accel_ = this->get_parameter("max_accel").as_double();
    max_dyawrate_ = this->get_parameter("max_dyawrate").as_double();
    vel_reso_ = this->get_parameter("vel_reso").as_double();
    yawrate_reso_ = this->get_parameter("yawrate_reso").as_double();
    dt_ = this->get_parameter("dt").as_double();
    predict_time1_ = this->get_parameter("predict_time1").as_double();
    predict_time2_ = this->get_parameter("predict_time2").as_double();
    roomba_radius_ = this->get_parameter("roomba_radius").as_double();
    radius_margin1_ = this->get_parameter("radius_margin1").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    weight_heading1_ = this->get_parameter("weight_heading1").as_double();
    weight_dist1_ = this->get_parameter("weight_dist1").as_double();
    weight_vel_ = this->get_parameter("weight_vel").as_double();
    //3.25篠田追加(yamlで宣言しているが、cppにはなかったもの)
    avoid_thres_vel_    = this->get_parameter("avoid_thres_vel").as_double();
    turn_thres_yawrate_ = this->get_parameter("turn_thres_yawrate").as_double();
    radius_margin2_     = this->get_parameter("radius_margin2").as_double();
    search_range_       = this->get_parameter("search_range").as_double();
    weight_heading2_ = this->get_parameter("weight_heading2").as_double();
    weight_dist2_    = this->get_parameter("weight_dist2").as_double();
    mode_log_time_   = this->get_parameter("mode_log_time").as_double();
    stop_vel_th_     = this->get_parameter("stop_vel_th").as_double();
    stop_yawrate_th_ = this->get_parameter("stop_yawrate_th").as_double();

    // ###### tf_buffer_とtf_listenerを初期化 ######
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ####### Subscriber #######
    sub_local_goal_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "local_goal", 10, std::bind(&DWAPlanner::local_goal_callback, this, std::placeholders::_1));
    sub_obs_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "obs_poses", 10, std::bind(&DWAPlanner::obs_poses_callback, this, std::placeholders::_1));

    // ###### Publisher ######
    // pub_cmd_vel_ = this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>("roomba_control", 10);
    pub_optimal_path_ = this->create_publisher<nav_msgs::msg::Path>("optimal_path", 10);
    pub_predict_path_ = this->create_publisher<nav_msgs::msg::Path>("predict_paths", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

// local_goalのコールバック関数
// local_goalはマップ座標系(map)だが，実際の移動に合わせるためにルンバ座標系(base_link)に変換する処理を行う
void DWAPlanner::local_goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    printf("local_goal_callback\n");
    geometry_msgs::msg::TransformStamped transform;
    try
    {
        // tfを取得，成功したらフラグを立てる
        geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(robot_frame_, msg->header.frame_id, tf2::TimePointZero);
        tf2::doTransform(*msg, local_goal_, tf);
        flag_local_goal_ = true;
    }
    catch(tf2::TransformException& ex)
    {
        // 取得に失敗した場合，警告を出力しフラグをリセット
        RCLCPP_WARN(this->get_logger(), "TF failure: %s", ex.what());
        flag_local_goal_ = false;
    }
    // 取得した変換を用いて，local_goal_ をロボット座標系 (base_link) に変換


}

// obs_posesのコールバック関数
void DWAPlanner::obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    obs_poses_ = *msg;
    flag_obs_poses_ = true;
}

// hzを返す関数
int DWAPlanner::get_freq()
{
    return hz_;
}

// 唯一，main文で実行する関数
// can_move()がTrueのとき，calc_final_input()を実行し，速度と旋回速度を計算
// それ以外の場合は，速度と旋回速度を0にする
void DWAPlanner::process()
{
    if (can_move()) {
        std::vector<double> input = calc_final_input();
        roomba_control(input[0], input[1]);
    } else {
        roomba_control(0.0, 0.0);
    }
}

// ゴールに着くまでTrueを返す
bool DWAPlanner::can_move()
{
    if (!flag_local_goal_) return false;
    double dist_to_goal = std::hypot(local_goal_.point.x, local_goal_.point.y);
    return dist_to_goal > goal_tolerance_;
}

// Roombaの制御入力を行う
void DWAPlanner::roomba_control(const double velocity, const double yawrate)
{
    geometry_msgs::msg::Twist msg;
    // msg.mode = 11; // Drive Mode
    msg.linear.x = velocity;
    msg.angular.z = yawrate;
    cmd_vel_pub_->publish(msg);
}

// 最適な制御入力を計算
std::vector<double> DWAPlanner::calc_final_input()
{
    // 変数を定義，初期化
    std::vector<double> input{0.0, 0.0};          // {velocity, yawrate}
    std::vector<std::vector<State>> trajectories; // すべての軌跡格納用
    double max_score = -1e6;                      // 評価値の最大値格納用
    int index_of_max_score = 0;                   // 評価値の最大値に対する軌跡のインデックス格納用

    // 旋回状況に応じた減速機能
    change_mode();

    // ダイナミックウィンドウを計算
    calc_dynamic_window();

    // ###### 並進速度と旋回速度のすべての組み合わせを評価 ######
    for (double v = dw_.min_vel; v <= dw_.max_vel; v += vel_reso_) {
        for (double y = dw_.min_yawrate; y <= dw_.max_yawrate; y += yawrate_reso_) {
            std::vector<State> traj = calc_traj(v, y);
            double score = calc_evaluation(traj);

            if (score > max_score) {
                max_score = score;
                input = {v, y};
            }
            if (is_visible_) trajectories.push_back(traj);
        }
    }

    // 現在速度の記録
    roomba_.velocity = input[0];
    roomba_.yawrate  = input[1];

    // ###### pathの可視化 #######
    if (is_visible_ && !trajectories.empty()) {
        visualize_traj(calc_traj(input[0], input[1]), pub_optimal_path_, this->now());
    }

    return input;
}

// 旋回状況に応じた減速機能
// ロボットの旋回速度や速度によって減速モードに切り替える（普段よりも遅く動く）
void DWAPlanner::change_mode()
{
    // 旋回速度が閾値を超えていたら慎重モード
    if (std::abs(roomba_.yawrate) > turn_thres_yawrate_) {
        max_vel_ = max_vel2_;
        max_yawrate_ = max_yawrate2_;
        predict_time_ = predict_time2_;
    } else {
        max_vel_ = max_vel1_;
        max_yawrate_ = max_yawrate1_;
        predict_time_ = predict_time1_;
    }

    weight_heading_ = weight_heading1_; 
    weight_dist_    = weight_dist1_;
    // (YAMLでweight2などが設定されていれば、モードに応じて切り替える)
}

// Dynamic Windowを計算
void DWAPlanner::calc_dynamic_window()
{
    // ###### 車両モデルによるWindow ######
    double Vs[] = {min_vel_, max_vel_, -max_yawrate_, max_yawrate_};

    // ####### 運動モデルによるWindow #######
    double Vd[] = 
    {
        roomba_.velocity - max_accel_ * dt_,
        roomba_.velocity + max_accel_ * dt_,
        roomba_.yawrate - max_dyawrate_ * dt_,
        roomba_.yawrate + max_dyawrate_ * dt_
    };

    // ###### 最終的なDynamic Window ######
    dw_.min_vel = std::max(Vs[0], Vd[0]);
    dw_.max_vel = std::min(Vs[1], Vd[1]);
    dw_.min_yawrate = std::max(Vs[2], Vd[2]);
    dw_.max_yawrate = std::min(Vs[3], Vd[3]);
}

// 指定された予測時間までロボットの状態を更新し，予測軌跡を生成
std::vector<State> DWAPlanner::calc_traj(const double velocity, const double yawrate)
{
    std::vector<State> traj;
    State temp_state = {0.0, 0.0, 0.0, velocity, yawrate}; // ロボット座標系なので(0,0,0)スタート
    for (double t = 0; t < predict_time_; t += dt_) {
        move(temp_state, velocity, yawrate);
        traj.push_back(temp_state);
    }
    return traj;
}

// 予測軌跡作成時における仮想ロボットを移動
void DWAPlanner::move(State& state, const double velocity, const double yawrate)
{
    state.yaw += yawrate * dt_;
    state.yaw = normalize_angle(state.yaw);
    state.x += velocity * std::cos(state.yaw) * dt_;
    state.y += velocity * std::sin(state.yaw) * dt_;
}

// angleを適切な角度(-M_PI ~ M_PI)の範囲にして返す
double DWAPlanner::normalize_angle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// 評価関数を計算
double DWAPlanner::calc_evaluation(const std::vector<State>& traj)
{
    const double heading_score  = weight_heading_ * calc_heading_eval(traj);
    const double distance_score = weight_dist_    * calc_dist_eval(traj);
    const double velocity_score = weight_vel_     * calc_vel_eval(traj);

    // 衝突判定（calc_dist_evalがマイナス値を返した場合）
    if (distance_score < 0) return -1e6;

    const double total_score = heading_score + distance_score + velocity_score;

    return total_score;
}

// headingの評価関数を計算
// 軌跡のゴール方向への向きやすさを評価する関数
double DWAPlanner::calc_heading_eval(const std::vector<State>& traj)
{
    double last_x = traj.back().x;
    double last_y = traj.back().y;
    double last_yaw = traj.back().yaw;

    double error_angle = std::atan2(local_goal_.point.y - last_y, local_goal_.point.x - last_x);
    double score = M_PI - std::abs(normalize_angle(error_angle - last_yaw));
    return score; // ゴールを向いているほど高い
}

// distの評価関数を計算
// 軌跡の障害物回避性能を評価する関数
double DWAPlanner::calc_dist_eval(const std::vector<State>& traj)
{
    double min_dist = 1e6;
    for (const auto& step : traj) {
        for (const auto& obs : obs_poses_.poses) {
            double d = std::hypot(step.x - obs.position.x, step.y - obs.position.y);
            if (d < roomba_radius_) return -1e6; // 衝突
            if (d < min_dist) min_dist = d;
        }
    }
    return min_dist;
}

// velocityの評価関数を計算
// 軌跡の速度評価を計算する関数
double DWAPlanner::calc_vel_eval(const std::vector<State>& traj)
{
    return traj.back().velocity;
}

// 軌跡を可視化するための関数
void DWAPlanner::visualize_traj(const std::vector<State>& traj, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_path, rclcpp::Time now)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = robot_frame_;
    path.header.stamp = now;
    for (const auto& s : traj) {
        geometry_msgs::msg::PoseStamped ps;
        ps.pose.position.x = s.x;
        ps.pose.position.y = s.y;
        path.poses.push_back(ps);
    }
    pub_local_path->publish(path);
}