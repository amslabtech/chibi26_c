// /home/amsl/ros2_ws/src/chibi26_c/localizer/src/localizer.cpp
#include "localizer/localizer.hpp"

// デフォルトコンストラクタ
Localizer::Localizer() : Node("c_localizer"), engine_(seed_gen_())
{ 
    // パラメータの宣言
    this->declare_parameter("hz", 10);
    this->declare_parameter("particle_num", 600);
    this->declare_parameter("max_particle_num", 1000);
    this->declare_parameter("min_particle_num", 100);
    this->declare_parameter("move_dist_th", 0.15);
    this->declare_parameter("init_x", 0.0);
    this->declare_parameter("init_y", 0.0);
    this->declare_parameter("init_yaw", 0.0);
    this->declare_parameter("init_x_dev", 0.5);
    this->declare_parameter("init_y_dev", 0.5);
    this->declare_parameter("init_yaw_dev", 0.2);
    this->declare_parameter("alpha_th", 0.0035);
    this->declare_parameter("reset_count_limit", 5);
    this->declare_parameter("expansion_x_dev", 0.05);
    this->declare_parameter("expansion_y_dev", 0.05);
    this->declare_parameter("expansion_yaw_dev", 0.01);
    this->declare_parameter("laser_step", 10);
    this->declare_parameter("sensor_noise_ratio", 0.06);
    this->declare_parameter("ignore_angle_range_list", std::vector<double>{});
    this->declare_parameter("flag_init_noise", true);
    this->declare_parameter("flag_broadcast", true);
    this->declare_parameter("flag_reverse", false);
    this->declare_parameter("is_visible", true);
    this->declare_parameter("ff", 0.17);
    this->declare_parameter("fr", 0.0005);
    this->declare_parameter("rf", 0.13);
    this->declare_parameter("rr", 0.2);

    // パラメータの取得
    this->get_parameter("hz", hz_);
    this->get_parameter("particle_num", particle_num_);
    this->get_parameter("max_particle_num", max_particle_num_);
    this->get_parameter("min_particle_num", min_particle_num_);
    this->get_parameter("move_dist_th", move_dist_th_);
    this->get_parameter("init_x", init_x_);
    this->get_parameter("init_y", init_y_);
    this->get_parameter("init_yaw", init_yaw_);
    this->get_parameter("init_x_dev", init_x_dev_);
    this->get_parameter("init_y_dev", init_y_dev_);
    this->get_parameter("init_yaw_dev", init_yaw_dev_);
    this->get_parameter("alpha_th", alpha_th_);
    this->get_parameter("reset_count_limit", reset_count_limit_);
    this->get_parameter("expansion_x_dev", expansion_x_dev_);
    this->get_parameter("expansion_y_dev", expansion_y_dev_);
    this->get_parameter("expansion_yaw_dev", expansion_yaw_dev_);
    this->get_parameter("laser_step", laser_step_);
    this->get_parameter("sensor_noise_ratio", sensor_noise_ratio_);
    this->get_parameter("ignore_angle_range_list", ignore_angle_range_list_);
    this->get_parameter("flag_init_noise", flag_init_noise_);
    this->get_parameter("flag_broadcast", flag_broadcast_);
    this->get_parameter("flag_reverse", flag_reverse_);
    this->get_parameter("is_visible", is_visible_);
    this->get_parameter("ff", ff_);
    this->get_parameter("fr", fr_);
    this->get_parameter("rf", rf_);
    this->get_parameter("rr", rr_);

    // Subscriberの設定
    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
        std::bind(&Localizer::map_callback, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&Localizer::odom_callback, this, std::placeholders::_1));
    sub_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Localizer::laser_callback, this, std::placeholders::_1));

    // Publisherの設定
    pub_estimated_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("estimated_pose", 10);
    pub_particle_cloud_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particle_cloud", 10);

    // frame idの設定
    estimated_pose_msg_.header.frame_id = "map";
    particle_cloud_msg_.header.frame_id = "map";

    // パーティクルクラウドのメモリの確保
    particles_.reserve(max_particle_num_);
    particle_cloud_msg_.poses.reserve(max_particle_num_);

    // odometryのモデルの初期化
    odom_model_ = OdomModel(ff_, fr_, rf_, rr_);
}

// mapのコールバック関数
void Localizer::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_ = *msg;
    flag_map_ = true;
}

// odometryのコールバック関数
void Localizer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    last_odom_ = *msg;
    
    // ★追加：初回受信時のみ、prev_odom_に現在の値を代入してジャンプを防ぐ
    if (is_first_odom_) {
        prev_odom_ = last_odom_;
        is_first_odom_ = false;
    }
    
    flag_odom_ = true;
}

// laserのコールバック関数
void Localizer::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    laser_ = *msg;
    flag_laser_ = true;
}

// hz_を返す関数
int Localizer::getOdomFreq()
{
    return hz_;
}

// ロボットとパーティクルの推定位置の初期化
void Localizer::initialize()
{
    // 推定位置の初期化
    estimated_pose_.set(init_x_, init_y_, init_yaw_);
    
    particles_.clear();
    double start_yaw = flag_reverse_ ? normalize_angle(init_yaw_ + M_PI) : init_yaw_;

    // ★追加：初期化時にOdomの初回フラグをリセット（再初期化対策）
    is_first_odom_ = true;

    // 初期位置近傍にパーティクルを配置
    for(int i=0; i<particle_num_; i++)
    {
        // 以下の3行が消えてしまっていたため、復活させました
        double x = init_x_;
        double y = init_y_;
        double yaw = start_yaw;

        if(flag_init_noise_)
        {
            x += norm_rv(0.0, init_x_dev_);
            y += norm_rv(0.0, init_y_dev_);
            yaw += norm_rv(0.0, init_yaw_dev_);
        }
        particles_.emplace_back(x, y, normalize_angle(yaw), 1.0 / particle_num_);
    }

    // パーティクルの重みの初期化
    reset_weight();
}

// main文のループ内で実行される関数
void Localizer::process()
{
    if(flag_map_ && flag_odom_ && flag_laser_)
    {
        localize();
        broadcast_odom_state();
        publish_estimated_pose();
        publish_particles();
    }
}

// 適切な角度(-M_PI ~ M_PI)を返す
double Localizer::normalize_angle(double angle)
{
    while(angle > M_PI) angle -= 2.0 * M_PI;
    while(angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// ランダム変数生成関数（正規分布）
double Localizer::norm_rv(const double mean, const double stddev)
{
    std::normal_distribution<> dist(mean, stddev);
    return dist(engine_);
}

// パーティクルの重みの初期化
void Localizer::reset_weight()
{
    if(particles_.empty()) return;
    double w = 1.0 / (double)particles_.size();
    for(auto& p : particles_) p.set_weight(w);
}

// map座標系からみたodom座標系の位置と姿勢をtfでbroadcast
void Localizer::broadcast_odom_state()
{
    if(flag_broadcast_)
    {
        static std::shared_ptr<tf2_ros::TransformBroadcaster> odom_state_broadcaster;
        if(!odom_state_broadcaster)
            odom_state_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // map座標系からみたbase_link座標系の位置と姿勢 (estimated pose)
        double map_base_x = estimated_pose_.x();
        double map_base_y = estimated_pose_.y();
        double map_base_yaw = estimated_pose_.yaw();

        // odom座標系からみたbase_link座標系の位置と姿勢
        double odom_base_x = last_odom_.pose.pose.position.x;
        double odom_base_y = last_odom_.pose.pose.position.y;
        double odom_base_yaw = tf2::getYaw(last_odom_.pose.pose.orientation);

        // map座標系からみたodom座標系の位置と姿勢を計算（回転行列を利用）
        double map_odom_yaw = normalize_angle(map_base_yaw - odom_base_yaw);
        double map_odom_x = map_base_x - (odom_base_x * cos(map_odom_yaw) - odom_base_y * sin(map_odom_yaw));
        double map_odom_y = map_base_y - (odom_base_x * sin(map_odom_yaw) + odom_base_y * cos(map_odom_yaw));

        tf2::Quaternion map_to_odom_quat;
        map_to_odom_quat.setRPY(0, 0, map_odom_yaw);

        geometry_msgs::msg::TransformStamped odom_state;
        odom_state.header.stamp = this->now();
        odom_state.header.frame_id = map_.header.frame_id;
        odom_state.child_frame_id  = last_odom_.header.frame_id;

        odom_state.transform.translation.x = map_odom_x;
        odom_state.transform.translation.y = map_odom_y;
        odom_state.transform.translation.z = 0.0;
        odom_state.transform.rotation = tf2::toMsg(map_to_odom_quat);

        odom_state_broadcaster->sendTransform(odom_state);
    }
}

// 自己位置推定
void Localizer::localize()
{
    motion_update(); // パーティクルを動かす
    if(flag_move_)
    {
        observation_update(); // 引き続きコメントアウト
    }
    
    // ★ここに追加！観測がなくても、動いたパーティクルの中心を推定位置とする
    estimate_pose(); 
}

// 動作更新
void Localizer::motion_update()
{
    double dx = last_odom_.pose.pose.position.x - prev_odom_.pose.pose.position.x;
    double dy = last_odom_.pose.pose.position.y - prev_odom_.pose.pose.position.y;
    
    double current_yaw = tf2::getYaw(last_odom_.pose.pose.orientation);
    double prev_yaw = tf2::getYaw(prev_odom_.pose.pose.orientation);
    double dyaw = normalize_angle(current_yaw - prev_yaw);

    double length = std::sqrt(dx*dx + dy*dy);
    
    // 修正ポイント：移動中の方位を「移動前後の平均の向き」で補正する
    // これにより、旋回しながら移動するときの座標更新が安定します
    double avg_yaw = normalize_angle(prev_yaw + dyaw / 2.0);
    double direction = (length > 0.0) ? (std::atan2(dy, dx) - avg_yaw) : 0.0;
    
    double rotation = dyaw;

    odom_model_.set_dev(length, rotation);
    for(auto& p : particles_)
    {
        p.pose_.move(length, direction, rotation, odom_model_.get_fw_noise(), odom_model_.get_rot_noise());
    }

    prev_odom_ = last_odom_;

    // dist_sum の更新とリセット（前回の提案通り、判定を正確にするため）
    static double dist_sum = 0.0;
    static double angle_sum = 0.0; // 追加

    dist_sum += length;
    angle_sum += std::abs(rotation); // 回転量の絶対値を加算

    if(dist_sum > move_dist_th_ || angle_sum > 0.17)
    {
        flag_move_ = true;
        dist_sum = 0.0; 
        angle_sum = 0.0; // リセット
    }
    else
    {
        flag_move_ = false; 
    }

    // motion_update() 内
    if (length > 0.0 || std::abs(rotation) > 0.0) {
        RCLCPP_INFO(this->get_logger(), "Odom move: len=%.3f, rot=%.3f, total_dist=%.3f", length, rotation, dist_sum);
    }
}

// 観測更新
void Localizer::observation_update()
{
    // 各パーティクルの尤度計算
    for(auto& p : particles_)
    {
        double l = p.likelihood(map_, laser_, sensor_noise_ratio_, laser_step_, ignore_angle_range_list_);
        p.set_weight(p.weight() * l);
    }

    // 重みの正規化
    normalize_belief();

    // 周辺尤度の算出（リセット判定用）
    const double alpha = calc_marginal_likelihood();

    // 膨張リセット判定
    if(alpha < alpha_th_)
    {
        expansion_resetting(); // 尤度が低い間は常に膨張させて探し続ける
    }
    else
    {
        reset_counter = 0;
    }

    // リサンプリングと推定位置の決定
    resampling(alpha);
    estimate_pose();
}

// 周辺尤度の算出
double Localizer::calc_marginal_likelihood()
{
    double sum = 0.0;
    for(const auto& p : particles_) sum += p.weight();
    return sum / (double)particles_.size();
}

// 推定位置の決定（全パーティクルの加重平均）
void Localizer::estimate_pose()
{
    double mean_x = 0.0, mean_y = 0.0, mean_sin = 0.0, mean_cos = 0.0;
    for(const auto& p : particles_)
    {
        mean_x += p.pose_.x() * p.weight();
        mean_y += p.pose_.y() * p.weight();
        mean_sin += std::sin(p.pose_.yaw()) * p.weight();
        mean_cos += std::cos(p.pose_.yaw()) * p.weight();
    }
    estimated_pose_.set(mean_x, mean_y, std::atan2(mean_sin, mean_cos));
}

// 重みの正規化
void Localizer::normalize_belief()
{
    double sum = 0.0;
    for(const auto& p : particles_) sum += p.weight();
    if(sum > 0.0)
    {
        for(auto& p : particles_) p.set_weight(p.weight() / sum);
    }
    else
    {
        reset_weight();
    }
}

// 膨張リセット
void Localizer::expansion_resetting()
{
    for(auto& p : particles_)
    {
        p.pose_.set(
            p.pose_.x() + norm_rv(0.0, expansion_x_dev_),
            p.pose_.y() + norm_rv(0.0, expansion_y_dev_),
            normalize_angle(p.pose_.yaw() + norm_rv(0.0, expansion_yaw_dev_))
        );
    }
}

// リサンプリング（系統サンプリング）
void Localizer::resampling(const double alpha)
{
    std::vector<double> accum;
    accum.reserve(particles_.size());
    double sum = 0.0;
    for(const auto& p : particles_)
    {
        sum += p.weight();
        accum.push_back(sum);
    }

    const std::vector<Particle> old(particles_);
    int old_size = old.size();

    // 尤度alphaが低い場合にパーティクル数を増やす等の動的変更も可能だが、
    // ここでは初期設定数(particle_num_)を維持する実装とする
    int next_size = particle_num_;
    
    particles_.clear();
    particles_.reserve(max_particle_num_);

    double step = sum / next_size;
    std::uniform_real_distribution<> dist(0.0, step);
    double r = dist(engine_);

    int idx = 0;
    for(int i=0; i<next_size; i++)
    {
        while(r > accum[idx] && idx < old_size - 1) idx++;
        particles_.push_back(old[idx]);
        r += step;
    }

    reset_weight();
}

// 推定位置のパブリッシュ
void Localizer::publish_estimated_pose()
{
    estimated_pose_msg_.header.stamp = this->now();
    estimated_pose_msg_.pose.position.x = estimated_pose_.x();
    estimated_pose_msg_.pose.position.y = estimated_pose_.y();
    
    tf2::Quaternion q;
    q.setRPY(0, 0, estimated_pose_.yaw());
    estimated_pose_msg_.pose.orientation = tf2::toMsg(q);

    pub_estimated_pose_->publish(estimated_pose_msg_);
}

// パーティクルクラウドのパブリッシュ
void Localizer::publish_particles()
{
    if(is_visible_)
    {
        particle_cloud_msg_.header.stamp = this->now();
        particle_cloud_msg_.poses.clear();
        for(const auto& p : particles_)
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = p.pose_.x();
            pose.position.y = p.pose_.y();
            tf2::Quaternion q;
            q.setRPY(0, 0, p.pose_.yaw());
            pose.orientation = tf2::toMsg(q);
            particle_cloud_msg_.poses.push_back(pose);
        }
        pub_particle_cloud_->publish(particle_cloud_msg_);
    }
}