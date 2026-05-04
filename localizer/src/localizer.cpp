// /home/amsl/ros2_ws/src/chibi26_c/localizer/src/localizer.cpp
#include "localizer/localizer.hpp"

// デフォルトコンストラクタ
Localizer::Localizer() : Node("c_localizer"), engine_(seed_gen_())
{
    // ★修正：declare_parameter の戻り値をそのまま代入することで、
    //         declare→get の2行を1行に短縮できる（YAMLで上書きされた値も正しく入る）
    hz_                       = this->declare_parameter<int>("hz", 10);
    particle_num_             = this->declare_parameter<int>("particle_num", 550);
    max_particle_num_         = this->declare_parameter<int>("max_particle_num", 1000);
    min_particle_num_         = this->declare_parameter<int>("min_particle_num", 100);
    move_dist_th_             = this->declare_parameter<double>("move_dist_th", 0.195);
    init_x_                   = this->declare_parameter<double>("init_x", 0.0);
    init_y_                   = this->declare_parameter<double>("init_y", 0.0);
    init_yaw_                 = this->declare_parameter<double>("init_yaw", 0.0);
    init_x_dev_               = this->declare_parameter<double>("init_x_dev", 0.5);
    init_y_dev_               = this->declare_parameter<double>("init_y_dev", 0.5);
    init_yaw_dev_             = this->declare_parameter<double>("init_yaw_dev", 0.2);
    alpha_th_                 = this->declare_parameter<double>("alpha_th", 0.00022);
    reset_count_limit_        = this->declare_parameter<int>("reset_count_limit", 5);
    expansion_x_dev_          = this->declare_parameter<double>("expansion_x_dev", 0.05);
    expansion_y_dev_          = this->declare_parameter<double>("expansion_y_dev", 0.05);
    expansion_yaw_dev_        = this->declare_parameter<double>("expansion_yaw_dev", 0.01);
    laser_step_               = this->declare_parameter<int>("laser_step", 10);
    sensor_noise_ratio_       = this->declare_parameter<double>("sensor_noise_ratio", 0.04);
    ignore_angle_range_list_  = this->declare_parameter<std::vector<double>>("ignore_angle_range_list", std::vector<double>{});
    flag_init_noise_          = this->declare_parameter<bool>("flag_init_noise", true);
    flag_broadcast_           = this->declare_parameter<bool>("flag_broadcast", true);
    flag_reverse_             = this->declare_parameter<bool>("flag_reverse", false);
    is_visible_               = this->declare_parameter<bool>("is_visible", true);
    ff_                       = this->declare_parameter<double>("ff", 0.17);
    fr_                       = this->declare_parameter<double>("fr", 0.0005);
    rf_                       = this->declare_parameter<double>("rf", 0.10);
    rr_                       = this->declare_parameter<double>("rr", 0.2);

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

    // ★修正：tfブロードキャスターをここで一度だけ生成し、ノードの寿命と同じにする。
    //         関数内staticにしていると、並行処理時の初期化競合や、
    //         ノード再生成時に古いインスタンスが残って二重通信になるリスクがあった。
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // frame idの設定
    estimated_pose_msg_.header.frame_id = "map";
    particle_cloud_msg_.header.frame_id = "map";

    // パーティクルクラウドのメモリの確保
    particles_.reserve(max_particle_num_);
    particle_cloud_msg_.poses.reserve(max_particle_num_);

    // odometryのモデルの初期化
    odom_model_ = OdomModel(ff_, fr_, rf_, rr_);
    initialize();
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

    // 初回受信時のみ、prev_odom_に現在の値を代入してジャンプを防ぐ
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

    // 初期化時にOdomの初回フラグをリセット（再初期化対策）
    is_first_odom_ = true;

    // 初期位置近傍にパーティクルを配置
    for(int i=0; i<particle_num_; i++)
    {
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
    if(!flag_broadcast_) return;

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

    // ★修正：static のローカル変数ではなく、メンバの tf_broadcaster_ を使う。
    tf_broadcaster_->sendTransform(odom_state);
}

// 自己位置推定
void Localizer::localize()
{
    motion_update(); // パーティクルを動かす

    // ★再修正：flag_move_ ゲートを復活させる。
    //
    // 【なぜ毎周期だとダメか：サンプル枯渇 (sample impoverishment)】
    //   1周期の移動量は ~2cm 程度で、motion_update で注入される
    //   オドメトリノイズの標準偏差も ~2cm 程度しかない。
    //   そこで 10Hz で observation_update + resampling を回すと、
    //   毎周期パーティクルが「今この瞬間最も尤もらしい1点」へ集約され、
    //   多様性が失われる(=点に潰れる)。
    //   一度潰れると、実ロボットが動いてもパーティクル全体が
    //   同じ場所に貼り付いてしまい、観測尤度の勾配だけでは追従できない。
    //   結果：「その場にとどまり続けて自己位置がじわじわずれていく」現象。
    //
    // 【flag_move_ ゲートの効果】
    //   move_dist_th_(=0.2m) ぶんの動作量が蓄積されてから観測更新することで、
    //   motion noise が ~7cm 程度まで蓄積され、
    //   観測で絞ってもパーティクルクラウドの広がりが残る。
    //
    // 【先輩の指摘「停止中も観測更新で補正」について】
    //   理屈としては正しいが、それを成立させるには
    //   ESS (Effective Sample Size) ベースのリサンプリング制御や
    //   動作ノイズの再調整、KLD-sampling などが必要。
    //   現状のシンプルな実装では本ゲート方式の方が安定する。
    if(flag_move_)
    {
        observation_update();
    }

    // 動いていなくても、現在のパーティクル分布の中心を推定位置として更新する
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

    // 移動中の方位は「移動前後の平均の向き」で補正する
    // これにより、旋回しながら移動するときの座標更新が安定する
    double avg_yaw = normalize_angle(prev_yaw + dyaw / 2.0);
    double direction = (length > 0.0) ? (std::atan2(dy, dx) - avg_yaw) : 0.0;

    double rotation = dyaw;

    odom_model_.set_dev(length, rotation);
    for(auto& p : particles_)
    {
        p.pose_.move(length, direction, rotation, odom_model_.get_fw_noise(), odom_model_.get_rot_noise());
    }

    prev_odom_ = last_odom_;

    // flag_move_ は「動いた量がある程度溜まったかどうか」の指標として残す
    // （リサンプリングを毎周期かけたくない場合のフラグなどに使える）
    static double dist_sum = 0.0;
    static double angle_sum = 0.0;

    dist_sum += length;
    angle_sum += std::abs(rotation);

    if(dist_sum > move_dist_th_ || angle_sum > 0.17)
    {
        flag_move_ = true;
        dist_sum = 0.0;
        angle_sum = 0.0;
    }
    else
    {
        flag_move_ = false;
    }

    if (length > 0.0 || std::abs(rotation) > 0.0) {
        RCLCPP_INFO(this->get_logger(), "Odom move: len=%.3f, rot=%.3f, total_dist=%.3f", length, rotation, dist_sum);
    }
}

// 観測更新
void Localizer::observation_update()
{
    // 各パーティクルの尤度計算 → 重みを更新
    for(auto& p : particles_)
    {
        double l = p.likelihood(map_, laser_, sensor_noise_ratio_, laser_step_, ignore_angle_range_list_);
        p.set_weight(p.weight() * l);
    }

    // ★修正：alpha は「正規化前」の重み合計から平均を取る。
    //         normalize_belief() の後に呼ぶと、合計が常に1.0になり、
    //         alpha = 1/N に固定されて観測の良し悪しを表せなくなる。
    const double alpha = calc_marginal_likelihood();

    // ★デバッグ用：alpha 実測値と alpha_th_ を表示。
    //   ロボットがマップに合っている状態で alpha がどれくらいの値を取るかを見て、
    //   yaml 側の alpha_th_ をその少し下に再設定する。
    RCLCPP_INFO(this->get_logger(),
                "obs: alpha=%.6e, alpha_th=%.6e, reset_cnt=%d",
                alpha, alpha_th_, reset_counter);

    // alpha を計算したあとに正規化する
    normalize_belief();

    // 膨張リセット判定
    if(alpha < alpha_th_)
    {
        // 連続して膨張しすぎることだけは防ぐ（拡散のループに入るのを止める）
        if(reset_counter < reset_count_limit_)
        {
            expansion_resetting();
            reset_counter++;
        }
        // ★ここがポイント：
        //   - フィードバック「膨張リセット後は重みを初期化」は守る（reset_weight）
        //   - ただし、毎周期ここで止まると収束力がなくなるので、リサンプリングは続行する。
        //     リサンプリング後にも reset_weight が呼ばれるので、両者の「重み初期化」が
        //     重複してしまうのを避けるため、ここではリサンプリングに任せる構成にする。
        resampling(alpha);
    }
    else
    {
        reset_counter = 0;
        resampling(alpha);
    }
}

// 周辺尤度の算出（正規化前の重み合計の平均）
// 注：必ず normalize_belief() より「前」に呼ぶこと。
//     正規化後だと sum = 1.0 となり、戻り値が常に 1/N になってしまうため。
double Localizer::calc_marginal_likelihood()
{
    if(particles_.empty()) return 0.0;
    double sum = 0.0;
    for(const auto& p : particles_) sum += p.weight();
    return sum / static_cast<double>(particles_.size());
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
void Localizer::resampling(const double /*alpha*/)
{
    std::vector<double> accum;
    accum.reserve(particles_.size());
    double sum = 0.0;
    for(const auto& p : particles_)
    {
        sum += p.weight();
        accum.push_back(sum);
    }
    if(sum <= 0.0) { reset_weight(); return; }

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