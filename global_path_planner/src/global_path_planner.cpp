#include "global_path_planner/global_path_planner.hpp"
#include <limits>

using namespace std::chrono_literals;

// デフォルトコンストラクタ
// パラメータの宣言と取得
Astar::Astar() : Node("team_global_path_planner_node"), clock_(RCL_ROS_TIME)
{
    // ###### パラメータの宣言 ######
    this->declare_parameter<double>("sleep_time");
    this->declare_parameter<double>("margin");
    this->declare_parameter<bool>("test_show");
    this->declare_parameter<std::vector<double>>("way_points_x");
    this->declare_parameter<std::vector<double>>("way_points_y");

    // ###### パラメータの取得 ######
    this->get_parameter("sleep_time", sleep_time_);
    this->get_parameter("margin", margin_);
    this->get_parameter("test_show", test_show_);
    this->get_parameter("way_points_x", way_points_x_);
    this->get_parameter("way_points_y", way_points_y_);

    // ###### global_path_とcurrent_node_のframe_id設定 ######
    global_path_.header.frame_id = "map";
    current_node_.header.frame_id = "map";
    new_map_.header.frame_id = "map";

    // dataサイズの確保
    global_path_.poses.reserve(2000);

    // ####### Subscriber #######
    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&Astar::map_callback, this, std::placeholders::_1)
    );

    // ###### Publisher ######
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>( // 計算した経路を発行
        "global_path", 10
    );

    pub_node_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>( // 可視化用ノード位置を発行
        "node_point", 10
    );

    pub_current_path_ = this->create_publisher<nav_msgs::msg::Path>( // 現在のパスを発行
        "current_path", 10
    );

    pub_new_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>( // 拡張後のマップを発行
        "new_map",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()
    );

}

// mapのコールバック関数
// msgを受け取り，map_に代入，その情報をそれぞれ取得
// process()を実行
void Astar::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)  //マップの読み込み
{
    // 1.msgを受け取り，代入
    map_ = *msg;

    // 2.情報の取得
    height_ = map_.info.height;  // マップの高さ
    width_ = map_.info.width;   // マップの幅
    resolution_ = map_.info.resolution; // マップ解像度
    origin_x_ = map_.info.origin.position.x;  // マップ原点
    origin_y_ = map_.info.origin.position.y;  // マップ原点

    // 3.map取得完了フラグ
    map_checker_ = true;

    // 4.process()を実行
    process();
}

// マップ全体の障害物を拡張処理（new_map_をpublishする）
void Astar::obs_expander()
{
    // new_map_の中身をmap_で初期化
    new_map_ = map_;

    // map_で障害物のとき障害物を拡張
    for(int i  = 0; i  < width_ * height_; i++){
        if(map_.data[i] == 100){
            obs_expand(i);
        }
    }

    pub_new_map_->publish(new_map_);
}

// 指定されたインデックスの障害物を拡張（margin_length分）
void Astar::obs_expand(const int index)
{
    // indexをx,y座標に変換
    int x = index % width_;
    int y = index / width_;

    // margin_をメートル単位からセル単位に変換
    const int cell_margin_ = std::ceil(margin_ / resolution_);
    for(int i = x - cell_margin_; i <= x + cell_margin_; i++){
        for(int j = y - cell_margin_; j <= y + cell_margin_; j++){
            // マップの境界をチェック
            if(i < 0 || j < 0 || i >= width_ || j >= height_ || new_map_.data[(j * width_ + i)] == 100){
                continue;
            }
            // 円形にマップを拡張
            int distance2 = ((i - x)*(i - x) + (j - y)*(j - y));
            if (distance2 <= (cell_margin_*cell_margin_)){
                new_map_.data[(j * width_ + i)] = 100;
            }
        }
    }
}

// ヒューリスティック関数の計算
double Astar::make_heuristic(const Node_ node)
{
    double dx = static_cast<double>(goal_node_.x - node.x);
    double dy = static_cast<double>(goal_node_.y - node.y);

    return std::sqrt(dx * dx + dy * dy);    
}

// スタートとゴールの取得（mからグリッド単位への変換も行う）
Node_ Astar::set_way_point(int phase)
{
    Node_ way_point;
        int tx = static_cast<int>((way_points_x_[phase] - origin_x_) / resolution_);
        int ty = static_cast<int>((way_points_y_[phase] - origin_y_) / resolution_);
        if(tx < 0)tx = 0; if(tx >= width_)tx = width_ - 1;
        if(ty < 0)ty = 0; if(ty >= height_)ty = height_ - 1;
        way_point.x = tx;
        way_point.y = ty;
        way_point.parent_x = -1;
        way_point.parent_y = -1;
        way_point.f = 0.0;
    return way_point;
}

// ノードをたどり，waypoint間のパスを作成．その後グローバルパスに追加
// 参考：push_back(...) https://cpprefjp.github.io/reference/vector/vector/push_back.html
void Astar::create_path(Node_ node)
{
    nav_msgs::msg::Path partial_path;
    
    // 1.親ノードをたどり，ゴールからスタートまでをリストアップ
    while(node.parent_x != -1 && node.parent_y != -1){
        partial_path.poses.push_back(node_to_pose(node));

        // 親の座標を頼りにCloseリストから親ノードを探し出す
        int parent_key = node.parent_y * width_ + node.parent_x;
        auto it = close_map_.find(parent_key);
        if(it == close_map_.end()) break;
        node = it->second;
    }
    
    // 2.最後にスタートノードを追加
    partial_path.poses.push_back(node_to_pose(node));
    // 3.順序の反転
    std::reverse(partial_path.poses.begin(), partial_path.poses.end());

    // 4.グローバルパスへ追加
    global_path_.poses.insert(global_path_.poses.end(), partial_path.poses.begin(), partial_path.poses.end());

}

// ノード座標（グリッド）をgeometry_msgs::msg::PoseStamped（m単位のワールド座標系）に変換
geometry_msgs::msg::PoseStamped Astar::node_to_pose(const Node_ node)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = clock_.now();

    pose.pose.position.x = origin_x_ + (node.x * resolution_) + (resolution_ / 2.0);
    pose.pose.position.y = origin_y_ + (node.y * resolution_) + (resolution_ / 2.0);
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    return pose;
}


// openリスト内で最もf値が小さいノードを取得する関数
Node_ Astar::select_min_f()
{
    Node_ best;
    double min_f = std::numeric_limits<double>::max();
    for(const auto& [key, node] : open_map_){
        if(node.f < min_f){
            min_f = node.f;
            best  = node;
        }
    }
    return best;
}

// スタートノードの場合，trueを返す
bool Astar::check_start(const Node_ node)
{
    return check_same_node(node, start_node_);
}

// ゴールノードの場合，trueを返す
bool Astar::check_goal(const Node_ node)
{
    return check_same_node(node, goal_node_);
}

// 2つが同じノードである場合，trueを返す
bool Astar::check_same_node(const Node_ n1, const Node_ n2)
{
    return (n1.x == n2.x && n1.y == n2.y);
}

// 指定したリストに指定のノードが含まれるか検索
//（含まれる場合はインデックス番号を返し，含まれない場合は-1を返す）
int Astar::check_list(const Node_ target_node, std::vector<Node_>& set)
{
    for(size_t i = 0; i < set.size(); i++){
        if(check_same_node(target_node, set[i])) return static_cast<int>(i);
    }
    return -1;
}

// open_map_から削除して close_map_ に移動する関数
void Astar::swap_node(const Node_ node)
{
    int key = node.y * width_ + node.x;
    open_map_.erase(key);
    close_map_[key] = node;
}

// 指定のノードが障害物である場合，trueを返す
bool Astar::check_obs(const Node_ node)
{
    //　マップの範囲内かのチェック(範囲外は障害物として扱う)
    if(node.x < 0 || node.x >= width_ || node.y < 0 || node.y >= height_){
        return true;
    }
    
    // 障害物かどうかの判定
    int index = node.y * width_ + node.x;
    if(new_map_.data[index] >= 50 || new_map_.data[index] == -1){
        return true;
    }

    return false;
}

// 隣接ノードを基にOpenリスト・Closeリストを更新
// 隣接ノードを計算し，障害物を避けつつ，リスト内のノードを適切に追加・更新
// 複数の変数への代入はstd::tie(...)を使用すると便利 https://minus9d.hatenablog.com/entry/2015/05/24/133253
void Astar::update_list(const Node_ node)
{
    // 隣接ノードを宣言
    std::vector<Node_> neighbor_nodes;
    create_neighbor_nodes(node, neighbor_nodes);

    for(auto& neighbor : neighbor_nodes){
        auto[list_id, index] = search_node(neighbor);
        // 1.すでに探索済の場合はスキップ
        if(list_id == 2) continue;

        // 2.コストの確定
        double g = neighbor.g;
        double h = make_heuristic(neighbor);
        neighbor.f = g + h;

        // 3.リストにない新規ノードの場合
        if(list_id == -1){
            open_map_[neighbor.y * width_ + neighbor.x] = neighbor;
        }

        // 4.すでにOpenリストにある場合
        else if(list_id == 1){
            if(neighbor.f < open_map_[index].f){
                open_map_[neighbor.y * width_ + neighbor.x] = neighbor;
            }
        }
    }
}

// 現在のノードを基に隣接ノードを作成
void Astar::create_neighbor_nodes(const Node_ node, std::vector<Node_>&  neighbor_nodes)
{
    // 動作モデルの作成
    std::vector<Motion_> motion_list;
    get_motion(motion_list);

    for(const auto& m : motion_list){
        Node_ neighbor = get_neighbor_node(node, m);
        if(!check_obs(neighbor)){
            neighbor_nodes.push_back(neighbor);
        }

    }
}

// 動作モデルを作成（前後左右，斜めの8方向）
void Astar::get_motion(std::vector<Motion_>& list)
{
    list.push_back(motion(  1,  0, 1)); // 前
    list.push_back(motion( -1,  0, 1)); // 後
    list.push_back(motion(  0,  1, 1)); // 左
    list.push_back(motion(  0, -1, 1)); // 右
    list.push_back(motion(  1,  1, 1.4142)); // 左前
    list.push_back(motion(  1, -1, 1.4142)); // 右前
    list.push_back(motion( -1,  1, 1.4142)); // 左後
    list.push_back(motion( -1, -1, 1.4142)); // 右後
}

// 与えられたdx, dy, costを基にモーション（移動）を作成
// 隣接したグリッドに移動しない場合はエラーメッセージを出力して終了
Motion_ Astar::motion(const int dx,const int dy,const int cost)
{
    if(std::abs(dx) > 1 || std::abs(dy) > 1 || (dx == 0 && dy == 0)){
        RCLCPP_ERROR(get_logger(), "Invalid motion: dx = %d, dy = %d", dx, dy);
        exit(1);
    }

    Motion_ m;
    m.dx = dx;
    m.dy = dy;
    m.cost = cost;
    return m;
}

// 現在のノードと与えられたモーションを基に隣接ノードを計算し，その隣接ノードのg値と親ノードを更新して返す
Node_ Astar::get_neighbor_node(const Node_ node, const Motion_ motion)
{
    Node_ neighbor_node;

    // 座標の更新
    neighbor_node.x = node.x + motion.dx;
    neighbor_node.y = node.y + motion.dy;

    // 親ノードの記録
    neighbor_node.parent_x = node.x;
    neighbor_node.parent_y = node.y;

    // g値の計算
    neighbor_node.g = node.g + motion.cost;

    return neighbor_node;
}

// 指定されたノードがOpenリストまたはCloseリストに含まれているかを調べ，結果をインデックスとともに返す
// 1はOpenリスト，2はCloseリストにノードが含まれていることを示す
// -1はどちらのリストにもノードが含まれていないことを示す
std::tuple<int, int> Astar::search_node(const Node_ node)
{
    int key = node.y * width_ + node.x;

    // 1.Openmapを検索
    if(open_map_.count(key))  return {1, key};
    
    // 2.Closemapを検索
    if(close_map_.count(key)) return {2, key};
    
    // 3.どちらにも含まれていない
    return {-1, -1};
}


// 親ノードかの確認
bool Astar::check_parent(const int index, const Node_ node)
{
    return (index == node.parent_y * width_ + node.parent_x);
}



// 指定リスト内のノード検索
// 同一のノードが見つかればそのインデックスを返す
// 見つからなければ-1を返す
int Astar::search_node_from_list(const Node_ node, std::vector<Node_>& list)
{
    for(size_t i = 0; i < list.size(); i++){
        if(check_same_node(node, list[i])) return static_cast<int>(i);
    }
    return -1;
}


// ［デバック用］指定されたノードの位置をRvizに表示
// test_show_がtrueの場合，ノードの座標をワールド座標系に変換し
// そのノードの情報をRvizにパブリッシュ
void Astar::show_node_point(const Node_ node)
{
    // test_show_ パラメータが false の場合は何もしない（負荷軽減）
    if (!test_show_) return;

    geometry_msgs::msg::PointStamped point;
    point.header.frame_id = "map";
    point.header.stamp = clock_.now();

    // グリッド座標をワールド座標(m)に変換
    point.point.x = origin_x_ + (node.x * resolution_) + (resolution_ / 2.0);
    point.point.y = origin_y_ + (node.y * resolution_) + (resolution_ / 2.0);
    point.point.z = 0.5; // 地面より少し浮かせて表示

    pub_node_point_->publish(point);
}

// ［デバック用］指定されたパスをRvizに表示
// test_show_がtrueの場合，パスのフレームIDを"map"に設定し
// パス情報をRvizにパブリッシュ
void Astar::show_path(nav_msgs::msg::Path& current_path)
{
    if (!test_show_) return;

    current_path.header.frame_id = "map";
    current_path.header.stamp = clock_.now();

    pub_current_path_->publish(current_path);
}

// 実行時間を表示（スタート時間beginを予め設定する）
void Astar::show_exe_time()
{
    RCLCPP_INFO_STREAM(get_logger(), "Duration = " << std::fixed << std::setprecision(2) << clock_.now().seconds() - begin_.seconds() << "s");
}



// 経路計画を行う関数
// 目的地までの経路をA*アルゴリズムを用いて計算し，グローバルパスを作成
// 各フェーズ（ウェイポイント間）について，OpenリストとCloseリストを操作しながら経路を探索
void Astar::planning()
{
    begin_ = clock_.now();
    const int total_phase = way_points_x_.size();

    start_node_ = set_way_point(0);

    // ###### ウェイポイント間の経路探索 ######
    for(int phase = 1; phase < total_phase; phase++){
        // 1.各フェーズの初期化
        open_map_.clear();
        close_map_.clear();
        goal_node_ = set_way_point(phase);

        open_map_[start_node_.y * width_ + start_node_.x] = start_node_;

        // 2.A* メインループ
        while(!open_map_.empty()){
            Node_ current = select_min_f();

            // ###### デバッグポイント A: 探索最前線の表示 ######
            // いま「検討中」の場所をRVizに表示する
            show_node_point(current);

            if(check_goal(current)){
                create_path(current);

                // ###### デバッグポイント B: フェーズ完了後のパス表示 ######
                // その区間のパスが完成した瞬間に表示を更新する
                pub_path_->publish(global_path_);

                start_node_ = current;
                break;
            }
            swap_node(current);
            update_list(current);
        }
    }
    pub_path_->publish(global_path_);

    show_exe_time();
    RCLCPP_INFO_STREAM(get_logger(), "COMPLITE ASTAR PROGLAM");
    //exit(0);
}


// map_callback()関数で実行する関数
// A*アルゴリズムを実行する前にマップのロードをチェック
// マップが読み込まれた後に壁判定と経路計画を実行
void Astar::process()
{
    RCLCPP_INFO_STREAM(get_logger(), "process is starting...");

    if(planning_done_) return;
    
    if(!map_checker_){
    RCLCPP_INFO_STREAM(get_logger(), "NOW LOADING...");
    }else
    {
        RCLCPP_INFO_STREAM(get_logger(), "NOW LOADED MAP");
        obs_expander(); // 壁の拡張
        planning(); // グローバルパスの作成
        planning_done_ = true;
    }

}
