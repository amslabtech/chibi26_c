#ifndef obstacle_detector_HPP
#define obstacle_detector_HPP

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <rclcpp/rclcpp.hpp>
#include <functional>  // bind & placeholders用
#include <memory>      // SharedPtr用
#include <optional>    // has_value()用
#include <string>

class ObstacleDetector : public rclcpp::Node
{
  public:
    ObstacleDetector();
  
  private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void process();
    bool scan_obstacle();
    bool is_ignore_scan(double range) const;

    int hz_;
    int laser_step_;
    std::string robot_frame_;
    double ignore_dist_;
    double ignore_pillar_;

    std::optional<sensor_msgs::msg::LaserScan> scan_;  // optional型で定義することによりscanをsubできたかの判定も同時に行う
    //カギカッコ < > の中には「箱に入れる中身の型」を指定する。
    //今回はLiDARのデータを入れたいので、sensor_msgs::msg::LaserScan という型を指定。
    //そして、その「空かもしれない箱」全体に scan_ という名前を付けた

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;  // scan
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacles_pub_; //障害物位置の送信
    rclcpp::TimerBase::SharedPtr timer_;  //タイマー宣言
};

#endif  // c_obstacle_detector_hpp