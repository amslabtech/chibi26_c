#ifndef LOCAL_GOAL_CREATOR_HPP
#define LOCAL_GOAL_CREATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

class LocalGoalCreator : public rclcpp::Node
{
public:
    LocalGoalCreator();
    void process();
    int getOdomFreq();

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publishGoal();
    double getDistance();

    int hz_;
    int index_step_;
    int goal_index_;
    double taeget_distance_;
    bool is_path_ = false;

    //　Subscriber
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr local_goal_pub_;


    nav_msgs::msg::Path path_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::PoseStamped pose_;
};

#endif // LOCAL_GOAL_CREATOR_H