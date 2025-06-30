#pragma once


#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace turtlebot3_autonav
{

class ExplorerNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  ExplorerNode();

private:
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  // Action client for navigation
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;

  // Latest map and pose
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_;

  // Callback functions
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Frontier clustering and goal sending
  struct Cluster {
    std::vector<int> indices;
    geometry_msgs::msg::Point centroid;
  };

  std::vector<int> detectFrontierCells(const nav_msgs::msg::OccupancyGrid & map);
  std::vector<Cluster> clusterFrontiers(const std::vector<int> & frontier_indices, const nav_msgs::msg::OccupancyGrid & map);
  geometry_msgs::msg::Point computeCentroid(const std::vector<int> & indices, const nav_msgs::msg::OccupancyGrid & map);
  void sendGoalToCluster(const Cluster & cluster);

  // Blacklist for unreachable clusters
  std::vector<geometry_msgs::msg::Point> blacklist_;
  void addToBlacklist(const geometry_msgs::msg::Point & centroid);
  bool isBlacklisted(const geometry_msgs::msg::Point & centroid);

  // Helper functions
  bool isFrontierCell(int idx, const nav_msgs::msg::OccupancyGrid & map);
  geometry_msgs::msg::Point indexToWorld(int idx, const nav_msgs::msg::OccupancyGrid & map);

  // Timer for periodic exploration
  rclcpp::TimerBase::SharedPtr explore_timer_;
  void explore();

  // State
  bool navigating_;
};

}  // namespace turtlebot3_autonav
