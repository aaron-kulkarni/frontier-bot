#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class ExplorerNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  struct Cluster {
    std::vector<std::pair<int, int>> cells;
    double centroid_x;
    double centroid_y;
  };

  ExplorerNode()
  : Node("explorer_node")
  {
    RCLCPP_INFO(this->get_logger(), "Explorer node starting up...");

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10,
      std::bind(&ExplorerNode::map_callback, this, std::placeholders::_1));

    // Add debug log for pose subscription
    RCLCPP_INFO(this->get_logger(), "Subscribing to amcl_pose as geometry_msgs::msg::PoseWithCovarianceStamped");
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose", 10,
      std::bind(&ExplorerNode::pose_callback, this, std::placeholders::_1));

    nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    timer_ = this->create_wall_timer(
      2s, std::bind(&ExplorerNode::timer_callback, this));
  }

private:
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

  // Action client
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Latest map and pose
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  geometry_msgs::msg::Pose latest_pose_;

  // Whether we are currently navigating
  bool navigating_ = false;

  // Blacklist of unreachable cluster centroids
  std::vector<std::pair<double, double>> blacklist_;

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    latest_map_ = msg;
    RCLCPP_INFO(this->get_logger(), "Received map update (size: %ux%u)", msg->info.width, msg->info.height);
  }

  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    latest_pose_ = msg->pose.pose;
    RCLCPP_INFO(this->get_logger(), "Received pose update (x: %.2f, y: %.2f)", latest_pose_.position.x, latest_pose_.position.y);
  }

  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Timer callback triggered.");
    if (navigating_) {
      RCLCPP_INFO(this->get_logger(), "Already navigating to a goal. Will not send a new goal.");
      return;
    }
    if (!latest_map_) {
      RCLCPP_WARN(this->get_logger(), "No map received yet, cannot explore.");
      return;
    }

    // Find and cluster frontiers
    auto clusters = cluster_frontiers(*latest_map_);
    if (clusters.empty()) {
      RCLCPP_INFO(this->get_logger(), "No frontier clusters found.");
      return;
    }

    // Pick the closest non-blacklisted cluster centroid to current pose
    geometry_msgs::msg::Point goal_point;
    bool found = pick_closest_cluster_centroid(clusters, latest_pose_, goal_point);
    if (!found) {
      RCLCPP_INFO(this->get_logger(), "No non-blacklisted frontier clusters found.");
      return;
    }

    // Send goal
    send_goal(goal_point);
  }

  // --- Frontier clustering and blacklist logic ---

  std::vector<Cluster> cluster_frontiers(const nav_msgs::msg::OccupancyGrid & map)
  {
    std::vector<Cluster> clusters;
    const int width = map.info.width;
    const int height = map.info.height;
    const auto & data = map.data;
    std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));

    // Helper lambda to check if a cell is a frontier
    auto is_frontier = [&](int x, int y) {
      int idx = y * width + x;
      if (data[idx] != -1) return false;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) continue;
          int nx = x + dx, ny = y + dy;
          if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
          int nidx = ny * width + nx;
          if (data[nidx] == 0) return true;
        }
      }
      return false;
    };

    // BFS region growing for clustering
    for (int y = 1; y < height - 1; ++y) {
      for (int x = 1; x < width - 1; ++x) {
        if (visited[y][x] || !is_frontier(x, y)) continue;
        // Start new cluster
        Cluster cluster;
        std::vector<std::pair<int, int>> queue = {{x, y}};
        visited[y][x] = true;
        size_t qi = 0;
        while (qi < queue.size()) {
          int cx = queue[qi].first, cy = queue[qi].second;
          cluster.cells.push_back({cx, cy});
          // 4-connectivity
          for (int d = 0; d < 4; ++d) {
            int nx = cx + (d == 0) - (d == 1);
            int ny = cy + (d == 2) - (d == 3);
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
            if (!visited[ny][nx] && is_frontier(nx, ny)) {
              queue.push_back({nx, ny});
              visited[ny][nx] = true;
            }
          }
          ++qi;
        }
        // Compute centroid
        double sum_x = 0.0, sum_y = 0.0;
        for (const auto & cell : cluster.cells) {
          sum_x += map.info.origin.position.x + (cell.first + 0.5) * map.info.resolution;
          sum_y += map.info.origin.position.y + (cell.second + 0.5) * map.info.resolution;
        }
        cluster.centroid_x = sum_x / cluster.cells.size();
        cluster.centroid_y = sum_y / cluster.cells.size();
        clusters.push_back(cluster);
      }
    }
    return clusters;
  }

  bool is_blacklisted(double x, double y)
  {
    for (const auto & b : blacklist_) {
      if (std::hypot(b.first - x, b.second - y) < 0.3) // 30cm tolerance
        return true;
    }
    return false;
  }

  bool pick_closest_cluster_centroid(
    const std::vector<Cluster> & clusters,
    const geometry_msgs::msg::Pose & pose,
    geometry_msgs::msg::Point & goal_point)
  {
    double min_dist = std::numeric_limits<double>::max();
    bool found = false;
    for (const auto & c : clusters) {
      if (is_blacklisted(c.centroid_x, c.centroid_y)) continue;
      double dx = c.centroid_x - pose.position.x;
      double dy = c.centroid_y - pose.position.y;
      double dist = std::hypot(dx, dy);
      if (dist < min_dist) {
        min_dist = dist;
        goal_point.x = c.centroid_x;
        goal_point.y = c.centroid_y;
        goal_point.z = 0.0;
        found = true;
      }
    }
    return found;
  }

  // Helper to check if a map cell is free
  bool is_cell_free(int x, int y, const nav_msgs::msg::OccupancyGrid & map) {
    int width = map.info.width;
    int height = map.info.height;
    if (x < 0 || x >= width || y < 0 || y >= height) return false;
    int idx = y * width + x;
    return map.data[idx] == 0;
  }

  // Find a nearby free cell to the centroid (if centroid is not free)
  geometry_msgs::msg::Point find_nearest_free(const nav_msgs::msg::OccupancyGrid & map, double cx, double cy) {
    int width = map.info.width;
    int height = map.info.height;
    double res = map.info.resolution;
    int mx = static_cast<int>((cx - map.info.origin.position.x) / res);
    int my = static_cast<int>((cy - map.info.origin.position.y) / res);

    // Search in a spiral outwards
    for (int r = 0; r < 10; ++r) {
      for (int dx = -r; dx <= r; ++dx) {
        for (int dy = -r; dy <= r; ++dy) {
          int nx = mx + dx;
          int ny = my + dy;
          if (is_cell_free(nx, ny, map)) {
            geometry_msgs::msg::Point p;
            p.x = map.info.origin.position.x + (nx + 0.5) * res;
            p.y = map.info.origin.position.y + (ny + 0.5) * res;
            p.z = 0.0;
            return p;
          }
        }
      }
    }
    // If no free cell found, return original centroid
    geometry_msgs::msg::Point p;
    p.x = cx;
    p.y = cy;
    p.z = 0.0;
    return p;
  }

  void send_goal(const geometry_msgs::msg::Point & goal_point)
  {
    // Check if map is available
    if (!latest_map_) {
      RCLCPP_WARN(this->get_logger(), "No map available for goal validation.");
      return;
    }

    // Check if goal is in free space, otherwise find nearest free cell
    double res = latest_map_->info.resolution;
    int mx = static_cast<int>((goal_point.x - latest_map_->info.origin.position.x) / res);
    int my = static_cast<int>((goal_point.y - latest_map_->info.origin.position.y) / res);
    geometry_msgs::msg::Point safe_goal = goal_point;
    if (!is_cell_free(mx, my, *latest_map_)) {
      RCLCPP_WARN(this->get_logger(), "Centroid (%.2f, %.2f) is not in free space. Searching for nearest free cell.", goal_point.x, goal_point.y);
      safe_goal = find_nearest_free(*latest_map_, goal_point.x, goal_point.y);
      if (!is_cell_free(static_cast<int>((safe_goal.x - latest_map_->info.origin.position.x) / res),
                        static_cast<int>((safe_goal.y - latest_map_->info.origin.position.y) / res),
                        *latest_map_)) {
        RCLCPP_ERROR(this->get_logger(), "No free cell found near centroid (%.2f, %.2f). Skipping goal.", goal_point.x, goal_point.y);
        blacklist_.emplace_back(goal_point.x, goal_point.y);
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Using nearest free cell at (%.2f, %.2f) as goal.", safe_goal.x, safe_goal.y);
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position = safe_goal;
    goal_msg.pose.pose.orientation.w = 1.0; // face forward

    navigating_ = true;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
      [this, safe_goal](const GoalHandleNavigateToPose::WrappedResult & result) {
        navigating_ = false;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Reached frontier cluster centroid!");
        } else {
          std::string reason;
          switch (result.code) {
            case rclcpp_action::ResultCode::ABORTED: reason = "ABORTED"; break;
            case rclcpp_action::ResultCode::CANCELED: reason = "CANCELED"; break;
            default: reason = "UNKNOWN"; break;
          }
          RCLCPP_WARN(this->get_logger(), "Failed to reach frontier cluster centroid (%.2f, %.2f). Result: %s. Blacklisting.", safe_goal.x, safe_goal.y, reason.c_str());
          blacklist_.emplace_back(safe_goal.x, safe_goal.y);
        }
      };

    nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "Sent navigation goal to cluster centroid (%.2f, %.2f)", safe_goal.x, safe_goal.y);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExplorerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
