#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <vector>
#include <queue>
#include <unordered_set>
#include <cmath>

using std::placeholders::_1;

class AStarNavigator : public rclcpp::Node {
public:
    AStarNavigator() : Node("a_star_navigator"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&AStarNavigator::map_callback, this, _1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&AStarNavigator::goal_callback, this, _1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    }

private:
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
    std::vector<int8_t> map_data_;
    unsigned int width_, height_;
    double resolution_;
    geometry_msgs::msg::Pose origin_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_msg_ = msg;
        map_data_ = msg->data;
        width_ = msg->info.width;
        height_ = msg->info.height;
        resolution_ = msg->info.resolution;
        origin_ = msg->info.origin;
        RCLCPP_INFO(this->get_logger(), "Map received");
    }

    std::pair<int, int> world_to_grid(double x, double y) {
        int gx = static_cast<int>((x - origin_.position.x) / resolution_);
        int gy = static_cast<int>((y - origin_.position.y) / resolution_);
        return {gx, gy};
    }

    bool is_valid(int x, int y) {
        if (x < 0 || y < 0 || x >= static_cast<int>(width_) || y >= static_cast<int>(height_))
            return false;
        return map_data_[y * width_ + x] == 0;
    }

    double heuristic(std::pair<int, int> a, std::pair<int, int> b) {
        return std::abs(a.first - b.first) + std::abs(a.second - b.second);
    }

    std::vector<std::pair<int, int>> a_star(std::pair<int, int> start, std::pair<int, int> goal) {
        using Node = std::tuple<double, double, std::pair<int, int>, std::vector<std::pair<int, int>>>;
        auto cmp = [](const Node& a, const Node& b) { return std::get<0>(a) > std::get<0>(b); };
        std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open_set(cmp);

        open_set.push({heuristic(start, goal), 0.0, start, {start}});
        std::unordered_set<int> visited;

        while (!open_set.empty()) {
            auto [f, g, current, path] = open_set.top();
            open_set.pop();
            int flat_index = current.second * width_ + current.first;
            if (visited.count(flat_index)) continue;
            visited.insert(flat_index);

            if (current == goal) return path;

            for (auto [dx, dy] : std::vector<std::pair<int, int>>{{-1,0},{1,0},{0,-1},{0,1},{-1,-1},{-1,1},{1,-1},{1,1}}) {
                int nx = current.first + dx;
                int ny = current.second + dy;
                if (!is_valid(nx, ny)) continue;

                double step_cost = (dx != 0 && dy != 0) ? 1.414 : 1.0;
                double new_cost = g + step_cost;
                auto next_node = std::make_pair(nx, ny);
                auto new_path = path;
                new_path.push_back(next_node);
                open_set.push({new_cost + heuristic(next_node, goal), new_cost, next_node, new_path});
            }
        }

        RCLCPP_WARN(this->get_logger(), "A* failed to find a path.");
        return {};
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg) {
        if (!map_msg_) {
            RCLCPP_WARN(this->get_logger(), "Map not yet received");
            return;
        }

        try {
            auto transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            auto start = world_to_grid(transform.transform.translation.x, transform.transform.translation.y);
            auto goal = world_to_grid(goal_msg->pose.position.x, goal_msg->pose.position.y);
            RCLCPP_INFO(this->get_logger(), "Planning from (%d,%d) to (%d,%d)", start.first, start.second, goal.first, goal.second);

            auto path = a_star(start, goal);
            if (!path.empty()) {
                RCLCPP_INFO(this->get_logger(), "Path found with %zu points", path.size());
                RCLCPP_INFO(this->get_logger(), "Goal callback triggered");
                nav_msgs::msg::Path path_msg;
                path_msg.header.stamp = this->get_clock()->now();
                path_msg.header.frame_id = "map";

                for (const auto& [x, y] : path) {
                    geometry_msgs::msg::PoseStamped pose_stamped;
                    pose_stamped.header = path_msg.header;
                    pose_stamped.pose.position.x = x * resolution_ + origin_.position.x;
                    pose_stamped.pose.position.y = y * resolution_ + origin_.position.y;
                    pose_stamped.pose.position.z = 0.0;
                    pose_stamped.pose.orientation.w = 1.0;
                    path_msg.poses.push_back(pose_stamped);
                }

                path_pub_->publish(path_msg);
                RCLCPP_INFO(this->get_logger(), "Published path to /planned_path");
            } else {
                RCLCPP_WARN(this->get_logger(), "No path found.");
            }
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        }
    }
};

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarNavigator>());
    rclcpp::shutdown();
    return 0;
}
