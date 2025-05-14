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
#include <cmath>
#include <cstdlib>
#include <algorithm>

using std::placeholders::_1;

class RRTNavigator : public rclcpp::Node
{
public:
    RRTNavigator() : Node("rrt_navigator"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&RRTNavigator::map_callback, this, _1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_point", 10, std::bind(&RRTNavigator::goal_callback, this, _1));
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
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_msg_ = msg;
        map_data_ = msg->data;
        width_ = msg->info.width;
        height_ = msg->info.height;
        resolution_ = msg->info.resolution;
        origin_ = msg->info.origin;
        RCLCPP_INFO(this->get_logger(), "Map received");
    }

    std::pair<int, int> world_to_grid(double x, double y)
    {
        int gx = static_cast<int>((x - origin_.position.x) / resolution_);
        int gy = static_cast<int>((y - origin_.position.y) / resolution_);
        return {gx, gy};
    }

    bool is_free(int x, int y)
    {
        if (x < 0 || y < 0 || x >= static_cast<int>(width_) || y >= static_cast<int>(height_))
            return false;
        int idx = y * width_ + x;
        return map_data_[idx] == 0;
    }

    struct Node
    {
        int x, y;
        Node *parent;
        Node(int x, int y, Node *p = nullptr) : x(x), y(y), parent(p) {}
    };

    std::vector<std::pair<int, int>> rrt(const std::pair<int, int> &start, const std::pair<int, int> &goal)
    {
        std::vector<Node *> tree;
        Node *start_node = new Node(start.first, start.second);
        tree.push_back(start_node);

        int max_iters = 500;
        int step_size = 5;
        int goal_thresh = 5;

        for (int i = 0; i < max_iters; ++i)
        {
            int rx = rand() % width_;
            int ry = rand() % height_;
            if (!is_free(rx, ry))
                continue;

            Node *nearest = nullptr;
            double min_dist = 1e9;
            for (auto *n : tree)
            {
                double dist = std::hypot(rx - n->x, ry - n->y);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    nearest = n;
                }
            }
            if (!nearest)
                continue;

            double theta = std::atan2(ry - nearest->y, rx - nearest->x);
            int nx = nearest->x + static_cast<int>(step_size * std::cos(theta));
            int ny = nearest->y + static_cast<int>(step_size * std::sin(theta));
            if (!is_free(nx, ny))
                continue;

            Node *new_node = new Node(nx, ny, nearest);
            tree.push_back(new_node);

            if (std::hypot(goal.first - nx, goal.second - ny) < goal_thresh)
            {
                Node *goal_node = new Node(goal.first, goal.second, new_node);
                tree.push_back(goal_node);

                std::vector<std::pair<int, int>> path;
                for (Node *n = goal_node; n != nullptr; n = n->parent)
                    path.emplace_back(n->x, n->y);
                std::reverse(path.begin(), path.end());
                for (auto *n : tree) delete n;
                return path;
            }
        }
        RCLCPP_WARN(this->get_logger(), "RRT failed to find a path");
        for (auto *n : tree) delete n;
        return {};
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
    {
        if (!map_msg_)
        {
            RCLCPP_WARN(this->get_logger(), "Map not yet received");
            return;
        }

        try
        {
            auto tf = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            auto start = world_to_grid(tf.transform.translation.x, tf.transform.translation.y);
            auto goal = world_to_grid(goal_msg->pose.position.x, goal_msg->pose.position.y);
            RCLCPP_INFO(this->get_logger(), "Planning from (%d,%d) to (%d,%d)", start.first, start.second, goal.first, goal.second);

            auto path = rrt(start, goal);
            if (!path.empty())
            {
                nav_msgs::msg::Path path_msg;
                path_msg.header.stamp = this->now();
                path_msg.header.frame_id = "map";

                for (auto &[x, y] : path)
                {
                    geometry_msgs::msg::PoseStamped p;
                    p.header = path_msg.header;
                    p.pose.position.x = x * resolution_ + origin_.position.x;
                    p.pose.position.y = y * resolution_ + origin_.position.y;
                    p.pose.orientation.w = 1.0;
                    path_msg.poses.push_back(p);
                }
                path_pub_->publish(path_msg);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "No path found");
            }
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF error: %s", ex.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRTNavigator>());
    rclcpp::shutdown();
    return 0;
}
