#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <vector>
#include <cmath>
#include <cstdlib>
#include <algorithm>

using std::placeholders::_1;

class DRRTNavigator : public rclcpp::Node
{
public:
    DRRTNavigator() : rclcpp::Node("drrt_navigator"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&DRRTNavigator::map_callback, this, _1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&DRRTNavigator::goal_callback, this, _1));
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_rrt_path", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&DRRTNavigator::scan_callback,this,_1));
    }

private:
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
    std::vector<int8_t> map_data_;
    unsigned int width_, height_;
    double resolution_;
    geometry_msgs::msg::Pose origin_;

    std::vector<std::pair<double,double>> rrt_path_world_;
    std::vector<std::pair<double,double>> lidar_obstacles_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

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

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            transformStamped = tf_buffer_.lookupTransform("map", msg->header.frame_id, tf2::TimePointZero);
        }
        catch(tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform scan: %s", ex.what());
            return;
        }
        
        double angle = msg->angle_min;

        for( size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float r = msg->ranges[i];
            if(std::isnan(r) || std::isinf(r) || r > msg->range_max)
            {
                angle += msg->angle_increment;
                continue;
            }

            geometry_msgs::msg::PointStamped pt_lidar;
            pt_lidar.header = msg->header;
            pt_lidar.point.x = r*std::cos(angle);
            pt_lidar.point.y = r*std::sin(angle);
            pt_lidar.point.z = 0.0;

            geometry_msgs::msg::PointStamped pt_map;
            
            tf2::doTransform(pt_lidar, pt_map, transformStamped);

            lidar_obstacles_.emplace_back(pt_map.point.x, pt_map.point.y);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10,"Lidar obstacle at [%.2f,%.2f]", pt_map.point.x, pt_map.point.y);

            angle += msg->angle_increment;
        }
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
        return map_data_[idx] >= 0 && map_data_[idx] < 50;
    }

    struct Node
    {
        int x, y;
        Node* parent;
        Node(int x, int y, Node* p = nullptr) : x(x), y(y), parent(p) {}
    };

    std::vector<std::pair<int, int>> rrt(const std::pair<int, int>& start, const std::pair<int, int>& goal)
    {
        std::vector<Node*> tree;
        Node* start_node = new Node(start.first, start.second);
        tree.push_back(start_node);

        int step_size = 5;
        int itermax = 3000;
        int goal_thresh = 5;

        for (int i = 0; i < itermax; ++i)
        {
            int rx = rand() % width_;
            int ry = rand() % height_;
            if (!is_free(rx, ry))
                continue;

            Node* nearest = nullptr;
            double min_dist = std::numeric_limits<double>::infinity();
            for (auto* n : tree)
            {
                double dx = n->x - rx;
                double dy = n->y - ry;
                double distance = std::hypot(dx, dy);
                if (distance < min_dist)
                {
                    min_dist = distance;
                    nearest = n;
                }
            }

            double dx = rx - nearest->x;
            double dy = ry - nearest->y;
            double ndis = std::hypot(dx, dy);
            if (ndis == 0)
                continue;

            int new_x = nearest->x + static_cast<int>(dx / ndis * step_size);
            int new_y = nearest->y + static_cast<int>(dy / ndis * step_size);

            if (!is_free(new_x, new_y))
                continue;

            RCLCPP_WARN(this->get_logger(), "RRT is assigning node at: %d, %d", new_x, new_y);

            Node* new_node = new Node(new_x, new_y, nearest);
            tree.push_back(new_node);

            if (std::hypot(new_node->x - goal.first, new_node->y - goal.second) < goal_thresh)
            {
                Node* goal_node = new Node(goal.first, goal.second, new_node);
                tree.push_back(goal_node);

                // Backtrack đường đi
                std::vector<std::pair<int, int>> path;
                for (Node* n = goal_node; n != nullptr; n = n->parent)
                    path.emplace_back(n->x, n->y);
                std::reverse(path.begin(), path.end());

                // Dọn dẹp bộ nhớ
                for (auto* n : tree)
                    delete n;

                return path;
            }
        }

        RCLCPP_WARN(this->get_logger(), "RRT failed to find a path");

        // Trả về toàn bộ cây (toàn bộ node) nếu không tìm được đường
        std::vector<std::pair<int, int>> tree_points;
        for (auto* n : tree)
            tree_points.emplace_back(n->x, n->y);

        for (auto* n : tree)
            delete n;

        return tree_points;
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

                for (auto& [x, y] : path)
                {
                    double wx = x * resolution_ + origin_.position.x;
                    double wy = y * resolution_ + origin_.position.y;
                    
                    rrt_path_world_.emplace_back(wx,wy);

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
                RCLCPP_WARN(this->get_logger(), "No path or tree nodes to publish");
            }
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF error: %s", ex.what());
        }
    }


};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DRRTNavigator>());
    rclcpp::shutdown();
    return 0;
}
