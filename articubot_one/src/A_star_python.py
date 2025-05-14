#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import heapq
import tf2_ros
import tf_transformations
from tf2_geometry_msgs import PoseStamped as TF2PoseStamped

class AStarNavigator(Node):

    def __init__(self):
        super().__init__('a_star_navigator')

        self.map_sub = self.create_subscription(OccupancyGrid,'/map', self.map_callback, 10)
        self.goal_pub = self.create_subscription(PoseStamped,'/goal_pose', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.map_data = None
        self.map_info = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        self.get_logger().info('Map received')

    def goal_callback(self, msg):
        if self.map_data is None:
            self.get_logger().warn('Map not yet received')
            return
        
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            current_pose = trans.transform.translation
            start = self.world_to_grid(current_pose.x, current_pose.y)
            goal = self.world_to_grid(msg.pose.position.x, msg.pose.position.y)
            
            self.get_logger().info(f"Planning from {start} to {goal}")
            path = self.a_star(start, goal)
            if path:
                self.get_logger().info(f"Path found with {len(path)} points:\n{path}")
                self.publish_path(path)  # Optional: if you're visualizing too
            else:
                self.get_logger().warn("No path found.")

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

    def world_to_grid(self, x, y):
        origin = self.map_info.origin.position
        res = self.map_info.resolution
        gx = int((x - origin.x) / res)
        gy = int((y - origin.y) / res)
        return (gx, gy)

    def is_valid(self, x, y):
        h, w = self.map_data.shape
        return 0 <= x < w and 0 <= y < h and self.map_data[y][x] == 0

    def a_star(self, start, goal):
        h,w = self.map_data.shape
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start, goal), 0, start, [start]))
        visited = set()

        while open_set:
            est_total, cost, current, path = heapq.heappop(open_set)
            if current in visited:
                continue
            visited.add(current)
            if current == goal:
                return path
            
            x,y = current
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                nx, ny = x+dx, y+dy
                if self.is_valid(nx,ny):
                    next_node = (nx,ny)
                    step_cost = 1.414 if dx != 0 and dy != 0 else 1
                    new_cost = cost + step_cost
                    f = new_cost + self.heuristic(next_node, goal)
                    heapq.heappush(open_set, (f, new_cost, next_node, path + [next_node] ))
        
        self.get_logger().warn("A* failed to find a path.")
        return []
    
    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance
    
def main(arg=None):
    rclpy.init(args=args)
    node = AStarNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()