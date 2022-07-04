#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Header.h"
#include "tf/transform_listener.h"
#include <unordered_set>
#include <queue>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using cell_t = size_t;
using point_t = Eigen::Vector2f;
using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

static bool is_valid_cell(const cell_t &cell, const nav_msgs::MapMetaData &map_meta_data) {
  if (cell < 0) return false;
  const cell_t map_size = map_meta_data.width * map_meta_data.height;
  if (cell >= map_size) return false;
  return true;
}

static auto get_4_neighbors(const cell_t &cell, const nav_msgs::MapMetaData &map_meta_data) {
  std::vector<cell_t> neighbors;
  const auto &width = map_meta_data.width;

  const cell_t right_neighbor = cell + 1;
  if (is_valid_cell(right_neighbor, map_meta_data)) neighbors.push_back(right_neighbor);

  const cell_t left_neighbor = cell - 1;
  if (is_valid_cell(left_neighbor, map_meta_data)) neighbors.push_back(left_neighbor);

  const cell_t top_neighbor = cell + width;
  if (is_valid_cell(top_neighbor, map_meta_data)) neighbors.push_back(top_neighbor);

  const cell_t bottom_neighbor = cell - width;
  if (is_valid_cell(bottom_neighbor, map_meta_data)) neighbors.push_back(bottom_neighbor);

  return neighbors;
}

static auto get_8_neighbors(const cell_t &cell, const nav_msgs::MapMetaData &map_meta_data) {
  std::vector<cell_t> neighbors;
  const auto &width = map_meta_data.width;

  const cell_t right_neighbor = cell + 1;
  if (is_valid_cell(right_neighbor, map_meta_data)) neighbors.push_back(right_neighbor);

  const cell_t left_neighbor = cell - 1;
  if (is_valid_cell(left_neighbor, map_meta_data)) neighbors.push_back(left_neighbor);

  const cell_t top_neighbor = cell + width;
  if (is_valid_cell(top_neighbor, map_meta_data)) neighbors.push_back(top_neighbor);
  if (is_valid_cell(top_neighbor - 1, map_meta_data)) neighbors.push_back(top_neighbor - 1);
  if (is_valid_cell(top_neighbor + 1, map_meta_data)) neighbors.push_back(top_neighbor + 1);

  const cell_t bottom_neighbor = cell - width;
  if (is_valid_cell(bottom_neighbor, map_meta_data)) neighbors.push_back(bottom_neighbor);
  if (is_valid_cell(bottom_neighbor - 1, map_meta_data)) neighbors.push_back(bottom_neighbor - 1);
  if (is_valid_cell(bottom_neighbor + 1, map_meta_data)) neighbors.push_back(bottom_neighbor + 1);

  return neighbors;
}

static bool is_unknown(const cell_t &cell, const nav_msgs::OccupancyGrid &map) {
  if (!is_valid_cell(cell, map.info)) {
    ROS_ERROR("Called is_unknown() with invalid cell value.");
    return false;
  }
  return map.data[cell] == -1;
}

static bool is_open(const cell_t &cell, const nav_msgs::OccupancyGrid &map) {
  if (!is_valid_cell(cell, map.info)) {
    ROS_ERROR("Called is_open() with invalid cell value.");
    return false;
  }
  const auto cell_value = map.data[cell];
  return cell_value >= 0 and cell_value < 50;
}

static bool is_closed(const cell_t &cell, const nav_msgs::OccupancyGrid &map) {
  if (!is_valid_cell(cell, map.info)) {
    ROS_ERROR("Called is_open() with invalid cell value.");
    return false;
  }
  return map.data[cell] > 50;
}

static bool is_frontier(const cell_t &cell, const nav_msgs::OccupancyGrid &map) {
  if (!is_unknown(cell, map)) return false;
  const auto neighbors = get_4_neighbors(cell, map.info);
  return std::any_of(neighbors.cbegin(),
                     neighbors.cend(),
                     [&map](const cell_t neighbor) { return is_open(neighbor, map); });
}

static auto position_to_cell(const point_t &point, const nav_msgs::MapMetaData &map_meta_data) {
  const auto &width = map_meta_data.width;
  const auto &resolution = map_meta_data.resolution;
  const auto &x_origin = map_meta_data.origin.position.x;
  const auto &y_origin = map_meta_data.origin.position.y;
  const cell_t
      cell = size_t(floor((point(0) - x_origin) / resolution) + floor((point(1) - y_origin) / resolution) * width);
  return cell;
}

static auto cell_to_position(const cell_t &cell, const nav_msgs::MapMetaData &map_meta_data) {
  const auto &width = map_meta_data.width;
  const auto &resolution = map_meta_data.resolution;
  const auto &x_origin = map_meta_data.origin.position.x;
  const auto &y_origin = map_meta_data.origin.position.y;
  const int row = int(cell / width);
  const int col = cell % width;
  const float x = x_origin + col * resolution;
  const float y = y_origin + row * resolution;
  return point_t{x, y};
}

static auto traverse_frontier(const cell_t &starting_cell,
                              const nav_msgs::OccupancyGrid &map,
                              std::unordered_set<cell_t> &map_closed_list) {
  std::queue<cell_t> frontier_queue;
  std::unordered_set<cell_t> frontier_open_list;
  std::unordered_set<cell_t> frontier_closed_list;
  std::vector<cell_t> new_frontier;
  frontier_queue.push(starting_cell);
  frontier_open_list.insert(starting_cell);
  while (!frontier_queue.empty()) {
    const auto cell = frontier_queue.front();
    frontier_queue.pop();
    if (map_closed_list.find(cell) != map_closed_list.end()) continue;
    if (frontier_closed_list.find(cell) != frontier_closed_list.end()) continue;
    if (is_frontier(cell, map)) {
      new_frontier.push_back(cell);
      for (const auto &neighbor: get_8_neighbors(cell, map.info)) {
        if (frontier_open_list.find(neighbor) != frontier_open_list.end()) continue;
        if (frontier_closed_list.find(neighbor) != frontier_closed_list.end()) continue;
        if (map_closed_list.find(neighbor) != map_closed_list.end()) continue;
        frontier_queue.push(neighbor);
        frontier_open_list.insert(neighbor);
      }
    }
    frontier_closed_list.insert(cell);
  }
  for (const auto &cell: new_frontier) map_closed_list.insert(cell);
  return new_frontier;
}

static auto get_frontiers(const cell_t &robot_cell, const nav_msgs::OccupancyGrid &map) {
  std::vector<std::vector<cell_t>> frontiers;
  std::queue<cell_t> cell_queue;
  std::unordered_set<cell_t> map_open_list;
  std::unordered_set<cell_t> map_closed_list;
  cell_queue.push(robot_cell);
  map_open_list.insert(robot_cell);
  while (!cell_queue.empty()) {
    const auto cell = cell_queue.front();
    cell_queue.pop();
    if (map_closed_list.find(cell) != map_closed_list.end()) {
      continue;
    }
    if (is_frontier(cell, map)) {
      frontiers.push_back(traverse_frontier(cell, map, map_closed_list));
    } else {
      for (const auto &neighbor: get_4_neighbors(cell, map.info)) {
        if (map_open_list.find(neighbor) != map_open_list.end()) continue;
        if (map_closed_list.find(neighbor) != map_closed_list.end()) continue;
        if (is_closed(neighbor, map)) continue;
        cell_queue.push(neighbor);
        map_open_list.insert(neighbor);
      }
    }
    map_closed_list.insert(cell);
  }
  return frontiers;
}

static auto get_frontier_centroid(const std::vector<cell_t> &frontier, const nav_msgs::MapMetaData &map_meta_data) {
  if (frontier.empty()) {
    ROS_ERROR("get_frontier_centroid: Frontier is empty.");
    return point_t{-1, -1};
  }
  float x_sum = 0;
  float y_sum = 0;
  for (const auto &cell: frontier) {
    const auto point = cell_to_position(cell, map_meta_data);
    x_sum += point(0);
    y_sum += point(1);
  }
  const float x = x_sum / frontier.size();
  const float y = y_sum / frontier.size();
  return point_t{x, y};
}

static float distance(point_t point1, point_t point2) {
  return std::sqrt(std::pow(point1(0) - point2(0), 2) + std::pow(point1(1) - point2(1), 2));
}

static auto get_nearest_frontier(const cell_t &robot_cell,
                                 const std::vector<std::vector<cell_t>> &frontiers,
                                 const nav_msgs::MapMetaData &map_meta_data) {
  if (frontiers.empty()) {
    ROS_ERROR("get_nearest_frontier: Frontiers is empty.");
    return point_t{-1, -1};
  }
  const auto robot_point = cell_to_position(robot_cell, map_meta_data);
  point_t nearest_frontier_centroid;
  float nearest_distance = std::numeric_limits<float>::max();
  for (const auto &frontier: frontiers) {
    const auto frontier_centroid = get_frontier_centroid(frontier, map_meta_data);
    const float dist_to_centroid = distance(robot_point, nearest_frontier_centroid);
    if (dist_to_centroid < nearest_distance) {
      nearest_distance = dist_to_centroid;
      nearest_frontier_centroid = frontier_centroid;
    }
  }
  return nearest_frontier_centroid;
}

class FrontierExploration {
 public:
  FrontierExploration() :
      move_base_client("move_base", true) {
    debug_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("debug_map", 1);
    map_sub = nh.subscribe("map", 1, &FrontierExploration::map_callback, this);
    get_frontiers_sub = nh.subscribe("get_frontiers", 1, &FrontierExploration::get_frontiers_callback, this);

    while (!move_base_client.waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    timer = nh.createTimer(ros::Duration(5.0), &FrontierExploration::timer_callback, this);
  }

 private:
  ros::NodeHandle nh;
  ros::Publisher debug_map_pub;
  ros::Subscriber map_sub;
  ros::Subscriber get_frontiers_sub;
  std::optional<nav_msgs::OccupancyGrid> map;
  nav_msgs::OccupancyGrid debug_map;
  std::optional<cell_t> robot_cell;
  tf::TransformListener listener;
  MoveBaseClient move_base_client;
  ros::Timer timer;

  void map_callback(const nav_msgs::OccupancyGrid &msg) {
    map = msg;
    tf::StampedTransform transform;
    try {
      listener.lookupTransform("map", "base_footprint",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      robot_cell = std::nullopt;
      return;
    }
    robot_cell = position_to_cell({transform.getOrigin().x(), transform.getOrigin().y()}, map->info);
  }

  void get_frontiers_callback(const std_msgs::Empty) {
    if (!robot_cell.has_value()) {
      ROS_ERROR("get_frontiers_callback: Robot_cell has no value.");
      return;
    }
    if (!map.has_value()) {
      ROS_ERROR("get_frontiers_callback: Map has no value.");
      return;
    }
    const auto start_time = std::chrono::high_resolution_clock::now();
    const auto frontiers = get_frontiers(robot_cell.value(), map.value());
    const auto nearest_frontier = get_nearest_frontier(robot_cell.value(), frontiers, map->info);
    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    ROS_INFO("Getting nearest frontier took: %ld milliseconds", duration.count());
    const auto nearest_frontier_cell = position_to_cell(nearest_frontier, map->info);
    draw_debug_map(frontiers, nearest_frontier_cell);
    debug_map_pub.publish(debug_map);
  }

  void draw_debug_map(const std::vector<std::vector<cell_t>> &frontiers, const cell_t &nearest_frontier_cell) {
    debug_map.info = map->info;
    debug_map.header = std_msgs::Header();
    debug_map.header.frame_id = map->header.frame_id;
    debug_map.header.stamp = ros::Time::now();
    debug_map.data = std::vector<std::int8_t>(debug_map.info.width * debug_map.info.height, 50);
    debug_map.data[robot_cell.value()] = 0;
    for (const auto &frontier: frontiers) {
      for (const auto &cell: frontier) {
        debug_map.data[cell] = 100;
      }
    }
    debug_map.data[nearest_frontier_cell] = 0;
  }

  void timer_callback(const ros::TimerEvent &event) {
    if (!robot_cell.has_value()) {
      ROS_ERROR("timer_callback: Robot_cell has no value.");
      return;
    }
    if (!map.has_value()) {
      ROS_ERROR("timer_callback: Map has no value.");
      return;
    }
    const auto start_time = std::chrono::high_resolution_clock::now();
    const auto frontiers = get_frontiers(robot_cell.value(), map.value());
    if (frontiers.empty()) {
      ROS_ERROR("timer_callback: Frontiers is empty.");
      return;
    }
    const auto nearest_frontier = get_nearest_frontier(robot_cell.value(), frontiers, map->info);
    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    ROS_INFO("Getting nearest frontier took: %ld milliseconds", duration.count());

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = nearest_frontier(0);
    goal.target_pose.pose.position.y = nearest_frontier(1);
    goal.target_pose.pose.orientation.w = 1;
    ROS_INFO("Sending goal");
    move_base_client.sendGoal(goal);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "terry_planning");
  FrontierExploration node;
  ros::spin();
  return 0;
}