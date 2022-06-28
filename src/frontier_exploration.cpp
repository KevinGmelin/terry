#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Header.h"
#include "tf/transform_listener.h"
#include <unordered_set>
#include <queue>
#include <chrono>

using cell_t = size_t;

static bool is_valid_cell(const cell_t &cell, const nav_msgs::MapMetaData &map_meta_data) {
  if (cell < 0) return false;
  const cell_t map_size = map_meta_data.width * map_meta_data.height;
  if (cell >= map_size) return false;
  return true;
}

static auto get_neighbors(const cell_t &cell, const nav_msgs::MapMetaData &map_meta_data) {
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

static bool is_frontier(const cell_t &cell, const nav_msgs::OccupancyGrid &map) {
  if (!is_unknown(cell, map)) return false;
  const auto neighbors = get_neighbors(cell, map.info);
  return std::any_of(neighbors.cbegin(),
                     neighbors.cend(),
                     [&map](const cell_t neighbor) { return is_open(neighbor, map); });
}

static bool has_open_neighbor(const cell_t &cell, const nav_msgs::OccupancyGrid &map) {
  const auto neighbors = get_neighbors(cell, map.info);
  return std::any_of(neighbors.cbegin(),
                     neighbors.cend(),
                     [&map](const cell_t neighbor) { return is_open(neighbor, map); });
}

static auto position_to_cell(const float x, const float y, const nav_msgs::MapMetaData &map_meta_data) {
  const auto &width = map_meta_data.width;
  const auto &resolution = map_meta_data.resolution;
  const auto &x_origin = map_meta_data.origin.position.x;
  const auto &y_origin = map_meta_data.origin.position.y;
  const cell_t cell = size_t(floor((x - x_origin) / resolution) + floor((y - y_origin) / resolution) * width);
  return cell;
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
      for (const auto &neighbor: get_neighbors(cell, map.info)) {
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
    }
    for (const auto &neighbor: get_neighbors(cell, map.info)) {
      if (map_open_list.find(neighbor) != map_open_list.end()) continue;
      if (map_closed_list.find(neighbor) != map_closed_list.end()) continue;
      if (!has_open_neighbor(neighbor, map)) continue;
      cell_queue.push(neighbor);
      map_open_list.insert(neighbor);
    }
    map_closed_list.insert(cell);
  }
  return frontiers;
}

class FrontierExploration {
 public:
  FrontierExploration() {
    debug_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("debug_map", 1);
    map_sub = nh.subscribe("map", 1, &FrontierExploration::map_callback, this);
    get_frontiers_sub = nh.subscribe("get_frontiers", 1, &FrontierExploration::get_frontiers_callback, this);
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

  void map_callback(const nav_msgs::OccupancyGrid &msg) {
    map = msg;
    tf::StampedTransform transform;
    try {
      listener.lookupTransform("base_footprint", "map",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      robot_cell = std::nullopt;
      return;
    }
    robot_cell = position_to_cell(transform.getOrigin().x(), transform.getOrigin().y(), map->info);
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
    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    ROS_INFO("Getting all frontiers took: %ld milliseconds", duration.count());
    draw_debug_map(frontiers);
    debug_map_pub.publish(debug_map);
  }

  void draw_debug_map(const std::vector<std::vector<cell_t>> &frontiers) {
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
  }

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "frontier_exploration");
  FrontierExploration node;
  ros::spin();
  return 0;
}