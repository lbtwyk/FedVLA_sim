/**
 * @file cube_spawner_node.hpp
 * @brief ROS 2 node for spawning colored cubes in the Gazebo simulation.
 *
 * This node spawns yellow and orange cubes in the Gazebo simulation
 * for the perception system to detect. It supports randomization of
 * cube positions within specified bounds and ensures cubes don't overlap.
 *
 * @author Yukun Wang
 */

#ifndef MYCOBOT_STACKING_PROJECT__CUBE_SPAWNER_NODE_HPP_
#define MYCOBOT_STACKING_PROJECT__CUBE_SPAWNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <string>
#include <random>
#include <cmath>
#include <sstream>
#include <fstream>
#include <filesystem>

namespace mycobot_stacking_project
{

/**
 * @class CubeSpawnerNode
 * @brief ROS 2 node for spawning colored cubes in the Gazebo simulation.
 */
class CubeSpawnerNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the CubeSpawnerNode.
   * @param options Node options.
   */
  explicit CubeSpawnerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Spawn the yellow cube in the simulation.
   */
  void spawn_yellow_cube();

  /**
   * @brief Spawn the orange cube in the simulation.
   */
  void spawn_orange_cube();

  /**
   * @brief Spawn the ground plane in the simulation.
   */
  void spawn_ground_plane();

  /**
   * @brief Generate a world file with randomized cube positions.
   * @param template_world_path Path to the template world file.
   * @param output_world_path Path to the output world file.
   * @return True if successful, false otherwise.
   */
  bool generate_world_file(const std::string& template_world_path, const std::string& output_world_path);

  /**
   * @brief Timer callback for spawning cubes.
   */
  void timer_callback();

  /**
   * @brief Generate a random position within the specified bounds.
   * @param min_x Minimum X coordinate.
   * @param max_x Maximum X coordinate.
   * @param min_y Minimum Y coordinate.
   * @param max_y Maximum Y coordinate.
   * @param z Z coordinate (height).
   * @return A randomly generated position.
   */
  geometry_msgs::msg::Point generate_random_position(
    double min_x, double max_x, double min_y, double max_y, double z);

  /**
   * @brief Check if two cubes would collide at the given positions.
   * @param pos1 Position of the first cube.
   * @param pos2 Position of the second cube.
   * @param min_distance Minimum allowed distance between cube centers.
   * @return True if the cubes would collide, false otherwise.
   */
  bool check_collision(
    const geometry_msgs::msg::Point& pos1,
    const geometry_msgs::msg::Point& pos2,
    double min_distance);

  // Timer for spawning cubes
  rclcpp::TimerBase::SharedPtr timer_;

  // Random number generator
  std::mt19937 rng_;

  // Randomization parameters
  // Yellow cube parameters
  double yellow_min_x_;
  double yellow_max_x_;
  double yellow_min_y_;
  double yellow_max_y_;

  // Orange cube parameters
  double orange_min_x_;
  double orange_max_x_;
  double orange_min_y_;
  double orange_max_y_;

  // Common parameters
  double cube_size_;
  double min_distance_between_cubes_;
  bool randomize_positions_;

  // Cube positions
  geometry_msgs::msg::Point yellow_cube_position_;
  geometry_msgs::msg::Point orange_cube_position_;

  // World file paths
  std::string template_world_path_;
  std::string output_world_path_;
};

} // namespace mycobot_stacking_project

#endif // MYCOBOT_STACKING_PROJECT__CUBE_SPAWNER_NODE_HPP_
