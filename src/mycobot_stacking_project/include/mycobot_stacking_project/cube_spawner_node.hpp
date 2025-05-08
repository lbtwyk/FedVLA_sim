/**
 * @file cube_spawner_node.hpp
 * @brief ROS 2 node for spawning colored cubes in the Gazebo simulation.
 *
 * This node spawns yellow and orange cubes in the Gazebo simulation
 * for the perception system to detect.
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
   * @brief Timer callback for spawning cubes.
   */
  void timer_callback();

  // Timer for spawning cubes
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace mycobot_stacking_project

#endif // MYCOBOT_STACKING_PROJECT__CUBE_SPAWNER_NODE_HPP_
