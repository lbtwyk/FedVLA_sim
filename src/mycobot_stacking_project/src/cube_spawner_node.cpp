/**
 * @file cube_spawner_node.cpp
 * @brief Implementation of the CubeSpawnerNode class.
 *
 * This file contains the implementation of the CubeSpawnerNode class, which
 * spawns yellow and orange cubes in the Gazebo simulation for the perception
 * system to detect.
 *
 * @author Yukun Wang
 */

#include "mycobot_stacking_project/cube_spawner_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace mycobot_stacking_project
{

CubeSpawnerNode::CubeSpawnerNode(const rclcpp::NodeOptions & options)
: Node("cube_spawner", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing Cube Spawner Node");

  // Create a timer to spawn cubes after a delay
  timer_ = this->create_wall_timer(
    5s, std::bind(&CubeSpawnerNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Cube Spawner Node initialized");
}

void CubeSpawnerNode::timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "Adding cubes to planning scene...");

  // Add the yellow cube to the planning scene
  spawn_yellow_cube();

  // Add the orange cube to the planning scene
  spawn_orange_cube();

  // Cancel the timer after adding the cubes
  timer_->cancel();

  RCLCPP_INFO(this->get_logger(), "Cubes added to planning scene successfully");
}

void CubeSpawnerNode::spawn_yellow_cube()
{
  RCLCPP_INFO(this->get_logger(), "Adding yellow cube to planning scene");

  // Create a planning scene interface
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create a collision object for the yellow cube
  moveit_msgs::msg::CollisionObject yellow_cube;
  yellow_cube.header.frame_id = "world";
  yellow_cube.id = "yellow_cube";

  // Define the cube dimensions
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.025; // x
  primitive.dimensions[1] = 0.025; // y
  primitive.dimensions[2] = 0.025; // z

  // Define the cube pose - adjusted for the planning scene
  geometry_msgs::msg::Pose cube_pose;
  cube_pose.position.x = 0.2;
  cube_pose.position.y = 0.15;
  cube_pose.position.z = -0.03;  // Raised by 0.02 from previous -0.05
  cube_pose.orientation.w = 1.0;

  // Add the primitive and pose to the collision object
  yellow_cube.primitives.push_back(primitive);
  yellow_cube.primitive_poses.push_back(cube_pose);
  yellow_cube.operation = yellow_cube.ADD;

  // Add the collision object to the planning scene
  planning_scene_interface.applyCollisionObject(yellow_cube);

  RCLCPP_INFO(this->get_logger(), "Yellow cube added to planning scene at position (%.2f, %.2f, %.2f)",
    cube_pose.position.x, cube_pose.position.y, cube_pose.position.z);
}

void CubeSpawnerNode::spawn_orange_cube()
{
  RCLCPP_INFO(this->get_logger(), "Adding orange cube to planning scene");

  // Create a planning scene interface
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create a collision object for the orange cube
  moveit_msgs::msg::CollisionObject orange_cube;
  orange_cube.header.frame_id = "world";
  orange_cube.id = "orange_cube";

  // Define the cube dimensions
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.025; // x
  primitive.dimensions[1] = 0.025; // y
  primitive.dimensions[2] = 0.025; // z

  // Define the cube pose - adjusted for the planning scene
  geometry_msgs::msg::Pose cube_pose;
  cube_pose.position.x = 0.35;
  cube_pose.position.y = 0.15;
  cube_pose.position.z = -0.03;  // Raised by 0.02 from previous -0.05
  cube_pose.orientation.w = 1.0;

  // Add the primitive and pose to the collision object
  orange_cube.primitives.push_back(primitive);
  orange_cube.primitive_poses.push_back(cube_pose);
  orange_cube.operation = orange_cube.ADD;

  // Add the collision object to the planning scene
  planning_scene_interface.applyCollisionObject(orange_cube);

  RCLCPP_INFO(this->get_logger(), "Orange cube added to planning scene at position (%.2f, %.2f, %.2f)",
    cube_pose.position.x, cube_pose.position.y, cube_pose.position.z);
}

} // namespace mycobot_stacking_project

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mycobot_stacking_project::CubeSpawnerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
