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
#include <moveit_msgs/srv/get_planning_scene.hpp>

using namespace std::chrono_literals;

namespace mycobot_stacking_project
{

CubeSpawnerNode::CubeSpawnerNode(const rclcpp::NodeOptions & options)
: Node("cube_spawner", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing Cube Spawner Node");

  // Wait for the planning scene service to be available
  RCLCPP_INFO(this->get_logger(), "Waiting for planning scene service to be available...");

  // Create a client for the planning scene service
  auto planning_scene_client = this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");

  // Wait for the service to be available with a timeout
  const int max_wait_seconds = 60;
  auto start_time = this->now();
  while (rclcpp::ok()) {
    if (planning_scene_client->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "Planning scene service is available!");
      break;
    }

    auto current_time = this->now();
    if ((current_time - start_time).seconds() > max_wait_seconds) {
      RCLCPP_WARN(this->get_logger(), "Timeout waiting for planning scene service after %d seconds. Continuing anyway...", max_wait_seconds);
      break;
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for planning scene service to become available... (%.1f seconds elapsed)",
               (current_time - start_time).seconds());
  }

  RCLCPP_INFO(this->get_logger(), "Planning scene service is available.");

  // Create a timer to spawn cubes after a shorter delay
  timer_ = this->create_wall_timer(
    2s, std::bind(&CubeSpawnerNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Cube Spawner Node initialized");
}

void CubeSpawnerNode::timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "Adding objects to planning scene...");

  // Add the ground plane to the planning scene
  spawn_ground_plane();

  // Add the yellow cube to the planning scene
  spawn_yellow_cube();

  // Add the orange cube to the planning scene
  spawn_orange_cube();

  // Cancel the timer after adding the objects
  timer_->cancel();

  RCLCPP_INFO(this->get_logger(), "Objects added to planning scene successfully");
}

void CubeSpawnerNode::spawn_yellow_cube()
{
  RCLCPP_INFO(this->get_logger(), "Adding yellow cube to planning scene");

  // Create a planning scene interface
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create a collision object for the yellow cube
  moveit_msgs::msg::CollisionObject yellow_cube;
  yellow_cube.header.frame_id = "base_link";
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
  cube_pose.position.z = 0.0125;  // Half the height of the cube (0.025/2)
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
  orange_cube.header.frame_id = "base_link";
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
  cube_pose.position.x = 0.15;
  cube_pose.position.y = 0.2;
  cube_pose.position.z = 0.0125;  // Half the height of the cube (0.025/2)
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

void CubeSpawnerNode::spawn_ground_plane()
{
  RCLCPP_INFO(this->get_logger(), "Adding ground plane to planning scene");

  // Create a planning scene interface
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create a collision object for the ground plane
  moveit_msgs::msg::CollisionObject ground_plane;
  ground_plane.header.frame_id = "base_link";
  ground_plane.id = "ground_plane";

  // Define the ground plane dimensions (large flat box)
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 2.0;  // x - 2 meters wide
  primitive.dimensions[1] = 2.0;  // y - 2 meters long
  primitive.dimensions[2] = 0.002; // z - 0.2 cm thick

  // Define the ground plane pose
  geometry_msgs::msg::Pose plane_pose;
  plane_pose.position.x = 0.0;    // Centered at origin
  plane_pose.position.y = 0.0;    // Centered at origin
  plane_pose.position.z = -0.005; // Half the thickness below z=0
  plane_pose.orientation.w = 1.0;

  // Add the primitive and pose to the collision object
  ground_plane.primitives.push_back(primitive);
  ground_plane.primitive_poses.push_back(plane_pose);
  ground_plane.operation = ground_plane.ADD;

  // Add the collision object to the planning scene
  planning_scene_interface.applyCollisionObject(ground_plane);

  RCLCPP_INFO(this->get_logger(), "Ground plane added to planning scene at position (%.2f, %.2f, %.2f)",
    plane_pose.position.x, plane_pose.position.y, plane_pose.position.z);
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
