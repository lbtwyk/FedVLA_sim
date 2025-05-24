/**
 * @file cube_spawner_node.cpp
 * @brief Implementation of the CubeSpawnerNode class.
 *
 * This file contains the implementation of the CubeSpawnerNode class, which
 * spawns yellow and orange cubes in the Gazebo simulation for the perception
 * system to detect. It supports randomization of cube positions within
 * specified bounds and ensures cubes don't overlap.
 *
 * @author Yukun Wang
 */

#include "mycobot_stacking_project/cube_spawner_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <cmath>
#include <moveit_msgs/srv/get_planning_scene.hpp>

using namespace std::chrono_literals;

namespace mycobot_stacking_project
{

CubeSpawnerNode::CubeSpawnerNode(const rclcpp::NodeOptions & options)
: Node("cube_spawner", options),
  rng_(std::random_device{}())
{
  RCLCPP_INFO(this->get_logger(), "Initializing Cube Spawner Node");

  // Declare parameters
  // Yellow cube parameters (±0.005 from world position: 0, 0.20)
  this->declare_parameter("yellow_min_x", -0.005);
  this->declare_parameter("yellow_max_x", 0.005);
  this->declare_parameter("yellow_min_y", 0.195);
  this->declare_parameter("yellow_max_y", 0.205);

  // Orange cube parameters (±0.005 from world position: 0.035, 0.25)
  this->declare_parameter("orange_min_x", 0.030);
  this->declare_parameter("orange_max_x", 0.040);
  this->declare_parameter("orange_min_y", 0.245);
  this->declare_parameter("orange_max_y", 0.255);

  // Common parameters
  this->declare_parameter("cube_size", 0.025);
  this->declare_parameter("min_distance_between_cubes", 0.03);
  this->declare_parameter("randomize_positions", true);
  this->declare_parameter("template_world_path", "");
  this->declare_parameter("output_world_path", "");

  // Get parameters
  // Yellow cube parameters
  yellow_min_x_ = this->get_parameter("yellow_min_x").as_double();
  yellow_max_x_ = this->get_parameter("yellow_max_x").as_double();
  yellow_min_y_ = this->get_parameter("yellow_min_y").as_double();
  yellow_max_y_ = this->get_parameter("yellow_max_y").as_double();

  // Orange cube parameters
  orange_min_x_ = this->get_parameter("orange_min_x").as_double();
  orange_max_x_ = this->get_parameter("orange_max_x").as_double();
  orange_min_y_ = this->get_parameter("orange_min_y").as_double();
  orange_max_y_ = this->get_parameter("orange_max_y").as_double();

  // Common parameters
  cube_size_ = this->get_parameter("cube_size").as_double();
  min_distance_between_cubes_ = this->get_parameter("min_distance_between_cubes").as_double();
  randomize_positions_ = this->get_parameter("randomize_positions").as_bool();
  template_world_path_ = this->get_parameter("template_world_path").as_string();
  output_world_path_ = this->get_parameter("output_world_path").as_string();

  RCLCPP_INFO(this->get_logger(), "Yellow cube position bounds: X=[%.2f, %.2f], Y=[%.2f, %.2f]",
              yellow_min_x_, yellow_max_x_, yellow_min_y_, yellow_max_y_);
  RCLCPP_INFO(this->get_logger(), "Orange cube position bounds: X=[%.2f, %.2f], Y=[%.2f, %.2f]",
              orange_min_x_, orange_max_x_, orange_min_y_, orange_max_y_);
  RCLCPP_INFO(this->get_logger(), "Minimum distance between cubes: %.2f", min_distance_between_cubes_);
  RCLCPP_INFO(this->get_logger(), "Randomize positions: %s", randomize_positions_ ? "true" : "false");

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

  // Generate random positions for the cubes
  if (randomize_positions_) {
    // Generate positions with collision checking
    bool valid_positions = false;
    int max_attempts = 100;
    int attempt = 0;

    while (!valid_positions && attempt < max_attempts) {
      // Generate random positions with separate bounds for each cube
      yellow_cube_position_ = generate_random_position(
        yellow_min_x_, yellow_max_x_, yellow_min_y_, yellow_max_y_, cube_size_ / 2.0);

      orange_cube_position_ = generate_random_position(
        orange_min_x_, orange_max_x_, orange_min_y_, orange_max_y_, cube_size_ / 2.0);

      // Check if the positions are valid (no collision)
      if (!check_collision(yellow_cube_position_, orange_cube_position_, min_distance_between_cubes_)) {
        valid_positions = true;
        RCLCPP_INFO(this->get_logger(), "Generated valid cube positions on attempt %d", attempt + 1);
      } else {
        attempt++;
      }
    }

    if (!valid_positions) {
      RCLCPP_WARN(this->get_logger(), "Failed to generate non-colliding positions after %d attempts. Using default positions.", max_attempts);
      // Use default positions based on the middle of the specified ranges
      yellow_cube_position_.x = (yellow_min_x_ + yellow_max_x_) / 2.0;
      yellow_cube_position_.y = (yellow_min_y_ + yellow_max_y_) / 2.0;
      yellow_cube_position_.z = cube_size_ / 2.0;

      orange_cube_position_.x = (orange_min_x_ + orange_max_x_) / 2.0;
      orange_cube_position_.y = (orange_min_y_ + orange_max_y_) / 2.0;
      orange_cube_position_.z = cube_size_ / 2.0;

      // Ensure the default positions don't collide
      if (check_collision(yellow_cube_position_, orange_cube_position_, min_distance_between_cubes_)) {
        // If they would collide, adjust the orange cube position
        orange_cube_position_.x += min_distance_between_cubes_;
        orange_cube_position_.y += min_distance_between_cubes_;
      }
    }
  } else {
    // Use default positions based on the middle of the specified ranges
    yellow_cube_position_.x = (yellow_min_x_ + yellow_max_x_) / 2.0;
    yellow_cube_position_.y = (yellow_min_y_ + yellow_max_y_) / 2.0;
    yellow_cube_position_.z = cube_size_ / 2.0;

    orange_cube_position_.x = (orange_min_x_ + orange_max_x_) / 2.0;
    orange_cube_position_.y = (orange_min_y_ + orange_max_y_) / 2.0;
    orange_cube_position_.z = cube_size_ / 2.0;

    // Ensure the default positions don't collide
    if (check_collision(yellow_cube_position_, orange_cube_position_, min_distance_between_cubes_)) {
      // If they would collide, adjust the orange cube position
      orange_cube_position_.x += min_distance_between_cubes_;
      orange_cube_position_.y += min_distance_between_cubes_;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Yellow cube position: (%.3f, %.3f, %.3f)",
              yellow_cube_position_.x, yellow_cube_position_.y, yellow_cube_position_.z);
  RCLCPP_INFO(this->get_logger(), "Orange cube position: (%.3f, %.3f, %.3f)",
              orange_cube_position_.x, orange_cube_position_.y, orange_cube_position_.z);

  // Generate a world file with the randomized cube positions if paths are provided
  if (!template_world_path_.empty() && !output_world_path_.empty()) {
    if (generate_world_file(template_world_path_, output_world_path_)) {
      RCLCPP_INFO(this->get_logger(), "Generated world file with randomized cube positions: %s",
                 output_world_path_.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate world file with randomized cube positions");
    }
  }

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

  // Exit the node after adding the objects to the planning scene
  // This will trigger the OnProcessExit event handler in the launch file
  RCLCPP_INFO(this->get_logger(), "Exiting cube spawner node to allow Gazebo to start with the generated world file");
  rclcpp::shutdown();
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
  primitive.dimensions[0] = cube_size_; // x
  primitive.dimensions[1] = cube_size_; // y
  primitive.dimensions[2] = cube_size_; // z

  // Define the cube pose - adjusted for the planning scene
  geometry_msgs::msg::Pose cube_pose;
  cube_pose.position.x = yellow_cube_position_.x;
  cube_pose.position.y = yellow_cube_position_.y;
  cube_pose.position.z = yellow_cube_position_.z;
  cube_pose.orientation.w = 1.0;

  // Add the primitive and pose to the collision object
  yellow_cube.primitives.push_back(primitive);
  yellow_cube.primitive_poses.push_back(cube_pose);
  yellow_cube.operation = yellow_cube.ADD;

  // Add the collision object to the planning scene
  planning_scene_interface.applyCollisionObject(yellow_cube);

  RCLCPP_INFO(this->get_logger(), "Yellow cube added to planning scene at position (%.3f, %.3f, %.3f)",
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
  primitive.dimensions[0] = cube_size_; // x
  primitive.dimensions[1] = cube_size_; // y
  primitive.dimensions[2] = cube_size_; // z

  // Define the cube pose - adjusted for the planning scene
  geometry_msgs::msg::Pose cube_pose;
  cube_pose.position.x = orange_cube_position_.x;
  cube_pose.position.y = orange_cube_position_.y;
  cube_pose.position.z = orange_cube_position_.z;
  cube_pose.orientation.w = 1.0;

  // Add the primitive and pose to the collision object
  orange_cube.primitives.push_back(primitive);
  orange_cube.primitive_poses.push_back(cube_pose);
  orange_cube.operation = orange_cube.ADD;

  // Add the collision object to the planning scene
  planning_scene_interface.applyCollisionObject(orange_cube);

  RCLCPP_INFO(this->get_logger(), "Orange cube added to planning scene at position (%.3f, %.3f, %.3f)",
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

geometry_msgs::msg::Point CubeSpawnerNode::generate_random_position(
  double min_x, double max_x, double min_y, double max_y, double z)
{
  // Create distributions for x and y coordinates
  std::uniform_real_distribution<double> x_dist(min_x, max_x);
  std::uniform_real_distribution<double> y_dist(min_y, max_y);

  // Generate random position
  geometry_msgs::msg::Point position;
  position.x = x_dist(rng_);
  position.y = y_dist(rng_);
  position.z = z;

  return position;
}

bool CubeSpawnerNode::check_collision(
  const geometry_msgs::msg::Point& pos1,
  const geometry_msgs::msg::Point& pos2,
  double min_distance)
{
  // Calculate Euclidean distance between the two points
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;
  double distance = std::sqrt(dx*dx + dy*dy);

  // Check if the distance is less than the minimum allowed distance
  return distance < min_distance;
}



bool CubeSpawnerNode::generate_world_file(const std::string& template_world_path, const std::string& output_world_path)
{
  RCLCPP_INFO(this->get_logger(), "Generating world file with randomized cube positions");
  RCLCPP_INFO(this->get_logger(), "Template world file: %s", template_world_path.c_str());
  RCLCPP_INFO(this->get_logger(), "Output world file: %s", output_world_path.c_str());

  try {
    // Check if the template world file exists
    if (!std::filesystem::exists(template_world_path)) {
      RCLCPP_ERROR(this->get_logger(), "Template world file does not exist: %s", template_world_path.c_str());
      return false;
    }

    // Create the output directory if it doesn't exist
    std::filesystem::path output_path(output_world_path);
    std::filesystem::create_directories(output_path.parent_path());

    // Read the template world file
    std::ifstream template_file(template_world_path);
    if (!template_file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open template world file: %s", template_world_path.c_str());
      return false;
    }

    std::string template_content((std::istreambuf_iterator<char>(template_file)),
                                std::istreambuf_iterator<char>());
    template_file.close();

    // Replace the yellow cube position
    std::string yellow_cube_pose_pattern = "<model name=\"yellow_cube\">\n      <pose>";
    size_t yellow_cube_pose_start = template_content.find(yellow_cube_pose_pattern);
    if (yellow_cube_pose_start == std::string::npos) {
      RCLCPP_ERROR(this->get_logger(), "Failed to find yellow cube pose in template world file");
      return false;
    }

    yellow_cube_pose_start += yellow_cube_pose_pattern.length();
    size_t yellow_cube_pose_end = template_content.find("</pose>", yellow_cube_pose_start);
    if (yellow_cube_pose_end == std::string::npos) {
      RCLCPP_ERROR(this->get_logger(), "Failed to find end of yellow cube pose in template world file");
      return false;
    }

    std::string yellow_cube_pose = std::to_string(yellow_cube_position_.x) + " " +
                                  std::to_string(yellow_cube_position_.y) + " " +
                                  std::to_string(yellow_cube_position_.z) + " 0 0 0";

    template_content.replace(yellow_cube_pose_start, yellow_cube_pose_end - yellow_cube_pose_start, yellow_cube_pose);

    // Replace the orange cube position
    std::string orange_cube_pose_pattern = "<model name=\"orange_cube\">\n      <pose>";
    size_t orange_cube_pose_start = template_content.find(orange_cube_pose_pattern);
    if (orange_cube_pose_start == std::string::npos) {
      RCLCPP_ERROR(this->get_logger(), "Failed to find orange cube pose in template world file");
      return false;
    }

    orange_cube_pose_start += orange_cube_pose_pattern.length();
    size_t orange_cube_pose_end = template_content.find("</pose>", orange_cube_pose_start);
    if (orange_cube_pose_end == std::string::npos) {
      RCLCPP_ERROR(this->get_logger(), "Failed to find end of orange cube pose in template world file");
      return false;
    }

    std::string orange_cube_pose = std::to_string(orange_cube_position_.x) + " " +
                                  std::to_string(orange_cube_position_.y) + " " +
                                  std::to_string(orange_cube_position_.z) + " 0 0 0";

    template_content.replace(orange_cube_pose_start, orange_cube_pose_end - orange_cube_pose_start, orange_cube_pose);

    // Write the modified content to the output world file
    std::ofstream output_file(output_world_path);
    if (!output_file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open output world file: %s", output_world_path.c_str());
      return false;
    }

    output_file << template_content;
    output_file.close();

    RCLCPP_INFO(this->get_logger(), "Successfully generated world file with randomized cube positions");
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception while generating world file: %s", e.what());
    return false;
  }
}

} // namespace mycobot_stacking_project

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mycobot_stacking_project::CubeSpawnerNode>();
  rclcpp::spin(node);
  // Note: rclcpp::shutdown() is called in the timer_callback
  return 0;
}
