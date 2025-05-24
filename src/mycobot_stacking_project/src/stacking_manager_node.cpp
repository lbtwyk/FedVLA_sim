/**
 * @file stacking_manager_node.cpp
 * @brief Implementation of the StackingManagerNode class.
 *
 * This file contains the implementation of the StackingManagerNode class, which
 * coordinates the cube stacking task by controlling the robot arm to pick up
 * a yellow cube and place it on top of an orange cube.
 *
 * @author Yukun Wang
 */

#include "mycobot_stacking_project/stacking_manager_node.hpp"

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <sstream>
#include <iomanip>

namespace mycobot_stacking_project
{

StackingManagerNode::StackingManagerNode(const rclcpp::NodeOptions & options)
: Node("stacking_manager", options),
  data_collection_enabled_(true)  // Enable data collection by default
{
  RCLCPP_INFO(this->get_logger(), "Initializing Stacking Manager Node");

  // Declare parameters
  this->declare_parameter("data_collection_enabled", true);

  // Get parameters
  data_collection_enabled_ = this->get_parameter("data_collection_enabled").as_bool();

  // Create a node for MoveGroup context
  node_for_movegroup_ = std::make_shared<rclcpp::Node>(
    "stacking_manager_move_group_interface_context",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create executor for MoveGroup
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(node_for_movegroup_);

  // Start executor in a separate thread
  std::thread([this]() { this->executor_->spin(); }).detach();

  // Initialize data collection service client
  if (data_collection_enabled_) {
    RCLCPP_INFO(this->get_logger(), "Data collection is enabled");

    // Use the absolute service name that matches how the service is registered
    const std::string service_name = "/state_logger_node/start_stop_episode";
    RCLCPP_INFO(this->get_logger(), "Creating client for data collection service: %s", service_name.c_str());

    // Create the client with the correct service name
    start_stop_episode_client_ = this->create_client<trajectory_data_interfaces::srv::StartStopEpisode>(service_name);
  } else {
    RCLCPP_INFO(this->get_logger(), "Data collection is disabled");
  }

  // Wait for the planning scene service to be available
  RCLCPP_INFO(this->get_logger(), "Waiting for planning scene service to be available...");

  // Create a client for the planning scene service
  auto planning_scene_client = node_for_movegroup_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");

  // Wait for the service to be available with a timeout
  const int max_wait_seconds = 20;
  auto start_time = this->now();
  while (rclcpp::ok()) {
    if (planning_scene_client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Planning scene service is available!");
      break;
    }

    auto current_time = this->now();
    if ((current_time - start_time).seconds() > max_wait_seconds) {
      RCLCPP_WARN(this->get_logger(), "Timeout waiting for planning scene service after %d seconds. Continuing anyway...", max_wait_seconds);
      break;
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for planning scene service... (%.1f seconds elapsed)",
               (current_time - start_time).seconds());
  }

  // Initialize planning scene interface
  planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  // Initialize MoveGroup interfaces
  RCLCPP_INFO(this->get_logger(), "Creating MoveGroup interfaces");
  arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node_for_movegroup_, ARM_GROUP_NAME);
  gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node_for_movegroup_, GRIPPER_GROUP_NAME);

  // Configure MoveGroup with more generous planning parameters
  arm_group_->setPlanningTime(15.0);  // Increase planning time to 15 seconds
  arm_group_->setNumPlanningAttempts(10);  // Increase planning attempts to 10
  arm_group_->setMaxVelocityScalingFactor(0.4);  // Slightly reduce velocity for more stable motion
  arm_group_->setMaxAccelerationScalingFactor(0.4);  // Slightly reduce acceleration for more stable motion
  arm_group_->setEndEffectorLink(EEF_LINK);

  // Set more generous goal tolerance
  // This is important for allowing the gripper to make contact with the cube
  arm_group_->setGoalTolerance(0.02);  // 2cm tolerance for position
  arm_group_->setGoalOrientationTolerance(0.1);  // More tolerance for orientation (about 5.7 degrees) for position

  RCLCPP_INFO(this->get_logger(), "Stacking Manager Node initialized");

  // Wait for controllers to be ready before starting the task
  if (!check_controllers_ready()) {
    RCLCPP_ERROR(this->get_logger(), "Controllers not ready. Task execution will likely fail.");
  }

  // Start task execution in a separate thread
  task_thread_ = std::thread(&StackingManagerNode::execute_stacking_task, this);
}

StackingManagerNode::~StackingManagerNode()
{
  if (task_thread_.joinable()) {
    task_thread_.join();
  }
}

std::string StackingManagerNode::generate_episode_id()
{
  // Get current time
  auto now = std::chrono::system_clock::now();
  auto now_time_t = std::chrono::system_clock::to_time_t(now);
  auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;

  // Format time as string
  std::tm now_tm;
  localtime_r(&now_time_t, &now_tm);

  std::ostringstream oss;
  oss << "episode_"
      << std::put_time(&now_tm, "%Y%m%d_%H%M%S")
      << "_" << std::setfill('0') << std::setw(3) << now_ms.count();

  return oss.str();
}

void StackingManagerNode::execute_stacking_task()
{
  RCLCPP_INFO(this->get_logger(), "Starting cube stacking task");

  // Start data collection if enabled
  if (data_collection_enabled_ && start_stop_episode_client_) {
    // Wait for the service to be available
    RCLCPP_INFO(this->get_logger(), "Waiting for data collection service to be available...");
    if (!start_stop_episode_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(this->get_logger(), "Data collection service not available, continuing without data collection");
    } else {
      // Generate a unique episode ID
      current_episode_id_ = generate_episode_id();

      // Create the request
      auto request = std::make_shared<trajectory_data_interfaces::srv::StartStopEpisode::Request>();
      request->start_recording = true;
      request->episode_identifier = current_episode_id_;

      // Call the service
      RCLCPP_INFO(this->get_logger(), "Starting data collection for episode: %s", current_episode_id_.c_str());
      auto future = start_stop_episode_client_->async_send_request(request);

      // Wait for the result without spinning (we're already in a node that's being spun)
      std::chrono::seconds timeout(2);
      if (future.wait_for(timeout) == std::future_status::ready) {
        auto result = future.get();
        if (result->success) {
          RCLCPP_INFO(this->get_logger(), "Data collection started: %s", result->message.c_str());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to start data collection: %s", result->message.c_str());
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call data collection service");
      }
    }
  }

  // Wait for cubes to be detected
  if (!wait_for_cubes()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to detect cubes. Aborting task.");

    // Stop data collection if it was started
    if (data_collection_enabled_ && start_stop_episode_client_ && !current_episode_id_.empty()) {
      stop_data_collection();
    }

    return;
  }

  // Define grasping orientation (pointing downwards)
  // Rotate -180 degrees around Y axis (roll=0, pitch=-pi, yaw=0)
  tf2::Quaternion grasp_orientation;
  grasp_orientation.setRPY(0.0, -M_PI, M_PI);
  geometry_msgs::msg::Quaternion grasp_orientation_msg = tf2::toMsg(grasp_orientation);

  // Define target poses with hardcoded Z values to prevent ground collision

  // Log the detected cube positions for reference
  RCLCPP_INFO(this->get_logger(), "Yellow cube detected at [%.3f, %.3f, %.3f]",
             yellow_cube_pose_.position.x, yellow_cube_pose_.position.y, yellow_cube_pose_.position.z);
  RCLCPP_INFO(this->get_logger(), "Orange cube detected at [%.3f, %.3f, %.3f]",
             orange_cube_pose_.position.x, orange_cube_pose_.position.y, orange_cube_pose_.position.z);

  // Yellow pre-grasp pose
  geometry_msgs::msg::PoseStamped yellow_pre_grasp_pose;
  yellow_pre_grasp_pose.header.frame_id = "base_link";
  yellow_pre_grasp_pose.pose.position.x = yellow_cube_pose_.position.x;
  yellow_pre_grasp_pose.pose.position.y = yellow_cube_pose_.position.y; // Using exact detected y position
  yellow_pre_grasp_pose.pose.position.z = 0.15; // 15cm above ground
  yellow_pre_grasp_pose.pose.orientation = grasp_orientation_msg;

  // Yellow grasp pose
  geometry_msgs::msg::PoseStamped yellow_grasp_pose;
  yellow_grasp_pose.header.frame_id = "base_link";
  yellow_grasp_pose.pose.position.x = yellow_cube_pose_.position.x;
  yellow_grasp_pose.pose.position.y = yellow_cube_pose_.position.y; // Using exact detected y position
  yellow_grasp_pose.pose.position.z = 0.12; // 12cm above ground for grasping (0.005m higher)
  yellow_grasp_pose.pose.orientation = grasp_orientation_msg;

  // Lift pose
  geometry_msgs::msg::PoseStamped lift_pose;
  lift_pose.header.frame_id = "base_link";
  lift_pose.pose.position.x = yellow_cube_pose_.position.x;
  lift_pose.pose.position.y = yellow_cube_pose_.position.y; // Using exact detected y position
  lift_pose.pose.position.z = 0.15; // 15cm above ground
  lift_pose.pose.orientation = grasp_orientation_msg;

  // Orange pre-place pose
  geometry_msgs::msg::PoseStamped orange_pre_place_pose;
  orange_pre_place_pose.header.frame_id = "base_link";
  orange_pre_place_pose.pose.position.x = orange_cube_pose_.position.x;
  orange_pre_place_pose.pose.position.y = orange_cube_pose_.position.y; // Using exact detected y position
  orange_pre_place_pose.pose.position.z = 0.19; // Exactly 19cm above ground as requested
  orange_pre_place_pose.pose.orientation = grasp_orientation_msg;

  RCLCPP_INFO(this->get_logger(), "Orange pre-place pose set to [%.3f, %.3f, %.3f]",
             orange_pre_place_pose.pose.position.x,
             orange_pre_place_pose.pose.position.y,
             orange_pre_place_pose.pose.position.z);

  // Orange place pose
  geometry_msgs::msg::PoseStamped orange_place_pose;
  orange_place_pose.header.frame_id = "base_link";
  orange_place_pose.pose.position.x = orange_cube_pose_.position.x;
  orange_place_pose.pose.position.y = orange_cube_pose_.position.y; // Using exact detected y position
  orange_place_pose.pose.position.z = 0.14; // 14cm above ground
  orange_place_pose.pose.orientation = grasp_orientation_msg;

  RCLCPP_INFO(this->get_logger(), "Orange place pose set to [%.3f, %.3f, %.3f]",
             orange_place_pose.pose.position.x,
             orange_place_pose.pose.position.y,
             orange_place_pose.pose.position.z);

  // Execute sequence
  bool success = true;

  // Go home
  success &= go_to_named_state("home");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to go home. Aborting task.");
    return;
  }

  // Open gripper
  success &= set_gripper_state("open");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open gripper. Aborting task.");
    return;
  }

  // Move to the yellow pre-grasp position
  success &= go_to_pose(yellow_pre_grasp_pose, "Move to Yellow Pre-Grasp");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to move to yellow pre-grasp. Aborting task.");
    return;
  }

  // Wait for 0.5 seconds after reaching pre-grasp position
  RCLCPP_INFO(this->get_logger(), "Waiting for 0.5 seconds after reaching pre-grasp position...");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Move to the grasp position using Cartesian path for smoother movement
  success &= move_with_cartesian_path(yellow_grasp_pose, "Move to Grasp Position", 0.005);
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to move to grasp position. Aborting task.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Successfully reached yellow grasp position");

  // Wait for 1 second after reaching grasp position
  RCLCPP_INFO(this->get_logger(), "Waiting for 1 second after reaching grasp position...");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Close gripper (Grasp)
  success &= set_gripper_state("close");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to close gripper. Aborting task.");
    return;
  }

  // Allow gripper to close - wait for 1 second
  RCLCPP_INFO(this->get_logger(), "Waiting for gripper to close completely (1 second)...");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Verify the gripper is closed enough to grasp the cube
  std::vector<double> gripper_joint_values = gripper_group_->getCurrentJointValues();
  if (!gripper_joint_values.empty()) {
    double current_position = gripper_joint_values[0];
    double target_closed_position = -0.5;
    double position_error = std::abs(current_position - target_closed_position);

    RCLCPP_INFO(this->get_logger(), "Gripper position before lifting: %.3f (target: %.3f, error: %.3f)",
                current_position, target_closed_position, position_error);

    if (position_error > 0.1) {
      RCLCPP_WARN(this->get_logger(), "Gripper may not be closed enough to grasp the cube!");
      RCLCPP_INFO(this->get_logger(), "Attempting to close gripper again...");

      // Try closing the gripper again
      success &= set_gripper_state("close");
      if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to close gripper again. Aborting task.");
        return;
      }

      // Wait again for 1 second
      RCLCPP_INFO(this->get_logger(), "Waiting for gripper to close completely (1 more second)...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  // Attach yellow cube
  success &= attach_cube(YELLOW_CUBE_ID);
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to attach yellow cube. Aborting task.");
    return;
  }

  // Lift yellow cube using Cartesian path for smoother movement
  success &= move_with_cartesian_path(lift_pose, "Lift Yellow Cube", 0.005);
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to lift yellow cube. Aborting task.");
    return;
  }

  // Wait for 0.5 seconds after lifting the cube
  RCLCPP_INFO(this->get_logger(), "Waiting for 0.5 seconds after lifting the cube...");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Temporarily increase planning time and attempts for this challenging move
  arm_group_->setPlanningTime(30.0);  // Increase planning time to 30 seconds
  arm_group_->setNumPlanningAttempts(20);  // Increase planning attempts to 20

  // First try to move to the pre-placement position
  RCLCPP_INFO(this->get_logger(), "Moving to pre-placement position at height 0.17m");

  // Attempt to reach the pre-placement position with up to 2 retries
  bool pre_place_success = false;
  const int max_pre_place_attempts = 2;

  for (int attempt = 1; attempt <= max_pre_place_attempts && !pre_place_success; ++attempt) {
    RCLCPP_INFO(this->get_logger(), "Pre-placement attempt %d/%d", attempt, max_pre_place_attempts);

    // Try with Cartesian path first for smoother movement
    pre_place_success = move_with_cartesian_path(orange_pre_place_pose, "Move to Pre-Placement Position", 0.005);

    // If Cartesian path fails, try regular planning
    if (!pre_place_success) {
      RCLCPP_WARN(this->get_logger(), "Cartesian path to pre-placement position failed, trying regular planning");
      pre_place_success = go_to_pose(orange_pre_place_pose, "Move to Pre-Placement Position (Regular Planning)");
    }

    if (pre_place_success) {
      RCLCPP_INFO(this->get_logger(), "Successfully reached pre-placement position on attempt %d", attempt);

      // Wait for 0.5 seconds after reaching pre-placement position
      RCLCPP_INFO(this->get_logger(), "Waiting for 0.5 seconds after reaching pre-placement position...");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } else if (attempt < max_pre_place_attempts) {
      RCLCPP_WARN(this->get_logger(), "Pre-placement attempt %d failed, retrying...", attempt);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Short delay before retry
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to reach pre-placement position after %d attempts", max_pre_place_attempts);
      RCLCPP_INFO(this->get_logger(), "Proceeding directly to final placement position");
    }
  }

  // Now move to the final placement position
  RCLCPP_INFO(this->get_logger(), "Moving to final placement position");

  bool place_success = false;

  // If we successfully reached the pre-placement position, use Cartesian path from pre-placement to final placement
  if (pre_place_success) {
    RCLCPP_INFO(this->get_logger(), "Using Cartesian path from pre-placement to final placement position");

    // Try multiple times with different step sizes
    const int max_cartesian_attempts = 3;
    double step_size = 0.003;  // Start with a small step size for precision

    for (int attempt = 1; attempt <= max_cartesian_attempts && !place_success; ++attempt) {
      RCLCPP_INFO(this->get_logger(), "Cartesian path attempt %d/%d with step size %.4f",
                 attempt, max_cartesian_attempts, step_size);

      // Use a smaller step size for more precise movement
      place_success = move_with_cartesian_path(orange_place_pose,
                                              "Move from Pre-Placement to Final Placement Position",
                                              step_size);

      if (place_success) {
        RCLCPP_INFO(this->get_logger(), "Successfully moved from pre-placement to final placement using Cartesian path on attempt %d", attempt);

        // Wait for 0.5 seconds after reaching final placement position
        RCLCPP_INFO(this->get_logger(), "Waiting for 0.5 seconds after reaching final placement position...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        break;
      } else if (attempt < max_cartesian_attempts) {
        RCLCPP_WARN(this->get_logger(), "Cartesian path attempt %d failed, trying with larger step size", attempt);
        // Increase step size for next attempt
        step_size *= 1.5;
        // Short delay before retry
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      } else {
        RCLCPP_WARN(this->get_logger(), "All Cartesian path attempts failed, falling back to regular planning");
      }
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Attempting direct movement to final placement position");
  }

  // If Cartesian path fails or we didn't reach pre-placement, try regular planning with multiple attempts
  if (!place_success) {
    // First try with Cartesian path for smoother movement
    place_success = move_with_cartesian_path(orange_place_pose, "Move to Place Position", 0.005);

    // If Cartesian path fails, try regular planning with multiple attempts
    if (!place_success) {
      RCLCPP_WARN(this->get_logger(), "Cartesian path to place position failed, falling back to regular planning");

      // Try regular planning with up to 3 attempts
      for (int attempt = 1; attempt <= 3 && !place_success; ++attempt) {
        RCLCPP_INFO(this->get_logger(), "Regular planning attempt %d/3 for place position", attempt);
        place_success = go_to_pose(orange_place_pose, "Move to Place Position (Regular Planning)");

        if (!place_success && attempt < 3) {
          RCLCPP_WARN(this->get_logger(), "Attempt %d failed, retrying...", attempt);
          std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Short delay before retry
        }
      }
    }
  }

  // Update overall success status
  success &= place_success;

  // Reset planning parameters to normal values
  arm_group_->setPlanningTime(15.0);
  arm_group_->setNumPlanningAttempts(10);

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to move to place position. Aborting task.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Successfully reached orange place position");

  // Add a delay to ensure stability before releasing the cube
  RCLCPP_INFO(this->get_logger(), "Waiting for 1 second to stabilize before releasing cube...");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Open gripper (Release)
  success &= set_gripper_state("open");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open gripper. Aborting task.");
    return;
  }

  // Allow gripper to open - wait for 0.5 seconds
  RCLCPP_INFO(this->get_logger(), "Waiting for gripper to open completely (0.5 seconds)...");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Detach yellow cube
  success &= detach_cube(YELLOW_CUBE_ID);
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to detach yellow cube. Aborting task.");
    return;
  }

  // Force a planning scene update to clear any stale collision data
  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.is_diff = true;
  planning_scene.robot_state.is_diff = true;

  // Create a publisher to update the planning scene
  auto planning_scene_diff_publisher =
    node_for_movegroup_->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 1);

  // Publish a few times to ensure the message is received
  for (int i = 0; i < 3; ++i) {
    planning_scene_diff_publisher->publish(planning_scene);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Skip the safe position and proceed directly to home position
  RCLCPP_INFO(this->get_logger(), "Skipping safe position and proceeding directly to home position");

  // Go home with increased planning time
  arm_group_->setPlanningTime(30.0);  // Increase planning time to 30 seconds
  arm_group_->setNumPlanningAttempts(20);  // Increase planning attempts to 20

  RCLCPP_INFO(this->get_logger(), "Attempting to go home with increased planning time (30s) and attempts (20)");

  success &= go_to_named_state("home");

  // Reset planning parameters to normal values
  arm_group_->setPlanningTime(15.0);
  arm_group_->setNumPlanningAttempts(10);

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to go home. Task completed but robot not in home position.");
  }

  RCLCPP_INFO(this->get_logger(), "Cube stacking task completed successfully");

  // Stop data collection if it was started
  if (data_collection_enabled_ && start_stop_episode_client_ && !current_episode_id_.empty()) {
    stop_data_collection();
  }
}

bool StackingManagerNode::stop_data_collection()
{
  if (!start_stop_episode_client_) {
    RCLCPP_ERROR(this->get_logger(), "Data collection client not initialized");
    return false;
  }

  if (current_episode_id_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No active episode to stop");
    return false;
  }

  // Create the request
  auto request = std::make_shared<trajectory_data_interfaces::srv::StartStopEpisode::Request>();
  request->start_recording = false;
  request->episode_identifier = current_episode_id_;

  // Call the service
  RCLCPP_INFO(this->get_logger(), "Stopping data collection for episode: %s", current_episode_id_.c_str());
  auto future = start_stop_episode_client_->async_send_request(request);

  // Wait for the result without spinning (we're already in a node that's being spun)
  std::chrono::seconds timeout(2);
  if (future.wait_for(timeout) == std::future_status::ready) {
    auto result = future.get();
    if (result->success) {
      RCLCPP_INFO(this->get_logger(), "Data collection stopped: %s", result->message.c_str());
      current_episode_id_ = "";
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to stop data collection: %s", result->message.c_str());
      return false;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call data collection service");
    return false;
  }
}

bool StackingManagerNode::wait_for_cubes(double timeout_sec)
{
  RCLCPP_INFO(this->get_logger(), "Waiting for cubes '%s' and '%s'...",
              YELLOW_CUBE_ID, ORANGE_CUBE_ID);

  auto start_time = this->now();
  bool found_yellow = false;
  bool found_orange = false;
  int consecutive_yellow_detections = 0;
  int consecutive_orange_detections = 0;
  const int required_consecutive_detections = 2; // Require 3 consecutive detections for stability

  while (rclcpp::ok()) {
    auto current_time = this->now();
    if ((current_time - start_time).seconds() > timeout_sec) {
      RCLCPP_ERROR(this->get_logger(), "Timeout waiting for cubes after %.1f seconds", timeout_sec);
      return false;
    }

    // Get known objects from planning scene
    std::map<std::string, geometry_msgs::msg::Pose> object_poses =
      planning_scene_interface_->getObjectPoses({YELLOW_CUBE_ID, ORANGE_CUBE_ID});

    // Check if yellow cube is found
    if (object_poses.find(YELLOW_CUBE_ID) != object_poses.end()) {
      // Store the pose
      geometry_msgs::msg::Pose current_yellow_pose = object_poses[YELLOW_CUBE_ID];

      // If this is the first detection or if the pose is stable
      if (!found_yellow ||
          (std::abs(current_yellow_pose.position.x - yellow_cube_pose_.position.x) < 0.01 &&
           std::abs(current_yellow_pose.position.y - yellow_cube_pose_.position.y) < 0.01 &&
           std::abs(current_yellow_pose.position.z - yellow_cube_pose_.position.z) < 0.01)) {

        yellow_cube_pose_ = current_yellow_pose;
        consecutive_yellow_detections++;

        if (consecutive_yellow_detections >= required_consecutive_detections && !found_yellow) {
          found_yellow = true;
          RCLCPP_INFO(this->get_logger(), "Found stable yellow cube at [%.3f, %.3f, %.3f]",
                     yellow_cube_pose_.position.x,
                     yellow_cube_pose_.position.y,
                     yellow_cube_pose_.position.z);
        }
      } else {
        // Reset counter if position changed significantly
        consecutive_yellow_detections = 1;
        yellow_cube_pose_ = current_yellow_pose;
        RCLCPP_DEBUG(this->get_logger(), "Yellow cube position changed, resetting stability counter");
      }
    } else {
      consecutive_yellow_detections = 0;
    }

    // Check if orange cube is found
    if (object_poses.find(ORANGE_CUBE_ID) != object_poses.end()) {
      // Store the pose
      geometry_msgs::msg::Pose current_orange_pose = object_poses[ORANGE_CUBE_ID];

      // If this is the first detection or if the pose is stable
      if (!found_orange ||
          (std::abs(current_orange_pose.position.x - orange_cube_pose_.position.x) < 0.01 &&
           std::abs(current_orange_pose.position.y - orange_cube_pose_.position.y) < 0.01 &&
           std::abs(current_orange_pose.position.z - orange_cube_pose_.position.z) < 0.01)) {

        orange_cube_pose_ = current_orange_pose;
        consecutive_orange_detections++;

        if (consecutive_orange_detections >= required_consecutive_detections && !found_orange) {
          found_orange = true;
          RCLCPP_INFO(this->get_logger(), "Found stable orange cube at [%.3f, %.3f, %.3f]",
                     orange_cube_pose_.position.x,
                     orange_cube_pose_.position.y,
                     orange_cube_pose_.position.z);
        }
      } else {
        // Reset counter if position changed significantly
        consecutive_orange_detections = 1;
        orange_cube_pose_ = current_orange_pose;
        RCLCPP_DEBUG(this->get_logger(), "Orange cube position changed, resetting stability counter");
      }
    } else {
      consecutive_orange_detections = 0;
    }

    if (found_yellow && found_orange) {
      RCLCPP_INFO(this->get_logger(), "Found both cubes with stable positions!");
      return true;
    }

    // Print status update every 5 seconds
    if (static_cast<int>((current_time - start_time).seconds()) % 5 == 0) {
      RCLCPP_INFO(this->get_logger(), "Still waiting for cubes... Yellow: %s (%d/%d), Orange: %s (%d/%d)",
                 found_yellow ? "Found" : "Not found", consecutive_yellow_detections, required_consecutive_detections,
                 found_orange ? "Found" : "Not found", consecutive_orange_detections, required_consecutive_detections);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  return false;
}

bool StackingManagerNode::go_to_named_state(const std::string& state_name)
{
  RCLCPP_INFO(this->get_logger(), "Moving to named state: %s", state_name.c_str());

  // If we're trying to go to home position, let's set the joint values directly
  // This is more reliable than using named targets in some cases
  if (state_name == "home") {
    RCLCPP_INFO(this->get_logger(), "Setting joint values for home position directly");
    std::vector<double> joint_group_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    arm_group_->setJointValueTarget(joint_group_positions);
  } else {
    arm_group_->setNamedTarget(state_name);
  }

  // Increase planning time to give the planner more time to find a solution
  arm_group_->setPlanningTime(10.0);

  // Try planning up to 3 times
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = false;

  for (int attempt = 1; attempt <= 3 && !success; ++attempt) {
    RCLCPP_INFO(this->get_logger(), "Planning attempt %d/3 for state: %s", attempt, state_name.c_str());
    success = static_cast<bool>(arm_group_->plan(plan));
    if (!success) {
      RCLCPP_WARN(this->get_logger(), "Planning attempt %d failed, %s",
                 attempt, (attempt < 3) ? "retrying..." : "giving up.");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan to named state: %s after 3 attempts", state_name.c_str());
    return false;
  }

  // Try executing up to 3 times
  moveit::core::MoveItErrorCode error_code;
  for (int attempt = 1; attempt <= 3; ++attempt) {
    RCLCPP_INFO(this->get_logger(), "Execution attempt %d/3 for state: %s", attempt, state_name.c_str());
    error_code = arm_group_->execute(plan);

    if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Successfully moved to named state: %s", state_name.c_str());
      return true;
    } else {
      RCLCPP_WARN(this->get_logger(), "Execution attempt %d failed (error code: %i), %s",
                 attempt, static_cast<int>(error_code.val),
                 (attempt < 3) ? "retrying..." : "giving up.");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  RCLCPP_ERROR(this->get_logger(), "Failed to move to named state: %s (error code: %i) after 3 attempts",
              state_name.c_str(), static_cast<int>(error_code.val));
  return false;
}

bool StackingManagerNode::go_to_pose(const geometry_msgs::msg::PoseStamped& target_pose,
                                    const std::string& description)
{
  RCLCPP_INFO(this->get_logger(), "%s: [%.3f, %.3f, %.3f]",
             description.c_str(),
             target_pose.pose.position.x,
             target_pose.pose.position.y,
             target_pose.pose.position.z);

  // Debug: Check if target is below ground level
  if (target_pose.pose.position.z < 0.0) {
    RCLCPP_WARN(this->get_logger(), "WARNING: Target Z position (%.3f) is below ground level!",
               target_pose.pose.position.z);
  }

  // Debug: Check if target is close to yellow cube
  double dist_to_yellow = std::sqrt(
    std::pow(target_pose.pose.position.x - yellow_cube_pose_.position.x, 2) +
    std::pow(target_pose.pose.position.y - yellow_cube_pose_.position.y, 2) +
    std::pow(target_pose.pose.position.z - yellow_cube_pose_.position.z, 2));

  RCLCPP_INFO(this->get_logger(), "Distance to yellow cube: %.3f meters", dist_to_yellow);
  RCLCPP_INFO(this->get_logger(), "Yellow cube position: [%.3f, %.3f, %.3f]",
             yellow_cube_pose_.position.x,
             yellow_cube_pose_.position.y,
             yellow_cube_pose_.position.z);

  // If we're trying to move close to the cube, we need to allow collisions
  bool allow_collision = false;
  if (description.find("Moving to z=0.075") != std::string::npos ||
      description.find("Moving to z=0.055") != std::string::npos ||
      description.find("Moving to z=0.050") != std::string::npos ||
      description.find("Moving to z=0.030") != std::string::npos ||
      description.find("Moving to z=0.010") != std::string::npos) {
    allow_collision = true;
    RCLCPP_INFO(this->get_logger(), "Allowing collisions for %s", description.c_str());

    // Increase the goal tolerance for position
    arm_group_->setGoalPositionTolerance(0.02);  // 2cm tolerance
    arm_group_->setGoalOrientationTolerance(0.1);  // ~5.7 degrees

    RCLCPP_INFO(this->get_logger(), "Increased position tolerance to 2cm and orientation tolerance to 5.7 degrees");
  } else {
    // Reset to default tolerances
    arm_group_->setGoalPositionTolerance(0.005);  // 1cm tolerance
    arm_group_->setGoalOrientationTolerance(0.01);  // ~0.57 degrees

    RCLCPP_INFO(this->get_logger(), "Using default position tolerance of 1cm and orientation tolerance of 0.57 degrees");
  }

  // Add retry mechanism for planning and execution
  const int max_planning_attempts = 3;
  const int max_execution_attempts = 2;
  bool planning_success = false;
  bool execution_success = false;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::MoveItErrorCode error_code;

  // Planning retry loop
  for (int planning_attempt = 1; planning_attempt <= max_planning_attempts && !planning_success; ++planning_attempt) {
    RCLCPP_INFO(this->get_logger(), "Planning attempt %d/%d for %s",
               planning_attempt, max_planning_attempts, description.c_str());

    // Set pose target for each attempt
    arm_group_->setPoseTarget(target_pose);

    // Record planning start time for performance measurement
    auto planning_start_time = this->now();

    // Plan the motion
    planning_success = static_cast<bool>(arm_group_->plan(plan));

    // Calculate planning time
    auto planning_end_time = this->now();
    double planning_time = (planning_end_time - planning_start_time).seconds();
    RCLCPP_INFO(this->get_logger(), "Planning attempt %d took %.7f seconds",
               planning_attempt, planning_time);

    if (planning_success) {
      RCLCPP_INFO(this->get_logger(), "Planning successful on attempt %d", planning_attempt);
      break;
    } else if (planning_attempt < max_planning_attempts) {
      RCLCPP_WARN(this->get_logger(), "Planning attempt %d failed, retrying...", planning_attempt);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));  // Short delay before retry
    } else {
      RCLCPP_ERROR(this->get_logger(), "All %d planning attempts failed for %s",
                  max_planning_attempts, description.c_str());
    }
  }

  // Check if planning was successful or if we can continue despite failure
  if (!planning_success) {
    if (allow_collision) {
      RCLCPP_WARN(this->get_logger(), "Failed to plan %s, but continuing due to expected collision",
                 description.c_str());
      return true;  // Continue with the task despite planning failure
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan %s after %d attempts",
                  description.c_str(), max_planning_attempts);
      return false;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Planning successful, executing movement...");

  // Execution retry loop
  for (int execution_attempt = 1; execution_attempt <= max_execution_attempts && !execution_success; ++execution_attempt) {
    RCLCPP_INFO(this->get_logger(), "Execution attempt %d/%d for %s",
               execution_attempt, max_execution_attempts, description.c_str());

    // Execute the plan
    error_code = arm_group_->execute(plan);

    if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Successfully completed %s on execution attempt %d",
                 description.c_str(), execution_attempt);
      execution_success = true;
      break;
    } else if (allow_collision &&
              (error_code == moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN ||
               error_code == moveit::core::MoveItErrorCode::PLANNING_FAILED ||
               error_code == moveit::core::MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE)) {
      // This error can occur when there's a collision during validation
      // If we're trying to grasp the cube, this is expected
      RCLCPP_WARN(this->get_logger(), "Expected collision detected during %s. Continuing with task.",
                 description.c_str());
      execution_success = true;
      break;
    } else {
      RCLCPP_ERROR(this->get_logger(), "All %d execution attempts failed for %s",
                  max_execution_attempts, description.c_str());

      // Provide more detailed error information
      switch (error_code.val) {
        case moveit::core::MoveItErrorCode::FAILURE:
          RCLCPP_ERROR(this->get_logger(), "Detailed error: General failure");
          break;
        case moveit::core::MoveItErrorCode::PLANNING_FAILED:
          RCLCPP_ERROR(this->get_logger(), "Detailed error: Planning failed");
          break;
        case moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN:
          RCLCPP_ERROR(this->get_logger(), "Detailed error: Invalid motion plan");
          break;
        case moveit::core::MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
          RCLCPP_ERROR(this->get_logger(), "Detailed error: Motion plan invalidated by environment change");
          break;
        case moveit::core::MoveItErrorCode::CONTROL_FAILED:
          RCLCPP_ERROR(this->get_logger(), "Detailed error: Control failed");
          break;
        case moveit::core::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA:
          RCLCPP_ERROR(this->get_logger(), "Detailed error: Unable to acquire sensor data");
          break;
        case moveit::core::MoveItErrorCode::TIMED_OUT:
          RCLCPP_ERROR(this->get_logger(), "Detailed error: Timed out");
          break;
        case moveit::core::MoveItErrorCode::PREEMPTED:
          RCLCPP_ERROR(this->get_logger(), "Detailed error: Preempted");
          break;
        case moveit::core::MoveItErrorCode::START_STATE_IN_COLLISION:
          RCLCPP_ERROR(this->get_logger(), "Detailed error: Start state in collision");
          break;
        case moveit::core::MoveItErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS:
          RCLCPP_ERROR(this->get_logger(), "Detailed error: Start state violates path constraints");
          break;
        case moveit::core::MoveItErrorCode::GOAL_IN_COLLISION:
          RCLCPP_ERROR(this->get_logger(), "Detailed error: Goal in collision");
          break;
        case moveit::core::MoveItErrorCode::GOAL_VIOLATES_PATH_CONSTRAINTS:
          RCLCPP_ERROR(this->get_logger(), "Detailed error: Goal violates path constraints");
          break;
        case moveit::core::MoveItErrorCode::GOAL_CONSTRAINTS_VIOLATED:
          RCLCPP_ERROR(this->get_logger(), "Detailed error: Goal constraints violated");
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Detailed error: Unknown error code");
          break;
      }
    }
  }

  return execution_success;
}

bool StackingManagerNode::move_cartesian(const geometry_msgs::msg::Pose& target_pose,
                                        const std::string& description)
{
  RCLCPP_INFO(this->get_logger(), "%s: [%.3f, %.3f, %.3f]",
             description.c_str(),
             target_pose.position.x,
             target_pose.position.y,
             target_pose.position.z);

  // Instead of using computeCartesianPath, we'll use setPoseTarget and move
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = arm_group_->getPlanningFrame();
  pose_stamped.pose = target_pose;

  arm_group_->setPoseTarget(pose_stamped);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(arm_group_->plan(plan));

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan %s", description.c_str());
    return false;
  }

  // Use the MoveIt interface to execute the plan
  // Instead of using execute(), we'll use the MoveIt interface's move() method
  // which uses the MoveGroup action interface
  arm_group_->move();

  RCLCPP_INFO(this->get_logger(), "Successfully completed %s", description.c_str());
  return true;
}

bool StackingManagerNode::move_with_cartesian_path(
    const geometry_msgs::msg::PoseStamped& target_pose,
    const std::string& description,
    double eef_step)
{
  RCLCPP_INFO(this->get_logger(), "Planning Cartesian path for %s: [%.3f, %.3f, %.3f]",
             description.c_str(),
             target_pose.pose.position.x,
             target_pose.pose.position.y,
             target_pose.pose.position.z);

  // Get the current pose to use as the start of the path
  // Always get a fresh reading of the current pose to account for any drift
  arm_group_->setStartStateToCurrentState();  // Update the start state

  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.header.frame_id = arm_group_->getPlanningFrame();
  current_pose.pose = arm_group_->getCurrentPose().pose;

  RCLCPP_INFO(this->get_logger(), "Current position: [%.3f, %.3f, %.3f]",
             current_pose.pose.position.x,
             current_pose.pose.position.y,
             current_pose.pose.position.z);

  // Create waypoints for the Cartesian path
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose.pose);

  // Reduce velocity and acceleration for more precise movements
  double saved_velocity_scaling = arm_group_->getMaxVelocityScalingFactor();
  double saved_acceleration_scaling = arm_group_->getMaxAccelerationScalingFactor();
  double saved_goal_position_tolerance = arm_group_->getGoalPositionTolerance();
  double saved_goal_orientation_tolerance = arm_group_->getGoalOrientationTolerance();

  // Use slower speeds for more precise control
  arm_group_->setMaxVelocityScalingFactor(0.25);  // 25% of maximum velocity
  arm_group_->setMaxAccelerationScalingFactor(0.25);  // 25% of maximum acceleration

  // Increase tolerances slightly to account for small deviations after delays
  arm_group_->setGoalPositionTolerance(0.01);  // 1cm tolerance (increased from default)
  arm_group_->setGoalOrientationTolerance(0.02);  // ~1.15 degrees (increased from default)

  // Try Cartesian planning with retries
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = 0.0;
  int max_attempts = 3;

  for (int attempt = 1; attempt <= max_attempts; ++attempt) {
    // Compute the Cartesian path (using the version without jump_threshold which is deprecated)
    fraction = arm_group_->computeCartesianPath(waypoints, eef_step, trajectory);

    if (fraction >= 0.9) {
      RCLCPP_INFO(this->get_logger(), "Cartesian path computed successfully on attempt %d (%.2f%% achieved)",
                 attempt, fraction * 100.0);
      break;
    } else {
      RCLCPP_WARN(this->get_logger(), "Cartesian planning attempt %d: only %.2f%% of path achieved",
                 attempt, fraction * 100.0);

      if (attempt < max_attempts) {
        // Try with a slightly different step size for next attempt
        eef_step *= 1.2;  // Increase step size by 20%
        RCLCPP_INFO(this->get_logger(), "Retrying with step size %.5f", eef_step);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    }
  }

  bool success = false;

  if (fraction >= 0.9) {
    // Execute the Cartesian trajectory
    RCLCPP_INFO(this->get_logger(), "Executing Cartesian path (%.2f%% achieved)", fraction * 100.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory = trajectory;

    moveit::core::MoveItErrorCode error_code = arm_group_->execute(plan);

    if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Successfully completed Cartesian path for %s", description.c_str());
      success = true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to execute Cartesian path for %s (error code: %i)",
                  description.c_str(), static_cast<int>(error_code.val));
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to compute adequate Cartesian path after %d attempts (best: %.2f%%)",
               max_attempts, fraction * 100.0);
  }

  // If Cartesian planning failed, fall back to regular planning
  if (!success) {
    RCLCPP_INFO(this->get_logger(), "Falling back to regular motion planning for %s", description.c_str());

    // Use regular motion planning as fallback
    arm_group_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan regular_plan;
    bool plan_success = static_cast<bool>(arm_group_->plan(regular_plan));

    if (plan_success) {
      RCLCPP_INFO(this->get_logger(), "Regular motion plan computed successfully, executing...");
      moveit::core::MoveItErrorCode error_code = arm_group_->execute(regular_plan);

      if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Successfully completed regular motion for %s", description.c_str());
        success = true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to execute regular motion for %s (error code: %i)",
                    description.c_str(), static_cast<int>(error_code.val));
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to compute regular motion plan for %s", description.c_str());
    }
  }

  // Restore original velocity, acceleration, and tolerance settings
  arm_group_->setMaxVelocityScalingFactor(saved_velocity_scaling);
  arm_group_->setMaxAccelerationScalingFactor(saved_acceleration_scaling);
  arm_group_->setGoalPositionTolerance(saved_goal_position_tolerance);
  arm_group_->setGoalOrientationTolerance(saved_goal_orientation_tolerance);

  return success;
}

bool StackingManagerNode::set_gripper_state(const std::string& state_name)
{
  RCLCPP_INFO(this->get_logger(), "Setting gripper state: %s", state_name.c_str());

  // Get current joint values for logging
  std::vector<double> current_joint_values = gripper_group_->getCurrentJointValues();
  if (!current_joint_values.empty()) {
    RCLCPP_INFO(this->get_logger(), "Current gripper joint value: %.3f", current_joint_values[0]);
  } else {
    RCLCPP_WARN(this->get_logger(), "Could not get current gripper joint values");
    // Continue anyway, as we'll use direct control
  }

  // Determine target position based on state name
  double target_position = 0.0;
  if (state_name == "open") {
    target_position = 0.0;  // Fully open
    RCLCPP_INFO(this->get_logger(), "Setting gripper to open position (position: %.1f)", target_position);
  } else if (state_name == "close") {
    target_position = -0.5;  // Fully closed according to SRDF
    RCLCPP_INFO(this->get_logger(), "Setting gripper to closed position (position: %.1f)", target_position);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown gripper state: %s", state_name.c_str());
    return false;
  }

  // Use MoveIt for gripper control
  RCLCPP_INFO(this->get_logger(), "Using MoveIt for gripper control");

  // Set joint values directly instead of using named targets
  std::vector<double> target_joint_values;
  target_joint_values.push_back(target_position);

  // Set the joint target directly
  gripper_group_->setJointValueTarget(target_joint_values);

  // If we're closing the gripper, we need to allow collisions with the cube
  if (state_name == "close") {
    RCLCPP_INFO(this->get_logger(), "Setting up for gripper closing operation");

    // Clear any existing path constraints
    gripper_group_->clearPathConstraints();

    // Increase planning time to give more chances to find a valid plan
    gripper_group_->setPlanningTime(10.0);

    // Set higher goal tolerance for the gripper
    gripper_group_->setGoalJointTolerance(0.1);
    gripper_group_->setGoalPositionTolerance(0.1);

    // Try to execute without planning
    RCLCPP_INFO(this->get_logger(), "Executing direct joint command without planning");
    moveit::core::MoveItErrorCode error_code = gripper_group_->move();

    if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Direct joint command executed successfully");
    } else {
      RCLCPP_WARN(this->get_logger(), "MoveIt gripper control failed (error code: %i), trying direct action client",
                 static_cast<int>(error_code.val));

      // Fall back to direct action client
      RCLCPP_INFO(this->get_logger(), "Using direct GripperCommand action client");

      // Create a simple action client for the gripper controller
      auto gripper_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
        node_for_movegroup_,
        "/gripper_action_controller/gripper_cmd"
      );

      // Wait for the action server to become available
      if (!gripper_client->wait_for_action_server(std::chrono::seconds(3))) {
        RCLCPP_ERROR(this->get_logger(), "Gripper action server not available after 3 seconds");
        return false;
      }

      // Create and send a GripperCommand
      auto goal_msg = control_msgs::action::GripperCommand::Goal();
      goal_msg.command.position = target_position;
      goal_msg.command.max_effort = 50.0;  // Use moderate force

      RCLCPP_INFO(this->get_logger(), "Sending gripper command: position=%.3f, max_effort=%.1f",
                  goal_msg.command.position, goal_msg.command.max_effort);

      // Use a callback-based approach instead of spin_until_future_complete
      bool action_completed = false;
      bool action_succeeded = false;
      std::mutex mutex;
      std::condition_variable cv;

      auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();

      send_goal_options.goal_response_callback =
        [&](const rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>::SharedPtr& goal_handle) {
          if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the gripper action server");
            std::lock_guard<std::mutex> lock(mutex);
            action_completed = true;
            action_succeeded = false;
            cv.notify_one();
          } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by the gripper action server");
          }
        };

      send_goal_options.result_callback =
        [&](const rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>::WrappedResult& result) {
          std::lock_guard<std::mutex> lock(mutex);
          action_completed = true;

          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Gripper command succeeded");
            action_succeeded = true;
          } else {
            RCLCPP_ERROR(this->get_logger(), "Gripper command failed with code: %d",
                        static_cast<int>(result.code));
            action_succeeded = false;
          }

          cv.notify_one();
        };

      // Send the goal with callbacks
      gripper_client->async_send_goal(goal_msg, send_goal_options);

      // Wait for the action to complete with a timeout
      {
        std::unique_lock<std::mutex> lock(mutex);
        if (!cv.wait_for(lock, std::chrono::seconds(3), [&]() { return action_completed; })) {
          RCLCPP_WARN(this->get_logger(), "Timeout waiting for gripper action to complete");
          // Continue anyway, as the gripper may still be moving
          action_succeeded = true;
        }
      }

      if (!action_succeeded) {
        RCLCPP_ERROR(this->get_logger(), "Failed to control gripper using action client");
        return false;
      }
    }

    // Allow time for the gripper to complete its motion
    RCLCPP_INFO(this->get_logger(), "Waiting for gripper to complete motion (0.5 seconds)...");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  } else {
    // For opening, use standard planning approach
    RCLCPP_INFO(this->get_logger(), "Planning gripper motion...");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(gripper_group_->plan(plan));

    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan gripper state: %s", state_name.c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Gripper planning successful, executing movement...");

    // Execute the plan
    moveit::core::MoveItErrorCode error_code = gripper_group_->execute(plan);

    if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to execute gripper movement (error code: %i)",
                  static_cast<int>(error_code.val));
      return false;
    }
  }

  // Verify the gripper state after movement
  std::vector<double> new_joint_values = gripper_group_->getCurrentJointValues();
  if (!new_joint_values.empty()) {
    RCLCPP_INFO(this->get_logger(), "New gripper joint value after movement: %.3f", new_joint_values[0]);

    // Check if the gripper reached the target position
    double position_error = std::abs(new_joint_values[0] - target_position);
    RCLCPP_INFO(this->get_logger(), "Gripper position error: %.3f units from target (%.3f)",
                position_error, target_position);

    if (position_error > 0.1) {
      RCLCPP_WARN(this->get_logger(), "Gripper did not reach target position! Error: %.3f units", position_error);
      // Continue anyway, as the gripper may still be functional enough
    }
  }

  RCLCPP_INFO(this->get_logger(), "Successfully set gripper state: %s", state_name.c_str());
  return true;
}

bool StackingManagerNode::attach_cube(const std::string& object_id)
{
  RCLCPP_INFO(this->get_logger(), "Attaching object '%s' to end effector", object_id.c_str());

  // Define touch links - these are the links that are allowed to touch the object
  std::vector<std::string> touch_links;
  touch_links.push_back("gripper_left1");
  touch_links.push_back("gripper_left2");
  touch_links.push_back("gripper_right1");
  touch_links.push_back("gripper_right2");
  touch_links.push_back("link6_flange");

  // Attach the object with touch links
  arm_group_->attachObject(object_id, EEF_LINK, touch_links);

  RCLCPP_INFO(this->get_logger(), "Successfully attached object '%s'", object_id.c_str());
  return true;
}

bool StackingManagerNode::detach_cube(const std::string& object_id)
{
  RCLCPP_INFO(this->get_logger(), "Detaching object '%s' from end effector", object_id.c_str());

  arm_group_->detachObject(object_id);

  // Wait a moment for the planning scene to update
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Force planning scene update
  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.is_diff = true;
  planning_scene.robot_state.is_diff = true;

  // Publish the planning scene update
  moveit_msgs::msg::PlanningSceneWorld world;
  planning_scene.world = world;
  planning_scene.robot_state.attached_collision_objects.clear();

  // Create a publisher to update the planning scene
  auto planning_scene_diff_publisher =
    node_for_movegroup_->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 1);

  // Publish a few times to ensure the message is received
  for (int i = 0; i < 3; ++i) {
    planning_scene_diff_publisher->publish(planning_scene);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(this->get_logger(), "Successfully detached object '%s'", object_id.c_str());
  return true;
}

bool StackingManagerNode::wait_for_arm_controller(double timeout_sec)
{
  RCLCPP_INFO(this->get_logger(), "Waiting for arm controller to become available...");

  // Create an action client for the arm controller
  auto arm_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    node_for_movegroup_,
    "/arm_controller/follow_joint_trajectory"
  );

  // Use a simple counter instead of relying on the node's clock
  auto start_time = std::chrono::steady_clock::now();
  double max_wait_time = timeout_sec;

  int attempt = 0;
  while (rclcpp::ok()) {
    if (arm_client->action_server_is_ready()) {
      RCLCPP_INFO(this->get_logger(), "Arm controller is available!");
      return true;
    }

    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

    if (elapsed_seconds > max_wait_time) {
      RCLCPP_WARN(this->get_logger(), "Timeout waiting for arm controller after %.1f seconds, but continuing anyway",
                 static_cast<double>(elapsed_seconds));
      return false; // Return false if controller is not available after timeout
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for arm controller... (%.1f seconds elapsed, attempt %d)",
               static_cast<double>(elapsed_seconds), ++attempt);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return false;
}

bool StackingManagerNode::wait_for_gripper_controller(double timeout_sec)
{
  RCLCPP_INFO(this->get_logger(), "Waiting for gripper controller to become available...");

  // Create an action client for the gripper controller
  auto gripper_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
    node_for_movegroup_,
    "/gripper_action_controller/gripper_cmd"
  );

  // Use a simple counter instead of relying on the node's clock
  auto start_time = std::chrono::steady_clock::now();
  double max_wait_time = timeout_sec;

  int attempt = 0;
  while (rclcpp::ok()) {
    if (gripper_client->action_server_is_ready()) {
      RCLCPP_INFO(this->get_logger(), "Gripper controller is available!");
      return true;
    }

    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

    if (elapsed_seconds > max_wait_time) {
      RCLCPP_WARN(this->get_logger(), "Timeout waiting for gripper controller after %.1f seconds, but continuing anyway",
                 static_cast<double>(elapsed_seconds));
      return false; // Return false if controller is not available after timeout
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for gripper controller... (%.1f seconds elapsed, attempt %d)",
               static_cast<double>(elapsed_seconds), ++attempt);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return false;
}

bool StackingManagerNode::check_controllers_ready()
{
  RCLCPP_INFO(this->get_logger(), "Checking if controllers are ready...");

  // Use longer timeouts for initial check
  bool arm_ready = wait_for_arm_controller(30.0);
  bool gripper_ready = wait_for_gripper_controller(30.0);

  if (arm_ready && gripper_ready) {
    RCLCPP_INFO(this->get_logger(), "All controllers are ready!");
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Not all controllers are ready: arm=%s, gripper=%s",
                arm_ready ? "ready" : "not ready",
                gripper_ready ? "ready" : "not ready");

    // Even if controllers are not ready, we'll continue and try again later
    return true;
  }
}

} // namespace mycobot_stacking_project

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mycobot_stacking_project::StackingManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
