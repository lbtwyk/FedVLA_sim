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

namespace mycobot_stacking_project
{

StackingManagerNode::StackingManagerNode(const rclcpp::NodeOptions & options)
: Node("stacking_manager", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing Stacking Manager Node");

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

  // Wait for the planning scene service to be available
  RCLCPP_INFO(this->get_logger(), "Waiting for planning scene service to be available...");

  // Create a client for the planning scene service
  auto planning_scene_client = node_for_movegroup_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");

  // Wait for the service to be available with a timeout
  const int max_wait_seconds = 60;
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

  // Configure MoveGroup
  arm_group_->setPlanningTime(10.0);
  arm_group_->setNumPlanningAttempts(5);
  arm_group_->setMaxVelocityScalingFactor(0.5);
  arm_group_->setMaxAccelerationScalingFactor(0.5);
  arm_group_->setEndEffectorLink(EEF_LINK);

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

void StackingManagerNode::execute_stacking_task()
{
  RCLCPP_INFO(this->get_logger(), "Starting cube stacking task");

  // Wait for cubes to be detected
  if (!wait_for_cubes()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to detect cubes. Aborting task.");
    return;
  }

  // Define grasping orientation (pointing downwards)
  // Rotate -180 degrees around Y axis (roll=0, pitch=-pi, yaw=0)
  tf2::Quaternion grasp_orientation;
  grasp_orientation.setRPY(0.0, -M_PI, 0.0);
  geometry_msgs::msg::Quaternion grasp_orientation_msg = tf2::toMsg(grasp_orientation);

  // Define target poses
  // Yellow pre-grasp pose - use a higher approach to avoid planning issues
  geometry_msgs::msg::PoseStamped yellow_pre_grasp_pose;
  yellow_pre_grasp_pose.header.frame_id = "base_link";
  yellow_pre_grasp_pose.pose.position.x = yellow_cube_pose_.position.x;
  yellow_pre_grasp_pose.pose.position.y = yellow_cube_pose_.position.y;
  yellow_pre_grasp_pose.pose.position.z = yellow_cube_pose_.position.z + APPROACH_DISTANCE; // Approach from above
  yellow_pre_grasp_pose.pose.orientation = grasp_orientation_msg;

  // Yellow grasp pose
  geometry_msgs::msg::Pose yellow_grasp_pose;
  yellow_grasp_pose.position.x = yellow_cube_pose_.position.x;
  yellow_grasp_pose.position.y = yellow_cube_pose_.position.y;
  yellow_grasp_pose.position.z = yellow_cube_pose_.position.z + CUBE_SIZE * 0.5; // Position at center of cube
  yellow_grasp_pose.orientation = grasp_orientation_msg;

  // Lift pose
  geometry_msgs::msg::Pose lift_pose;
  lift_pose.position.x = yellow_grasp_pose.position.x;
  lift_pose.position.y = yellow_grasp_pose.position.y;
  lift_pose.position.z = yellow_grasp_pose.position.z + LIFT_DISTANCE;
  lift_pose.orientation = grasp_orientation_msg;

  // Orange pre-place pose
  geometry_msgs::msg::PoseStamped orange_pre_place_pose;
  orange_pre_place_pose.header.frame_id = "base_link";
  orange_pre_place_pose.pose.position.x = orange_cube_pose_.position.x;
  orange_pre_place_pose.pose.position.y = orange_cube_pose_.position.y;
  orange_pre_place_pose.pose.position.z = orange_cube_pose_.position.z + CUBE_SIZE + APPROACH_DISTANCE * 0.5; // Approach from above
  orange_pre_place_pose.pose.orientation = grasp_orientation_msg;

  // Orange place pose
  geometry_msgs::msg::Pose orange_place_pose;
  orange_place_pose.position.x = orange_cube_pose_.position.x;
  orange_place_pose.position.y = orange_cube_pose_.position.y;
  orange_place_pose.position.z = orange_cube_pose_.position.z + CUBE_SIZE; // Position on top of the cube
  orange_place_pose.orientation = grasp_orientation_msg;

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

  // Move to yellow pre-grasp using a safer approach
  // First, move to a position above the cube
  geometry_msgs::msg::PoseStamped safe_approach_pose;
  safe_approach_pose.header.frame_id = "base_link";
  safe_approach_pose.pose.position.x = 0.2;  // Fixed position in front of the robot
  safe_approach_pose.pose.position.y = 0.0;  // Centered
  safe_approach_pose.pose.position.z = 0.25; // Safe height
  safe_approach_pose.pose.orientation = grasp_orientation_msg;

  success &= go_to_pose(safe_approach_pose, "Move to Safe Approach Position");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to move to safe approach position. Aborting task.");
    return;
  }

  // Now move to the yellow pre-grasp position
  success &= go_to_pose(yellow_pre_grasp_pose, "Move to Yellow Pre-Grasp");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to move to yellow pre-grasp. Aborting task.");
    return;
  }

  // Use a more robust approach for grasping the yellow cube
  // Instead of trying to go directly to the intermediate pose, use small incremental steps
  double start_z = yellow_pre_grasp_pose.pose.position.z;
  double target_z = yellow_cube_pose_.position.z + CUBE_SIZE * 0.5; // Final grasp height at center of cube
  double step_size = 0.02; // 2cm steps

  // Create a pose for incremental movement
  geometry_msgs::msg::PoseStamped current_pose = yellow_pre_grasp_pose;

  // Move down in small increments
  while (current_pose.pose.position.z > target_z && success) {
    // Calculate next z position
    current_pose.pose.position.z -= step_size;

    // Don't go below the target
    if (current_pose.pose.position.z < target_z) {
      current_pose.pose.position.z = target_z;
    }

    // Move to the next position
    std::string description = "Moving to z=" + std::to_string(current_pose.pose.position.z);
    success &= go_to_pose(current_pose, description);

    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Failed during incremental approach to yellow cube. Aborting task.");
      return;
    }

    // If we've reached the target, break out of the loop
    if (current_pose.pose.position.z <= target_z) {
      break;
    }

    // Small pause between movements
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(this->get_logger(), "Successfully reached yellow grasp position");

  // Close gripper (Grasp)
  success &= set_gripper_state("close");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to close gripper. Aborting task.");
    return;
  }

  // Allow gripper to close
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Attach yellow cube
  success &= attach_cube(YELLOW_CUBE_ID);
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to attach yellow cube. Aborting task.");
    return;
  }

  // Lift yellow cube using regular motion planning instead of Cartesian path
  geometry_msgs::msg::PoseStamped lift_pose_stamped;
  lift_pose_stamped.header.frame_id = "base_link";
  lift_pose_stamped.pose = lift_pose;

  success &= go_to_pose(lift_pose_stamped, "Lift Yellow Cube");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to lift yellow cube. Aborting task.");
    return;
  }

  // Move to orange pre-place using a safer approach
  // First, move to a position above the destination
  geometry_msgs::msg::PoseStamped safe_place_pose;
  safe_place_pose.header.frame_id = "base_link";
  safe_place_pose.pose.position.x = 0.3;  // Position between yellow and orange cube
  safe_place_pose.pose.position.y = 0.0;  // Centered
  safe_place_pose.pose.position.z = 0.25; // Safe height
  safe_place_pose.pose.orientation = grasp_orientation_msg;

  success &= go_to_pose(safe_place_pose, "Move to Safe Place Position");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to move to safe place position. Aborting task.");
    return;
  }

  // Now move to the orange pre-place position
  success &= go_to_pose(orange_pre_place_pose, "Move to Orange Pre-Place");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to move to orange pre-place. Aborting task.");
    return;
  }

  // Use the same incremental approach for placing the yellow cube on the orange cube
  double place_start_z = orange_pre_place_pose.pose.position.z;
  double place_target_z = orange_cube_pose_.position.z + CUBE_SIZE; // Final place height on top of the cube
  double place_step_size = 0.02; // 2cm steps

  // Create a pose for incremental movement
  geometry_msgs::msg::PoseStamped place_current_pose = orange_pre_place_pose;

  // Move down in small increments
  while (place_current_pose.pose.position.z > place_target_z && success) {
    // Calculate next z position
    place_current_pose.pose.position.z -= place_step_size;

    // Don't go below the target
    if (place_current_pose.pose.position.z < place_target_z) {
      place_current_pose.pose.position.z = place_target_z;
    }

    // Move to the next position
    std::string description = "Moving to place position z=" + std::to_string(place_current_pose.pose.position.z);
    success &= go_to_pose(place_current_pose, description);

    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Failed during incremental approach to place position. Aborting task.");
      return;
    }

    // If we've reached the target, break out of the loop
    if (place_current_pose.pose.position.z <= place_target_z) {
      break;
    }

    // Small pause between movements
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(this->get_logger(), "Successfully reached orange place position");

  // Open gripper (Release)
  success &= set_gripper_state("open");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open gripper. Aborting task.");
    return;
  }

  // Allow gripper to open
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Detach yellow cube
  success &= detach_cube(YELLOW_CUBE_ID);
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to detach yellow cube. Aborting task.");
    return;
  }

  // Move back to orange pre-place using regular motion planning instead of Cartesian path
  success &= go_to_pose(orange_pre_place_pose, "Move back to Orange Pre-Place");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to move back to orange pre-place. Aborting task.");
    return;
  }

  // Move to safe position before going home
  geometry_msgs::msg::PoseStamped safe_return_pose;
  safe_return_pose.header.frame_id = "base_link";
  safe_return_pose.pose.position.x = 0.2;  // Fixed position in front of the robot
  safe_return_pose.pose.position.y = 0.0;  // Centered
  safe_return_pose.pose.position.z = 0.25; // Safe height
  safe_return_pose.pose.orientation = grasp_orientation_msg;

  success &= go_to_pose(safe_return_pose, "Move to Safe Return Position");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to move to safe return position. Aborting task.");
    return;
  }

  // Go home
  success &= go_to_named_state("home");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to go home. Aborting task.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Cube stacking task completed successfully");
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
  const int required_consecutive_detections = 3; // Require 3 consecutive detections for stability

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

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return false;
}

bool StackingManagerNode::go_to_named_state(const std::string& state_name)
{
  RCLCPP_INFO(this->get_logger(), "Moving to named state: %s", state_name.c_str());

  // Set the named target
  arm_group_->setNamedTarget(state_name);

  // Plan and execute in a single call
  moveit::core::MoveItErrorCode error_code = arm_group_->move();
  bool success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to move to named state: %s (error code: %i)",
                state_name.c_str(), static_cast<int>(error_code.val));
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Successfully moved to named state: %s", state_name.c_str());
  return true;
}

bool StackingManagerNode::go_to_pose(const geometry_msgs::msg::PoseStamped& target_pose,
                                    const std::string& description)
{
  RCLCPP_INFO(this->get_logger(), "%s: [%.3f, %.3f, %.3f]",
             description.c_str(),
             target_pose.pose.position.x,
             target_pose.pose.position.y,
             target_pose.pose.position.z);

  arm_group_->setPoseTarget(target_pose);

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

bool StackingManagerNode::set_gripper_state(const std::string& state_name)
{
  RCLCPP_INFO(this->get_logger(), "Setting gripper state: %s", state_name.c_str());

  gripper_group_->setNamedTarget(state_name);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(gripper_group_->plan(plan));

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan gripper state: %s", state_name.c_str());
    return false;
  }

  // Use the MoveIt interface to execute the plan
  // Instead of using execute(), we'll use the MoveIt interface's move() method
  // which uses the MoveGroup action interface
  gripper_group_->move();

  RCLCPP_INFO(this->get_logger(), "Successfully set gripper state: %s", state_name.c_str());
  return true;
}

bool StackingManagerNode::attach_cube(const std::string& object_id)
{
  RCLCPP_INFO(this->get_logger(), "Attaching object '%s' to end effector", object_id.c_str());

  arm_group_->attachObject(object_id, EEF_LINK);

  RCLCPP_INFO(this->get_logger(), "Successfully attached object '%s'", object_id.c_str());
  return true;
}

bool StackingManagerNode::detach_cube(const std::string& object_id)
{
  RCLCPP_INFO(this->get_logger(), "Detaching object '%s' from end effector", object_id.c_str());

  arm_group_->detachObject(object_id);

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

  // Wait for the action server to become available
  auto start_time = node_for_movegroup_->now();
  while (rclcpp::ok()) {
    if (arm_client->action_server_is_ready()) {
      RCLCPP_INFO(this->get_logger(), "Arm controller is available!");
      return true;
    }

    auto current_time = node_for_movegroup_->now();
    if ((current_time - start_time).seconds() > timeout_sec) {
      RCLCPP_ERROR(this->get_logger(), "Timeout waiting for arm controller after %.1f seconds", timeout_sec);
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for arm controller... (%.1f seconds elapsed)",
               (current_time - start_time).seconds());
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

  // Wait for the action server to become available
  auto start_time = node_for_movegroup_->now();
  while (rclcpp::ok()) {
    if (gripper_client->action_server_is_ready()) {
      RCLCPP_INFO(this->get_logger(), "Gripper controller is available!");
      return true;
    }

    auto current_time = node_for_movegroup_->now();
    if ((current_time - start_time).seconds() > timeout_sec) {
      RCLCPP_ERROR(this->get_logger(), "Timeout waiting for gripper controller after %.1f seconds", timeout_sec);
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for gripper controller... (%.1f seconds elapsed)",
               (current_time - start_time).seconds());
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return false;
}

bool StackingManagerNode::check_controllers_ready()
{
  RCLCPP_INFO(this->get_logger(), "Checking if controllers are ready...");

  bool arm_ready = wait_for_arm_controller();
  bool gripper_ready = wait_for_gripper_controller();

  if (arm_ready && gripper_ready) {
    RCLCPP_INFO(this->get_logger(), "All controllers are ready!");
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Not all controllers are ready: arm=%s, gripper=%s",
                arm_ready ? "ready" : "not ready",
                gripper_ready ? "ready" : "not ready");
    return false;
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
