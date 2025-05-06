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
  // Yellow pre-grasp pose
  geometry_msgs::msg::PoseStamped yellow_pre_grasp_pose;
  yellow_pre_grasp_pose.header.frame_id = "base_link";
  yellow_pre_grasp_pose.pose.position.x = yellow_cube_pose_.position.x;
  yellow_pre_grasp_pose.pose.position.y = yellow_cube_pose_.position.y;
  yellow_pre_grasp_pose.pose.position.z = yellow_cube_pose_.position.z + APPROACH_DISTANCE;
  yellow_pre_grasp_pose.pose.orientation = grasp_orientation_msg;

  // Yellow grasp pose
  geometry_msgs::msg::Pose yellow_grasp_pose;
  yellow_grasp_pose.position.x = yellow_cube_pose_.position.x;
  yellow_grasp_pose.position.y = yellow_cube_pose_.position.y;
  yellow_grasp_pose.position.z = yellow_cube_pose_.position.z + CUBE_SIZE * 0.5 + 0.005; // Small offset
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
  orange_pre_place_pose.pose.position.z = orange_cube_pose_.position.z + CUBE_SIZE + APPROACH_DISTANCE;
  orange_pre_place_pose.pose.orientation = grasp_orientation_msg;

  // Orange place pose
  geometry_msgs::msg::Pose orange_place_pose;
  orange_place_pose.position.x = orange_cube_pose_.position.x;
  orange_place_pose.position.y = orange_cube_pose_.position.y;
  orange_place_pose.position.z = orange_cube_pose_.position.z + CUBE_SIZE + CUBE_SIZE * 0.5 + 0.005; // Small offset
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

  // Move to yellow pre-grasp
  success &= go_to_pose(yellow_pre_grasp_pose, "Move to Yellow Pre-Grasp");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to move to yellow pre-grasp. Aborting task.");
    return;
  }

  // Approach yellow linearly
  success &= move_cartesian(yellow_grasp_pose, "Approach Yellow Cube");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to approach yellow cube. Aborting task.");
    return;
  }

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

  // Lift yellow cube linearly
  success &= move_cartesian(lift_pose, "Lift Yellow Cube");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to lift yellow cube. Aborting task.");
    return;
  }

  // Move to orange pre-place
  success &= go_to_pose(orange_pre_place_pose, "Move to Orange Pre-Place");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to move to orange pre-place. Aborting task.");
    return;
  }

  // Place yellow cube on orange cube linearly
  success &= move_cartesian(orange_place_pose, "Place Yellow Cube on Orange Cube");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to place yellow cube on orange cube. Aborting task.");
    return;
  }

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

  // Move back to orange pre-place
  success &= move_cartesian(orange_pre_place_pose.pose, "Move back to Orange Pre-Place");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to move back to orange pre-place. Aborting task.");
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
      yellow_cube_pose_ = object_poses[YELLOW_CUBE_ID];
      found_yellow = true;
      RCLCPP_INFO(this->get_logger(), "Found yellow cube at [%.3f, %.3f, %.3f]",
                 yellow_cube_pose_.position.x,
                 yellow_cube_pose_.position.y,
                 yellow_cube_pose_.position.z);
    }

    // Check if orange cube is found
    if (object_poses.find(ORANGE_CUBE_ID) != object_poses.end()) {
      orange_cube_pose_ = object_poses[ORANGE_CUBE_ID];
      found_orange = true;
      RCLCPP_INFO(this->get_logger(), "Found orange cube at [%.3f, %.3f, %.3f]",
                 orange_cube_pose_.position.x,
                 orange_cube_pose_.position.y,
                 orange_cube_pose_.position.z);
    }

    if (found_yellow && found_orange) {
      RCLCPP_INFO(this->get_logger(), "Found both cubes!");
      return true;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return false;
}

bool StackingManagerNode::go_to_named_state(const std::string& state_name)
{
  RCLCPP_INFO(this->get_logger(), "Moving to named state: %s", state_name.c_str());

  arm_group_->setNamedTarget(state_name);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(arm_group_->plan(plan));

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan to named state: %s", state_name.c_str());
    return false;
  }

  success = static_cast<bool>(arm_group_->execute(plan));

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to execute plan to named state: %s", state_name.c_str());
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

  success = static_cast<bool>(arm_group_->execute(plan));

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to execute plan for %s", description.c_str());
    return false;
  }

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

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.01;

  // Use the non-deprecated version of computeCartesianPath (without jump_threshold)
  double fraction = arm_group_->computeCartesianPath(waypoints, eef_step, trajectory);

  if (fraction < 0.9) {
    RCLCPP_ERROR(this->get_logger(), "Failed to compute Cartesian path for %s (%.2f%% achieved)",
                description.c_str(), fraction * 100.0);
    return false;
  }

  bool success = static_cast<bool>(arm_group_->execute(trajectory));

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to execute Cartesian path for %s", description.c_str());
    return false;
  }

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

  success = static_cast<bool>(gripper_group_->execute(plan));

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to execute plan for gripper state: %s", state_name.c_str());
    return false;
  }

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
