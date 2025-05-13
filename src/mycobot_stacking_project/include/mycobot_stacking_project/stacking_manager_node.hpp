/**
 * @file stacking_manager_node.hpp
 * @brief ROS 2 node for managing the cube stacking task using MoveIt.
 *
 * This node coordinates the cube stacking task by controlling the robot arm
 * to pick up a yellow cube and place it on top of an orange cube.
 *
 * @author Yukun Wang
 */

#ifndef MYCOBOT_STACKING_PROJECT__STACKING_MANAGER_NODE_HPP_
#define MYCOBOT_STACKING_PROJECT__STACKING_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <trajectory_data_interfaces/srv/start_stop_episode.hpp>
#include <memory>
#include <string>
#include <thread>
#include <chrono>

namespace mycobot_stacking_project
{

/**
 * @class StackingManagerNode
 * @brief ROS 2 node for managing the cube stacking task.
 */
class StackingManagerNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the StackingManagerNode.
   * @param options Node options.
   */
  explicit StackingManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for the StackingManagerNode.
   */
  ~StackingManagerNode();

private:
  // Constants
  static constexpr const char* ARM_GROUP_NAME = "arm";
  static constexpr const char* GRIPPER_GROUP_NAME = "gripper";
  static constexpr const char* EEF_LINK = "link6_flange";
  static constexpr const char* YELLOW_CUBE_ID = "yellow_cube";
  static constexpr const char* ORANGE_CUBE_ID = "orange_cube";
  static constexpr double APPROACH_DISTANCE = 0.10;
  static constexpr double LIFT_DISTANCE = 0.08;
  static constexpr double CUBE_SIZE = 0.025;

  /**
   * @brief Execute the stacking task.
   */
  void execute_stacking_task();

  /**
   * @brief Wait for cubes to be detected.
   * @param timeout_sec Timeout in seconds.
   * @return True if both cubes were detected, false otherwise.
   */
  bool wait_for_cubes(double timeout_sec = 120.0);

  /**
   * @brief Move the arm to a named state.
   * @param state_name The name of the state to move to.
   * @return True if successful, false otherwise.
   */
  bool go_to_named_state(const std::string& state_name);

  /**
   * @brief Move the arm to a target pose.
   * @param target_pose The target pose.
   * @param description Description of the movement for logging.
   * @return True if successful, false otherwise.
   */
  bool go_to_pose(const geometry_msgs::msg::PoseStamped& target_pose, const std::string& description);

  /**
   * @brief Move the arm in a straight line (Cartesian path).
   * @param target_pose The target pose.
   * @param description Description of the movement for logging.
   * @return True if successful, false otherwise.
   */
  bool move_cartesian(const geometry_msgs::msg::Pose& target_pose, const std::string& description);

  /**
   * @brief Move the arm using a Cartesian path for smoother, more predictable movements.
   * @param target_pose The target pose.
   * @param description Description of the movement for logging.
   * @param jump_threshold The jump threshold for validating the path (0.0 disables it).
   * @param eef_step The end effector step size for Cartesian path generation.
   * @return True if successful, false otherwise.
   */
  bool move_with_cartesian_path(
      const geometry_msgs::msg::PoseStamped& target_pose,
      const std::string& description,
      double jump_threshold = 0.0,
      double eef_step = 0.01);

  /**
   * @brief Set the gripper state (open or close).
   * @param state_name The name of the state to set.
   * @return True if successful, false otherwise.
   */
  bool set_gripper_state(const std::string& state_name);

  /**
   * @brief Attach an object to the end effector.
   * @param object_id The ID of the object to attach.
   * @return True if successful, false otherwise.
   */
  bool attach_cube(const std::string& object_id);

  /**
   * @brief Detach an object from the end effector.
   * @param object_id The ID of the object to detach.
   * @return True if successful, false otherwise.
   */
  bool detach_cube(const std::string& object_id);

  /**
   * @brief Wait for the arm controller to become available.
   * @param timeout_sec Timeout in seconds.
   * @return True if the controller is available, false otherwise.
   */
  bool wait_for_arm_controller(double timeout_sec = 60.0);

  /**
   * @brief Wait for the gripper controller to become available.
   * @param timeout_sec Timeout in seconds.
   * @return True if the controller is available, false otherwise.
   */
  bool wait_for_gripper_controller(double timeout_sec = 60.0);

  /**
   * @brief Check if controllers are ready before starting the task.
   * @return True if all controllers are ready, false otherwise.
   */
  bool check_controllers_ready();

  // Node for MoveGroup context
  std::shared_ptr<rclcpp::Node> node_for_movegroup_;

  // Executor for MoveGroup
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;

  // MoveGroup interfaces
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

  // Planning scene interface
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  // Task execution thread
  std::thread task_thread_;

  // Cube poses
  geometry_msgs::msg::Pose yellow_cube_pose_;
  geometry_msgs::msg::Pose orange_cube_pose_;

  // Data collection service client
  rclcpp::Client<trajectory_data_interfaces::srv::StartStopEpisode>::SharedPtr start_stop_episode_client_;
  std::string current_episode_id_;
  bool data_collection_enabled_;

  /**
   * @brief Generate a unique episode ID based on timestamp.
   * @return A unique episode identifier string.
   */
  std::string generate_episode_id();

  /**
   * @brief Stop data collection for the current episode.
   * @return True if successful, false otherwise.
   */
  bool stop_data_collection();
};

} // namespace mycobot_stacking_project

#endif // MYCOBOT_STACKING_PROJECT__STACKING_MANAGER_NODE_HPP_
