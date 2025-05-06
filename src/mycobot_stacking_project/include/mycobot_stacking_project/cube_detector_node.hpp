/**
 * @file cube_detector_node.hpp
 * @brief ROS 2 node for detecting colored cubes in a point cloud and adding them to the MoveIt planning scene.
 *
 * This node subscribes to a point cloud topic, processes the data to detect yellow and orange cubes,
 * and adds them to the MoveIt planning scene as collision objects.
 *
 * @author Yukun Wang
 */

#ifndef MYCOBOT_STACKING_PROJECT__CUBE_DETECTOR_NODE_HPP_
#define MYCOBOT_STACKING_PROJECT__CUBE_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <set>
#include <string>
#include <memory>

namespace mycobot_stacking_project
{

/**
 * @class CubeDetectorNode
 * @brief ROS 2 node for detecting colored cubes in a point cloud.
 */
class CubeDetectorNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the CubeDetectorNode.
   * @param options Node options.
   */
  explicit CubeDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Callback function for processing point cloud data.
   * @param msg The received point cloud message.
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Subscriber for point cloud data
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;

  // MoveIt planning scene interface
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  // TF buffer and listener for coordinate transformations
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Set of detected object IDs
  std::set<std::string> detected_object_ids_;

  // Parameters
  double voxel_leaf_size_;
  double plane_distance_threshold_;
  double cluster_tolerance_;
  int min_cluster_points_;
  int max_cluster_points_;
  double target_cube_size_;
  double size_tolerance_;

  // Color ranges for cube detection
  struct ColorRange {
    double r_min, r_max;
    double g_min, g_max;
    double b_min, b_max;
  };

  ColorRange yellow_color_range_;
  ColorRange orange_color_range_;
};

} // namespace mycobot_stacking_project

#endif // MYCOBOT_STACKING_PROJECT__CUBE_DETECTOR_NODE_HPP_
