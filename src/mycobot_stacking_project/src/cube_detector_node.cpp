/**
 * @file cube_detector_node.cpp
 * @brief Implementation of the CubeDetectorNode class.
 *
 * This file contains the implementation of the CubeDetectorNode class, which
 * processes point cloud data to detect colored cubes and add them to the MoveIt
 * planning scene.
 *
 * @author Yukun Wang
 */

#include "mycobot_stacking_project/cube_detector_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mycobot_stacking_project
{

CubeDetectorNode::CubeDetectorNode(const rclcpp::NodeOptions & options)
: Node("cube_detector", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing Cube Detector Node");

  // Declare parameters
  this->declare_parameter<double>("voxel_leaf_size", 0.005);
  this->declare_parameter<double>("plane_distance_threshold", 0.01);
  this->declare_parameter<double>("cluster_tolerance", 0.02);
  this->declare_parameter<int>("min_cluster_points", 100);
  this->declare_parameter<int>("max_cluster_points", 10000);
  this->declare_parameter<double>("target_cube_size", 0.04);
  this->declare_parameter<double>("size_tolerance", 0.01);

  // Yellow color range (RGB values 0-1)
  this->declare_parameter<double>("yellow_r_min", 0.7);
  this->declare_parameter<double>("yellow_r_max", 1.0);
  this->declare_parameter<double>("yellow_g_min", 0.7);
  this->declare_parameter<double>("yellow_g_max", 1.0);
  this->declare_parameter<double>("yellow_b_min", 0.0);
  this->declare_parameter<double>("yellow_b_max", 0.3);

  // Orange color range (RGB values 0-1)
  this->declare_parameter<double>("orange_r_min", 0.7);
  this->declare_parameter<double>("orange_r_max", 1.0);
  this->declare_parameter<double>("orange_g_min", 0.3);
  this->declare_parameter<double>("orange_g_max", 0.7);
  this->declare_parameter<double>("orange_b_min", 0.0);
  this->declare_parameter<double>("orange_b_max", 0.3);

  // Get parameters
  voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
  plane_distance_threshold_ = this->get_parameter("plane_distance_threshold").as_double();
  cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
  min_cluster_points_ = this->get_parameter("min_cluster_points").as_int();
  max_cluster_points_ = this->get_parameter("max_cluster_points").as_int();
  target_cube_size_ = this->get_parameter("target_cube_size").as_double();
  size_tolerance_ = this->get_parameter("size_tolerance").as_double();

  RCLCPP_INFO(this->get_logger(), "Loaded parameters: size_tolerance=%.3f", size_tolerance_);

  // Get color range parameters
  yellow_color_range_.r_min = this->get_parameter("yellow_r_min").as_double();
  yellow_color_range_.r_max = this->get_parameter("yellow_r_max").as_double();
  yellow_color_range_.g_min = this->get_parameter("yellow_g_min").as_double();
  yellow_color_range_.g_max = this->get_parameter("yellow_g_max").as_double();
  yellow_color_range_.b_min = this->get_parameter("yellow_b_min").as_double();
  yellow_color_range_.b_max = this->get_parameter("yellow_b_max").as_double();

  orange_color_range_.r_min = this->get_parameter("orange_r_min").as_double();
  orange_color_range_.r_max = this->get_parameter("orange_r_max").as_double();
  orange_color_range_.g_min = this->get_parameter("orange_g_min").as_double();
  orange_color_range_.g_max = this->get_parameter("orange_g_max").as_double();
  orange_color_range_.b_min = this->get_parameter("orange_b_min").as_double();
  orange_color_range_.b_max = this->get_parameter("orange_b_max").as_double();

  // Initialize planning scene interface
  planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  // Initialize TF buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize detected object IDs set
  detected_object_ids_ = std::set<std::string>();

  // Create point cloud subscriber
  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera_head/depth/color/points", 10,
    std::bind(&CubeDetectorNode::pointCloudCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Cube Detector Node initialized and ready");
}

void CubeDetectorNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received point cloud with %d points", msg->width * msg->height);

  // Convert ROS message to PCL point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  try {
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
      return;
    }

    // Check for NaN values in the point cloud
    int nan_count = 0;
    for (const auto& point : *cloud) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        nan_count++;
      }
    }

    if (nan_count > 0) {
      RCLCPP_DEBUG(this->get_logger(), "Point cloud contains %d NaN values out of %ld points",
                  nan_count, cloud->size());
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during point cloud conversion: %s", e.what());
    return;
  }

  // Filter out NaN points before downsampling
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_no_nans(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> nan_indices;

  try {
    pcl::removeNaNFromPointCloud(*cloud, *cloud_no_nans, nan_indices);
    RCLCPP_DEBUG(this->get_logger(), "Removed NaN points: %ld points remaining", cloud_no_nans->size());

    if (cloud_no_nans->empty()) {
      RCLCPP_WARN(this->get_logger(), "After NaN filtering, point cloud is empty");
      return;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during NaN filtering: %s", e.what());
    return;
  }

  // Downsample using VoxelGrid filter
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

  try {
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(cloud_no_nans);
    voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_filter.filter(*cloud_filtered);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during voxel grid filtering: %s", e.what());
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "Downsampled point cloud to %ld points", cloud_filtered->size());

  // Segment plane (table surface) using RANSAC
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  try {
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(plane_distance_threshold_);
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
      RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset");
      return;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during plane segmentation: %s", e.what());
    return;
  }

  // Extract non-plane points (potential objects)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_objects(new pcl::PointCloud<pcl::PointXYZRGB>);

  try {
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_objects);

    if (cloud_objects->empty()) {
      RCLCPP_WARN(this->get_logger(), "No objects found above the plane");
      return;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during object extraction: %s", e.what());
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "Extracted %ld non-plane points", cloud_objects->size());

  // Cluster extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

  // Filter out NaN points before clustering
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_nan(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> indices;

  try {
    pcl::removeNaNFromPointCloud(*cloud_objects, *cloud_filtered_nan, indices);

    // Additional check for NaN values
    for (auto it = cloud_filtered_nan->begin(); it != cloud_filtered_nan->end();) {
      if (!std::isfinite(it->x) || !std::isfinite(it->y) || !std::isfinite(it->z)) {
        it = cloud_filtered_nan->erase(it);
      } else {
        ++it;
      }
    }

    if (cloud_filtered_nan->empty()) {
      RCLCPP_WARN(this->get_logger(), "After NaN filtering, point cloud is empty");
      return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Filtered %ld NaN points, %ld points remaining",
                cloud_objects->size() - cloud_filtered_nan->size(), cloud_filtered_nan->size());

    // Make sure the cloud is organized properly
    cloud_filtered_nan->width = cloud_filtered_nan->size();
    cloud_filtered_nan->height = 1;
    cloud_filtered_nan->is_dense = true;

    tree->setInputCloud(cloud_filtered_nan);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during NaN filtering for clustering: %s", e.what());
    return;
  }

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_points_);
  ec.setMaxClusterSize(max_cluster_points_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered_nan);

  try {
    ec.extract(cluster_indices);

    if (cluster_indices.empty()) {
      RCLCPP_INFO(this->get_logger(), "No clusters found in the point cloud");
      return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Found %ld clusters", cluster_indices.size());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during cluster extraction: %s", e.what());
    return;
  }

  // Process each cluster
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  std::set<std::string> current_frame_object_ids;

  for (const auto& indices : cluster_indices) {
    try {
      // Extract cluster
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

      // Check for valid indices
      for (const auto& idx : indices.indices) {
        if (idx >= 0 && idx < static_cast<int>(cloud_filtered_nan->size())) {
          const auto& point = (*cloud_filtered_nan)[idx];
          if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            cluster->push_back(point);
          }
        }
      }

      // Skip if cluster is empty after filtering
      if (cluster->empty()) {
        RCLCPP_DEBUG(this->get_logger(), "Skipping empty cluster after filtering");
        continue;
      }

      // Make sure the cloud is organized properly
      cluster->width = cluster->size();
      cluster->height = 1;
      cluster->is_dense = true;

      // Compute cluster dimensions
      pcl::PointXYZRGB min_pt, max_pt;
      pcl::getMinMax3D(*cluster, min_pt, max_pt);

      double width = max_pt.x - min_pt.x;
      double height = max_pt.y - min_pt.y;
      double depth = max_pt.z - min_pt.z;

      // Check if dimensions match a cube
      double width_diff = std::abs(width - target_cube_size_);
      double height_diff = std::abs(height - target_cube_size_);
      double depth_diff = std::abs(depth - target_cube_size_);

      // Special case: be more tolerant of height differences
      double height_tolerance = size_tolerance_ * 10.0;  // Much more tolerant for height

      bool is_cube_sized =
        width_diff < size_tolerance_ &&
        height_diff < height_tolerance &&  // Use the larger tolerance for height
        depth_diff < size_tolerance_;

      RCLCPP_INFO(this->get_logger(), "Cluster dimensions: (%.3f, %.3f, %.3f), target: %.3f, tolerance: %.3f, height_tolerance: %.3f",
                width, height, depth, target_cube_size_, size_tolerance_, height_tolerance);
      RCLCPP_INFO(this->get_logger(), "Dimension differences: width: %.3f, height: %.3f, depth: %.3f",
                width_diff, height_diff, depth_diff);

      if (!is_cube_sized) {
        RCLCPP_INFO(this->get_logger(), "Cluster dimensions don't match cube size (exceeds tolerance)");
        continue;
      }

      // Calculate average color
      double r_sum = 0, g_sum = 0, b_sum = 0;
      for (const auto& point : *cluster) {
        r_sum += point.r / 255.0;
        g_sum += point.g / 255.0;
        b_sum += point.b / 255.0;
      }
      double r_avg = r_sum / cluster->size();
      double g_avg = g_sum / cluster->size();
      double b_avg = b_sum / cluster->size();

      RCLCPP_DEBUG(this->get_logger(), "Cluster average color: (%.2f, %.2f, %.2f)", r_avg, g_avg, b_avg);

      // Check if color matches yellow or orange
      RCLCPP_INFO(this->get_logger(), "Checking color: (%.2f, %.2f, %.2f)", r_avg, g_avg, b_avg);

      // Check yellow color range
      RCLCPP_INFO(this->get_logger(), "Yellow range: R(%.2f-%.2f), G(%.2f-%.2f), B(%.2f-%.2f)",
                yellow_color_range_.r_min, yellow_color_range_.r_max,
                yellow_color_range_.g_min, yellow_color_range_.g_max,
                yellow_color_range_.b_min, yellow_color_range_.b_max);

      bool yellow_r_match = r_avg >= yellow_color_range_.r_min && r_avg <= yellow_color_range_.r_max;
      bool yellow_g_match = g_avg >= yellow_color_range_.g_min && g_avg <= yellow_color_range_.g_max;
      bool yellow_b_match = b_avg >= yellow_color_range_.b_min && b_avg <= yellow_color_range_.b_max;
      bool is_yellow = yellow_r_match && yellow_g_match && yellow_b_match;

      // Check orange color range
      RCLCPP_INFO(this->get_logger(), "Orange range: R(%.2f-%.2f), G(%.2f-%.2f), B(%.2f-%.2f)",
                orange_color_range_.r_min, orange_color_range_.r_max,
                orange_color_range_.g_min, orange_color_range_.g_max,
                orange_color_range_.b_min, orange_color_range_.b_max);

      bool orange_r_match = r_avg >= orange_color_range_.r_min && r_avg <= orange_color_range_.r_max;
      bool orange_g_match = g_avg >= orange_color_range_.g_min && g_avg <= orange_color_range_.g_max;
      bool orange_b_match = b_avg >= orange_color_range_.b_min && b_avg <= orange_color_range_.b_max;
      bool is_orange = orange_r_match && orange_g_match && orange_b_match;

      RCLCPP_INFO(this->get_logger(), "Yellow match: R(%s), G(%s), B(%s), Overall: %s",
                yellow_r_match ? "yes" : "no",
                yellow_g_match ? "yes" : "no",
                yellow_b_match ? "yes" : "no",
                is_yellow ? "YES" : "NO");

      RCLCPP_INFO(this->get_logger(), "Orange match: R(%s), G(%s), B(%s), Overall: %s",
                orange_r_match ? "yes" : "no",
                orange_g_match ? "yes" : "no",
                orange_b_match ? "yes" : "no",
                is_orange ? "YES" : "NO");

      std::string object_id;
      if (is_yellow) {
        object_id = "yellow_cube";
        RCLCPP_INFO(this->get_logger(), "Detected yellow cube");
      } else if (is_orange) {
        object_id = "orange_cube";
        RCLCPP_INFO(this->get_logger(), "Detected orange cube");
      } else {
        RCLCPP_INFO(this->get_logger(), "Cluster color doesn't match yellow or orange");
        continue;
      }

      // Compute centroid
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cluster, centroid);

      // Create pose in camera frame
      geometry_msgs::msg::PoseStamped pose_camera_frame;
      pose_camera_frame.header = msg->header;
      pose_camera_frame.pose.position.x = centroid[0];
      pose_camera_frame.pose.position.y = centroid[1];
      pose_camera_frame.pose.position.z = centroid[2];
      pose_camera_frame.pose.orientation.w = 1.0;  // Identity quaternion

      // Transform pose to base_link frame
      geometry_msgs::msg::PoseStamped pose_base_frame;
      try {
        pose_base_frame = tf_buffer_->transform(pose_camera_frame, "base_link");
        RCLCPP_DEBUG(this->get_logger(), "Transformed pose to base_link: [%.3f, %.3f, %.3f]",
                    pose_base_frame.pose.position.x,
                    pose_base_frame.pose.position.y,
                    pose_base_frame.pose.position.z);
      } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        continue;
      }

      // Create collision object
      moveit_msgs::msg::CollisionObject collision_object;
      collision_object.header.frame_id = "base_link";
      collision_object.id = object_id;

      // Define the cube as a box primitive
      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = target_cube_size_;  // x
      primitive.dimensions[1] = target_cube_size_;  // y
      primitive.dimensions[2] = target_cube_size_;  // z

      // Add the primitive and its pose
      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(pose_base_frame.pose);
      collision_object.operation = collision_object.ADD;

      // Add to collision objects list
      collision_objects.push_back(collision_object);
      current_frame_object_ids.insert(object_id);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception during cluster processing: %s", e.what());
      continue;
    }
  }

  // Find objects to remove (objects that were detected before but not in this frame)
  std::set<std::string> objects_to_remove;
  for (const auto& id : detected_object_ids_) {
    if (current_frame_object_ids.find(id) == current_frame_object_ids.end()) {
      objects_to_remove.insert(id);
    }
  }

  // Create collision objects for removal
  for (const auto& id : objects_to_remove) {
    moveit_msgs::msg::CollisionObject remove_object;
    remove_object.id = id;
    remove_object.operation = remove_object.REMOVE;
    collision_objects.push_back(remove_object);
    RCLCPP_INFO(this->get_logger(), "Removing object: %s", id.c_str());
  }

  // Apply collision objects to planning scene
  if (!collision_objects.empty()) {
    planning_scene_interface_->applyCollisionObjects(collision_objects);
    RCLCPP_INFO(this->get_logger(), "Applied %ld collision objects to planning scene",
               collision_objects.size());
  }

  // Update detected object IDs
  detected_object_ids_ = current_frame_object_ids;
}

} // namespace mycobot_stacking_project

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mycobot_stacking_project::CubeDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
