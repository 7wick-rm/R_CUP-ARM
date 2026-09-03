// Copyright 2026 Akshit Bhaskara

#include "pcl_geometry/pcl_geometry_node.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <perception_msgs/msg/perception_msg.hpp> 

PclGeometryNode::PclGeometryNode() : Node("pcl_geometry_node")
{
  rclcpp::QoS pointcloud_qos(10);
  pointcloud_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort); 
  pointcloud_qos.durability(rclcpp::DurabilityPolicy::Volatile);

  processing_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  publisher_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  
  rclcpp::SubscriptionOptions processing_cb_options; 
  rclcpp::PublisherOptions publisher_cb_options; 

  processing_cb_options.callback_group = processing_cb_group;
  publisher_cb_options.callback_group = publisher_cb_group;  

  std::string point_cloud_topic = this->declare_parameter("point_cloud_topic", "/rgb_camera/points");
  pc_sub_.subscribe(this, point_cloud_topic, pointcloud_qos.get_rmw_qos_profile(), processing_cb_options);
  
  det_sub_.subscribe(this, "/vision/detected_objects", rmw_qos_profile_default, processing_cb_options);
 
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(50));
  sync_->connectInput(pc_sub_, det_sub_);
  sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));
  sync_->registerCallback(
    std::bind(&PclGeometryNode::syncedCallback, this, std::placeholders::_1, std::placeholders::_2));

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "/debug/brick_poses", 10, publisher_cb_options);
  block_data_pub_ = this->create_publisher<perception_msgs::msg::PerceptionMsg>(
    "/perception/brick_data", 10, publisher_cb_options);
    
  debug_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug/points", 10);
  ransac_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug/ransac", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/obb_boxes", 10);
  
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "PCL Geometry Node Initialized. Waiting for data...");

  // Tunable parameters
  ransac_distance_threshold_ = this->declare_parameter("ransac_distance_threshold", 0.008);
  cluster_tolerance_ = this->declare_parameter("cluster_tolerance", 0.01);
  ransac_max_iterations_ = this->declare_parameter("ransac_max_iterations", 1000);
  ransac_eps_angle_deg_ = this->declare_parameter("ransac_eps_angle_deg", 4.0);
  min_cluster_size_ = this->declare_parameter("min_cluster_size", 30);
  max_cluster_size_ = this->declare_parameter("max_cluster_size", 500);
  margin_px_ = static_cast<float>(this->declare_parameter("margin_px", 8.0));
  min_cloud_size_ = this->declare_parameter("min_cloud_size", 10);

  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&PclGeometryNode::parametersCallback, this, std::placeholders::_1));

  toggle_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "/perception/set_active",
    std::bind(&PclGeometryNode::handleToggle, this, std::placeholders::_1, std::placeholders::_2), 
    rmw_qos_profile_services_default, 
    service_cb_group);
    
  active_ = false; 
  get_place_pose = true; 
  number_of_places = 3; 

  RCLCPP_INFO(this->get_logger(), "MUJOCO_WS VERSION");
}

rcl_interfaces::msg::SetParametersResult PclGeometryNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto & param : parameters)
  {
    if (param.get_name() == "ransac_distance_threshold")
    {
      ransac_distance_threshold_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated RANSAC Threshold: %f", ransac_distance_threshold_);
    }
    else if (param.get_name() == "cluster_tolerance")
    {
      cluster_tolerance_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated Cluster Tolerance: %f", cluster_tolerance_);
    }
    else if (param.get_name() == "ransac_max_iterations")
    {
      ransac_max_iterations_ = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Updated RANSAC Max Iterations: %ld", ransac_max_iterations_);
    }
    else if (param.get_name() == "ransac_eps_angle_deg")
    {
      ransac_eps_angle_deg_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated RANSAC Eps Angle Deg: %f", ransac_eps_angle_deg_);
    }
    else if (param.get_name() == "min_cluster_size")
    {
      min_cluster_size_ = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Updated Min Cluster Size: %ld", min_cluster_size_);
    }
    else if (param.get_name() == "max_cluster_size")
    {
      max_cluster_size_ = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Updated Max Cluster Size: %ld", max_cluster_size_);
    }
    else if (param.get_name() == "margin_px")
    {
      margin_px_ = static_cast<float>(param.as_double());
      RCLCPP_INFO(this->get_logger(), "Updated Margin Px: %f", margin_px_);
    }
    else if (param.get_name() == "min_cloud_size")
    {
      min_cloud_size_ = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Updated Min Cloud Size: %ld", min_cloud_size_);
    }
  }
  return result;
}

geometry_msgs::msg::Pose PclGeometryNode::estimateHybridOrientation(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & brick_cloud, float theta,
  Eigen::Vector3f surface_normal,
  Eigen::Matrix3f rot_matrix)
{
  (void)theta;
  (void)rot_matrix;
  geometry_msgs::msg::Pose pose;

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*brick_cloud, centroid);
  Eigen::Vector3f V_z = surface_normal;

  pcl::PCA<pcl::PointXYZRGB> pca;
  pca.setInputCloud(brick_cloud);
  Eigen::Matrix3f eigen_vectors = pca.getEigenVectors(); 
  
  Eigen::Vector3f V_x_rough = eigen_vectors.col(0);  
  V_z.normalize();

  Eigen::Vector3f V_x = V_x_rough - (V_x_rough.dot(V_z)) * V_z;
  if (V_x.norm() < 1e-4)
  {
    V_x = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  }

  V_x.normalize();

  if (V_x.x() < 0.0)
  {
    V_x = -V_x;
  }

  Eigen::Vector3f V_y = V_z.cross(V_x);
  V_y.normalize();

  Eigen::Matrix3f R;
  R.col(0) = V_x;
  R.col(1) = V_y;
  R.col(2) = V_z;

  Eigen::Quaternionf q(R);

  pose.position.x = centroid(0);
  pose.position.y = centroid(1);
  pose.position.z = centroid(2);

  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  return pose;
}

void PclGeometryNode::syncedCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc_msg,
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & det_msg)
{ 

  if(get_place_pose){ 

  }
  if (!active_) return; 
  RCLCPP_INFO(this->get_logger(), "Callback fired");

  if (det_msg->detections.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Empty detections");
    return;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr optical_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*pc_msg, *optical_cloud);

  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.frame_id = "map";
  pose_array.header.stamp = pc_msg->header.stamp;

  perception_msgs::msg::PerceptionMsg perception_msg;
  perception_msg.block_poses.header.frame_id = "map";
  perception_msg.block_poses.header.stamp = pc_msg->header.stamp;  

  visualization_msgs::msg::MarkerArray marker_array;
  geometry_msgs::msg::TransformStamped optical_to_map;
  try
  {
    optical_to_map =
      tf_buffer_->lookupTransform("map", pc_msg->header.frame_id, tf2::TimePointZero);
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_INFO(this->get_logger(), "Couldn't find transform: %s", ex.what());
    return;
  }
  
  visualization_msgs::msg::Marker delete_all;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_all);

  int marker_id = 0;
  pcl::PointCloud<PointT>::Ptr debug_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr ransac_cloud(new pcl::PointCloud<PointT>);

  for (const auto & det : det_msg->detections)
  {
    float scale_x = static_cast<float>(pc_msg->width) / 1280.0f;
    float scale_y = static_cast<float>(pc_msg->height) / 720.0f;
    float center_x = det.bbox.center.position.x * scale_x;
    float center_y = det.bbox.center.position.y * scale_y;

    float size_x = det.bbox.size_x * scale_x; 
    float size_y = det.bbox.size_y * scale_y; 

    float angle = det.bbox.center.theta * (180.0f / CV_PI);

    cv::RotatedRect rotated_rect(
      cv::Point2f(center_x, center_y), cv::Size2f(size_x, size_y), angle);

    cv::RotatedRect expanded_rect = rotated_rect;
    expanded_rect.size.width += 2 * margin_px_;
    expanded_rect.size.height += 2 * margin_px_;

    cv::Mat mask = cv::Mat::zeros(pc_msg->height, pc_msg->width, CV_8UC1);
    cv::Mat mask_tight = cv::Mat::zeros(pc_msg->height, pc_msg->width, CV_8UC1);

    cv::Point2f vertices_f[4];
    expanded_rect.points(vertices_f);

    cv::Point vertices_i[4];
    for (int i = 0; i < 4; i++)
    {
      vertices_i[i] = vertices_f[i];
    }

    cv::fillConvexPoly(mask, vertices_i, 4, cv::Scalar(255));

    cv::Point2f vertices_f_tight[4];
    rotated_rect.points(vertices_f_tight);
    cv::Point vertices_i_tight[4];
    for (int i = 0; i < 4; i++)
    {
      vertices_i_tight[i] = vertices_f_tight[i];
    }
    cv::fillConvexPoly(mask_tight, vertices_i_tight, 4, cv::Scalar(255));

    cv::Rect bounding_box = expanded_rect.boundingRect(); 

    int min_u = std::max(0, bounding_box.x);
    int max_u = std::min(mask.cols - 1, bounding_box.x + bounding_box.width);
    int min_v = std::max(0, bounding_box.y);
    int max_v = std::min(mask.rows - 1, bounding_box.y + bounding_box.height);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rough_optical_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tight_optical_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int v = min_v; v <= max_v; v++)
    {
      for (int u = min_u; u <= max_u; u++)
      {
        if (mask.at<uchar>(v, u) == 255)
        {
          int memory_index = v * pc_msg->width + u;
          pcl::PointXYZRGB point = optical_cloud->points[memory_index];

          // Explicitly guard against NaN and Inf across all three spatial components
          if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
          {
            rough_optical_cloud->push_back(point);
            if (mask_tight.at<uchar>(v, u) == 255)
            {
              tight_optical_cloud->push_back(point);
            }
          }
        }
      }
    }

    if (rough_optical_cloud->points.size() < static_cast<size_t>(min_cloud_size_))
    {
      RCLCPP_WARN(this->get_logger(), "Point Cloud too small");
      continue;
    }

    Eigen::Affine3d eigen_transform = tf2::transformToEigen(optical_to_map);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*rough_optical_cloud, *transformed_cloud, eigen_transform);

    Eigen::Vector3f reference_point(0.0f, 0.0f, 0.0f);
    bool have_reference_point = false;
    if (!tight_optical_cloud->points.empty())
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_tight_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::transformPointCloud(*tight_optical_cloud, *transformed_tight_cloud, eigen_transform);
      Eigen::Vector4f tight_centroid;
      pcl::compute3DCentroid(*transformed_tight_cloud, tight_centroid);
      reference_point = tight_centroid.head<3>();
      have_reference_point = true;
    }

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(static_cast<int>(ransac_max_iterations_));
    seg.setDistanceThreshold(ransac_distance_threshold_);

    seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    seg.setEpsAngle(ransac_eps_angle_deg_ * (M_PI / 180.0f));

    seg.setInputCloud(transformed_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty() || coefficients->values.size() < 3)
    {
      RCLCPP_WARN(this->get_logger(), "RANSAC failed! Skipping this brick.");
      continue;
    }

    pcl::ExtractIndices<PointT> extract;
    pcl::PointCloud<PointT>::Ptr rough_brick_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr ransac_brick_cloud(new pcl::PointCloud<PointT>);

    extract.setInputCloud(transformed_cloud);
    extract.setIndices(inliers);

    extract.setNegative(true);
    extract.filter(*rough_brick_cloud);

    extract.setNegative(false);
    extract.filter(*ransac_brick_cloud);

    *ransac_cloud += *ransac_brick_cloud;

    if (rough_brick_cloud->points.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Cloud empty after table removal! Skipping.");
      continue;
    }

    // Sanitize non-plane cloud: strip any non-finite coordinates and set is_dense flag for FLANN
    pcl::PointCloud<PointT>::Ptr clean_brick_cloud(new pcl::PointCloud<PointT>);
    clean_brick_cloud->reserve(rough_brick_cloud->size());

    for (const auto & pt : rough_brick_cloud->points)
    {
      if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
      {
        clean_brick_cloud->push_back(pt);
      }
    }

    clean_brick_cloud->width = clean_brick_cloud->points.size();
    clean_brick_cloud->height = 1;
    clean_brick_cloud->is_dense = true;

    if (clean_brick_cloud->points.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Cloud empty after finite filter! Skipping.");
      continue;
    }

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(clean_brick_cloud);

    std::vector<pcl::PointIndices> indices_cluster;

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setInputCloud(clean_brick_cloud);
    ec.setSearchMethod(tree);
    ec.setMinClusterSize(static_cast<int>(min_cluster_size_));
    ec.setMaxClusterSize(static_cast<int>(max_cluster_size_));
    ec.setClusterTolerance(cluster_tolerance_);

    ec.extract(indices_cluster);

    if (indices_cluster.empty())
    {
      continue;
    }

    int best_cluster_idx = 0;
    if (have_reference_point)
    {
      float best_dist_sq = std::numeric_limits<float>::max();
      for (size_t c = 0; c < indices_cluster.size(); ++c)
      {
        Eigen::Vector3f sum(0.0f, 0.0f, 0.0f);
        for (const auto & idx : indices_cluster[c].indices)
        {
          const auto & p = clean_brick_cloud->points[idx];
          sum += Eigen::Vector3f(p.x, p.y, p.z);
        }
        Eigen::Vector3f cluster_centroid =
          sum / static_cast<float>(indices_cluster[c].indices.size());
        float dist_sq = (cluster_centroid - reference_point).squaredNorm();
        if (dist_sq < best_dist_sq)
        {
          best_dist_sq = dist_sq;
          best_cluster_idx = static_cast<int>(c);
        }
      }
    }

    pcl::PointCloud<PointT>::Ptr brick_cloud(new pcl::PointCloud<PointT>);
    for (const auto & idx : indices_cluster[best_cluster_idx].indices)
    {
      brick_cloud->push_back(clean_brick_cloud->points[idx]);
    }
    
    Eigen::Vector3f table_normal{
      coefficients->values[0], coefficients->values[1], coefficients->values[2]};

    float theta = det.bbox.center.theta;
    if (table_normal(2) < 0)
    {
      table_normal = -table_normal;
    }

    RCLCPP_INFO(
      this->get_logger(), "table_normal: [%f, %f, %f]", table_normal.x(), table_normal.y(),
      table_normal.z());

    *debug_cloud += *brick_cloud;
    Eigen::Matrix3f cam_to_world_rot = eigen_transform.rotation().cast<float>(); 
    geometry_msgs::msg::Pose final_pose =
      estimateHybridOrientation(brick_cloud, theta, table_normal, cam_to_world_rot);
    std::string combined_id_string = det.id; 
    size_t dotPos = combined_id_string.find('.');
    perception_msg.block_classes.push_back(combined_id_string.substr(0, dotPos));
    perception_msg.block_ids.push_back(combined_id_string.substr(dotPos + 1));
    perception_msg.block_poses.poses.push_back(final_pose);
    pose_array.poses.push_back(final_pose);
    
    Eigen::Quaternionf q_final(
      final_pose.orientation.w, final_pose.orientation.x, final_pose.orientation.y,
      final_pose.orientation.z);
    Eigen::Matrix3f rotation_matrix = q_final.toRotationMatrix();
    Eigen::Matrix4f inv_transform = Eigen::Matrix4f::Identity();
    inv_transform.block<3, 3>(0, 0) = rotation_matrix.transpose();
    Eigen::Vector3f centroid(final_pose.position.x, final_pose.position.y, final_pose.position.z);
    inv_transform.block<3, 1>(0, 3) = -rotation_matrix.transpose() * centroid;

    pcl::PointCloud<PointT>::Ptr unrotated_cloud(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*brick_cloud, *unrotated_cloud, inv_transform);

    PointT min_pt, max_pt;
    pcl::getMinMax3D(*unrotated_cloud, min_pt, max_pt);

    visualization_msgs::msg::Marker obb_marker;
    obb_marker.header.frame_id = "map";
    obb_marker.header.stamp = pc_msg->header.stamp;
    obb_marker.ns = "obb_boxes";
    obb_marker.id = marker_id++;
    obb_marker.type = visualization_msgs::msg::Marker::CUBE;
    obb_marker.action = visualization_msgs::msg::Marker::ADD;
    obb_marker.pose = final_pose;

    obb_marker.scale.x = std::max(0.001f, max_pt.x - min_pt.x);
    obb_marker.scale.y = std::max(0.001f, max_pt.y - min_pt.y);
    obb_marker.scale.z = std::max(0.001f, max_pt.z - min_pt.z);
    obb_marker.color.r = 0.0;
    obb_marker.color.g = 1.0;
    obb_marker.color.b = 1.0;
    obb_marker.color.a = 0.5;
    marker_array.markers.push_back(obb_marker);
    
    shape_msgs::msg::SolidPrimitive shape; 
    shape.type = shape_msgs::msg::SolidPrimitive::BOX; 
    shape.dimensions.resize(3);
    shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = obb_marker.scale.x;
    shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = obb_marker.scale.y;
    shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = obb_marker.scale.z;

    perception_msg.block_dims.push_back(shape); 
  }

  if (!pose_array.poses.empty())
  {
    pose_pub_->publish(pose_array);
    block_data_pub_->publish(perception_msg);
  }

  marker_pub_->publish(marker_array);

  sensor_msgs::msg::PointCloud2 debug_msg;
  pcl::toROSMsg(*debug_cloud, debug_msg);
  debug_msg.header.frame_id = "map";
  debug_msg.header.stamp = pc_msg->header.stamp;
  debug_cloud_pub_->publish(debug_msg);

  sensor_msgs::msg::PointCloud2 ransac_msg;
  pcl::toROSMsg(*ransac_cloud, ransac_msg);
  ransac_msg.header.frame_id = "map";
  ransac_msg.header.stamp = pc_msg->header.stamp;
  ransac_cloud_pub_->publish(ransac_msg);
}

void PclGeometryNode::handleToggle(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  active_ = request->data; 

  if (active_) {
    RCLCPP_INFO(this->get_logger(), "PCL Node Waking Up");
  }
  else {
    RCLCPP_INFO(this->get_logger(), "PCL Node Going to Sleep"); 
  }

  response->success = true; 
  response->message = active_ ? "Activated" : "Deactivated"; 
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PclGeometryNode>();
  
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}