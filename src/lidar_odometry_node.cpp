#include <cstdio>
#include <memory>
#include <string>
#include "builtin_interfaces/msg/time.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lidar_odometry/lidar_odometry.hpp"

class LidarOdometryNode : public rclcpp::Node
{
  public:
    LidarOdometryNode() : Node("lidar_odometry_node")
    {
      RCLCPP_INFO(this->get_logger(), "lidar_odometry_node");

      parameter_initilization();

      double max_correspondence_distance;
      double transformation_epsilon;
      double maximum_iterations;
      double base_to_lidar_x;
      double base_to_lidar_y;
      double base_to_lidar_z;
      double base_to_lidar_roll;
      double base_to_lidar_pitch;
      double base_to_lidar_yaw;
      std::string point_cloud_topic_name;
      std::string legacy_scan_topic_name;
      std::string odom_topic_name;

      this->get_parameter("max_correspondence_distance", max_correspondence_distance);
      this->get_parameter("transformation_epsilon", transformation_epsilon);
      this->get_parameter("maximum_iterations", maximum_iterations);
      this->get_parameter("base_to_lidar_x", base_to_lidar_x);
      this->get_parameter("base_to_lidar_y", base_to_lidar_y);
      this->get_parameter("base_to_lidar_z", base_to_lidar_z);
      this->get_parameter("base_to_lidar_roll", base_to_lidar_roll);
      this->get_parameter("base_to_lidar_pitch", base_to_lidar_pitch);
      this->get_parameter("base_to_lidar_yaw", base_to_lidar_yaw);
      this->get_parameter("point_cloud_topic_name", point_cloud_topic_name);
      this->get_parameter("scan_topic_name", legacy_scan_topic_name);
      this->get_parameter("odom_topic_name", odom_topic_name);
      this->get_parameter("odom_frame_id", odom_frame_id_);
      this->get_parameter("odom_child_frame_id", odom_child_frame_id_);
      this->get_parameter("pose_xy_covariance", pose_xy_covariance_);
      this->get_parameter("pose_yaw_covariance", pose_yaw_covariance_);
      this->get_parameter("twist_xy_covariance", twist_xy_covariance_);
      this->get_parameter("twist_yaw_covariance", twist_yaw_covariance_);

      if (point_cloud_topic_name.empty()) {
        point_cloud_topic_name = legacy_scan_topic_name.empty() ? "lidar/PointCloudFiltered" : legacy_scan_topic_name;
      }

      RCLCPP_INFO(this->get_logger(), "===== Configuration =====");

      RCLCPP_INFO(this->get_logger(), "max_correspondence_distance: %.4f", max_correspondence_distance);
      RCLCPP_INFO(this->get_logger(), "transformation_epsilon: %.4f", transformation_epsilon);
      RCLCPP_INFO(this->get_logger(), "maximum_iterations %.4f", maximum_iterations);
      RCLCPP_INFO(this->get_logger(), "base_to_lidar_x: %.4f", base_to_lidar_x);
      RCLCPP_INFO(this->get_logger(), "base_to_lidar_y: %.4f", base_to_lidar_y);
      RCLCPP_INFO(this->get_logger(), "base_to_lidar_z: %.4f", base_to_lidar_z);
      RCLCPP_INFO(this->get_logger(), "base_to_lidar_roll: %.4f", base_to_lidar_roll);
      RCLCPP_INFO(this->get_logger(), "base_to_lidar_pitch: %.4f", base_to_lidar_pitch);
      RCLCPP_INFO(this->get_logger(), "base_to_lidar_yaw: %.4f", base_to_lidar_yaw);
      RCLCPP_INFO(this->get_logger(), "point_cloud_topic_name: %s", point_cloud_topic_name.c_str());
      RCLCPP_INFO(this->get_logger(), "odom_topic_name: %s", odom_topic_name.c_str());
      RCLCPP_INFO(this->get_logger(), "odom_frame_id: %s", odom_frame_id_.c_str());
      RCLCPP_INFO(this->get_logger(), "odom_child_frame_id: %s",
                  odom_child_frame_id_.empty() ? "<cloud frame>" : odom_child_frame_id_.c_str());
      RCLCPP_INFO(this->get_logger(), "pose_xy_covariance: %.6f", pose_xy_covariance_);
      RCLCPP_INFO(this->get_logger(), "pose_yaw_covariance: %.6f", pose_yaw_covariance_);
      RCLCPP_INFO(this->get_logger(), "twist_xy_covariance: %.6f", twist_xy_covariance_);
      RCLCPP_INFO(this->get_logger(), "twist_yaw_covariance: %.6f", twist_yaw_covariance_);

      lidar_odometry_ptr = std::make_shared<LidarOdometry>(
        max_correspondence_distance,
        transformation_epsilon,
        maximum_iterations,
        base_to_lidar_x,
        base_to_lidar_y,
        base_to_lidar_z,
        base_to_lidar_roll,
        base_to_lidar_pitch,
        base_to_lidar_yaw);

      odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name, 100);
      point_cloud_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        point_cloud_topic_name, 1000, std::bind(&LidarOdometryNode::point_cloud_callback, this, std::placeholders::_1)
      );
    }

    private:
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber;
      std::shared_ptr<LidarOdometry> lidar_odometry_ptr;
      std::string odom_frame_id_;
      std::string odom_child_frame_id_;
      double pose_xy_covariance_;
      double pose_yaw_covariance_;
      double twist_xy_covariance_;
      double twist_yaw_covariance_;

      void parameter_initilization() {
        this->declare_parameter<double>("max_correspondence_distance", 1.0);
        this->declare_parameter<double>("transformation_epsilon", 0.005);
        this->declare_parameter<double>("maximum_iterations", 30);
        this->declare_parameter<double>("base_to_lidar_x", -0.02);
        this->declare_parameter<double>("base_to_lidar_y", 0.0);
        this->declare_parameter<double>("base_to_lidar_z", 0.0);
        this->declare_parameter<double>("base_to_lidar_roll", 0.0);
        this->declare_parameter<double>("base_to_lidar_pitch", -3.1415);
        this->declare_parameter<double>("base_to_lidar_yaw", 0.20);
        this->declare_parameter<std::string>("point_cloud_topic_name", "lidar/PointCloudFiltered");
        this->declare_parameter<std::string>("scan_topic_name", "");
        this->declare_parameter<std::string>("odom_topic_name", "scan_odom");
        this->declare_parameter<std::string>("odom_frame_id", "odom");
        this->declare_parameter<std::string>("odom_child_frame_id", "base_frame");
        this->declare_parameter<double>("pose_xy_covariance", 0.05);
        this->declare_parameter<double>("pose_yaw_covariance", 0.03);
        this->declare_parameter<double>("twist_xy_covariance", 0.10);
        this->declare_parameter<double>("twist_yaw_covariance", 0.05);
      }

      void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg) {
        auto pcl_point_cloud = cloudmsg2cloud(*point_cloud_msg);

        auto scan_data = std::make_shared<ScanData>();
        scan_data->timestamp = point_cloud_msg->header.stamp.sec + point_cloud_msg->header.stamp.nanosec / 1e9;
        scan_data->point_cloud = pcl_point_cloud;

        lidar_odometry_ptr->process_scan_data(scan_data);
        publish_odometry(point_cloud_msg->header.stamp, point_cloud_msg->header.frame_id);
      }

      void publish_odometry(const builtin_interfaces::msg::Time &stamp, const std::string &cloud_frame_id) {
        auto state = lidar_odometry_ptr->get_state();
        std::string child_frame_id = odom_child_frame_id_.empty() ? cloud_frame_id : odom_child_frame_id_;
        if (child_frame_id.empty()) {
          child_frame_id = "lidar_frame";
        }

        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.frame_id = odom_frame_id_;
        odom_msg.child_frame_id = child_frame_id;
        odom_msg.header.stamp = stamp;

        odom_msg.pose.pose = Eigen::toMsg(state->pose);
        odom_msg.twist.twist = Eigen::toMsg(state->velocity);
        set_odometry_covariance(odom_msg);

        odom_publisher->publish(odom_msg);
      }

      void set_odometry_covariance(nav_msgs::msg::Odometry &odom_msg) {
        constexpr double ignored_axis_covariance = 1000.0;

        odom_msg.pose.covariance.fill(0.0);
        odom_msg.pose.covariance[0] = pose_xy_covariance_;
        odom_msg.pose.covariance[7] = pose_xy_covariance_;
        odom_msg.pose.covariance[14] = ignored_axis_covariance;
        odom_msg.pose.covariance[21] = ignored_axis_covariance;
        odom_msg.pose.covariance[28] = ignored_axis_covariance;
        odom_msg.pose.covariance[35] = pose_yaw_covariance_;

        odom_msg.twist.covariance.fill(0.0);
        odom_msg.twist.covariance[0] = twist_xy_covariance_;
        odom_msg.twist.covariance[7] = twist_xy_covariance_;
        odom_msg.twist.covariance[14] = ignored_axis_covariance;
        odom_msg.twist.covariance[21] = ignored_axis_covariance;
        odom_msg.twist.covariance[28] = ignored_axis_covariance;
        odom_msg.twist.covariance[35] = twist_yaw_covariance_;
      }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
