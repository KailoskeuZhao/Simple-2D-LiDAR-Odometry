#ifndef LIDAR_ODOMETRY_H
#define LIDAR_ODOMETRY_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/registration/icp.h>
#include "lidar_odometry/utils.hpp"

class LidarOdometry
{
    public:
        LidarOdometry(double max_correspondence_distance = 1.0,
                      double transformation_epsilon = 0.001,
                      double maximum_iterations = 1000,
                      double base_to_lidar_x = 0.0,
                      double base_to_lidar_y = 0.0,
                      double base_to_lidar_z = 0.0,
                      double base_to_lidar_roll = 0.0,
                      double base_to_lidar_pitch = 0.0,
                      double base_to_lidar_yaw = 0.0);
        StatePtr get_state();
        void process_scan_data(const ScanDataPtr scan_data);
    private:

        ScanDataPtr last_scan_ptr;
        StatePtr state_ptr;
        Eigen::Matrix4d POSE_G_B; // planar pose in SE(3) form (odom -> base)
        Eigen::Matrix4d BASE_T_LIDAR;

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp;

        void transform_scan_to_base(ScanDataPtr scan);
        bool get_transform_matrix(ScanDataPtr source, ScanDataPtr target, Eigen::Matrix4d &transform);
};

#endif
