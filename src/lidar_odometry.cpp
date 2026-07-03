
#include "lidar_odometry/lidar_odometry.hpp"

#include <cmath>

#include <pcl/registration/transformation_estimation_2D.h>

namespace {
constexpr std::size_t MIN_SCAN_POINTS = 20;

double yawFromTransform(const Eigen::Matrix4d &transform)
{
    return std::atan2(transform(1, 0), transform(0, 0));
}

Eigen::Matrix4d planarTransform(double x, double y, double yaw)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    transform(0, 0) = c;
    transform(0, 1) = -s;
    transform(1, 0) = s;
    transform(1, 1) = c;
    transform(0, 3) = x;
    transform(1, 3) = y;
    return transform;
}

Eigen::Matrix4d projectToPlanar(const Eigen::Matrix4d &transform)
{
    return planarTransform(transform(0, 3), transform(1, 3), yawFromTransform(transform));
}
}

LidarOdometry::LidarOdometry(double max_correspondence_distance,
                             double transformation_epsilon,
                             double maximum_iterations)
{
    icp = std::make_shared<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>();
    auto transformation_estimation =
        std::make_shared<pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>>();

    icp->setTransformationEstimation(transformation_estimation);
    icp->setMaxCorrespondenceDistance(max_correspondence_distance);
    icp->setTransformationEpsilon(transformation_epsilon);
    icp->setEuclideanFitnessEpsilon(1e-6);
    icp->setMaximumIterations(maximum_iterations);

    POSE_G_B = Eigen::Matrix4d::Identity();

    state_ptr = std::make_shared<State>();
    state_ptr->pose.linear() = POSE_G_B.block<3, 3>(0, 0);
    state_ptr->pose.translation() = POSE_G_B.block<3, 1>(0, 3);
    state_ptr->velocity = Eigen::Matrix<double, 6, 1>::Zero();
}

StatePtr LidarOdometry::get_state()
{
    return state_ptr;
}

void LidarOdometry::process_scan_data(ScanDataPtr scan_ptr)
{
    if (last_scan_ptr) {
        double dt = scan_ptr->timestamp - last_scan_ptr->timestamp;
        Eigen::Matrix4d source_to_target = Eigen::Matrix4d::Identity();
        if (dt <= 0.0 || !get_transform_matrix(last_scan_ptr, scan_ptr, source_to_target)) {
            last_scan_ptr = scan_ptr;
            return;
        }

        const Eigen::Matrix4d delta_base = projectToPlanar(inverseSE3(source_to_target));
        POSE_G_B = projectToPlanar(POSE_G_B * delta_base);

        Eigen::Vector3d translation_velocity = Eigen::Vector3d::Zero();
        translation_velocity.x() = delta_base(0, 3) / dt;
        translation_velocity.y() = delta_base(1, 3) / dt;

        Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
        angular_velocity.z() = yawFromTransform(delta_base) / dt;

        state_ptr->pose.linear() = POSE_G_B.block<3, 3>(0, 0);
        state_ptr->pose.translation() = POSE_G_B.block<3, 1>(0, 3);
        state_ptr->velocity.block<3, 1>(0, 0) = translation_velocity;
        state_ptr->velocity.block<3, 1>(3, 0) = angular_velocity;
    }

    last_scan_ptr = scan_ptr;
}

bool LidarOdometry::get_transform_matrix(ScanDataPtr source, ScanDataPtr target, Eigen::Matrix4d &transform)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);

    if (source->point_cloud.size() < MIN_SCAN_POINTS || target->point_cloud.size() < MIN_SCAN_POINTS) {
        return false;
    }

    icp->setInputSource(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(source->point_cloud));
    icp->setInputTarget(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(target->point_cloud));
    icp->align(*align);

    if (!icp->hasConverged()) {
        return false;
    }

    Eigen::Matrix4f src2tgt = icp->getFinalTransformation();
    transform = projectToPlanar(src2tgt.cast<double>());

    return true;
}
