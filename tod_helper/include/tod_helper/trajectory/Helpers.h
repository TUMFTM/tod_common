// Copyright 2021 Schimpe
#pragma once
#include <nav_msgs/Path.h>
#include <tod_msgs/Trajectory.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tod_helper/geometry/Helpers.h>

namespace tod_helper::Trajectory {

inline static nav_msgs::Path to_path(const tod_msgs::Trajectory &trajectory,
                                     const bool desiredVelocityAsZCoordinate = false) {
    nav_msgs::Path path;
    path.header = trajectory.header;
    for (const auto &pt : trajectory.points) {
        path.poses.emplace_back(pt.pose);
        if (desiredVelocityAsZCoordinate)
            path.poses.back().pose.position.z = pt.twist.twist.linear.x;
    }
    return path;
}

inline bool is_point_ahead_in_x_dir(const geometry_msgs::Point& point, const geometry_msgs::Pose& pose) {
    return tod_helper::Geometry::calc_relative_position(point, pose).x > -0.1;
}

inline std::vector<float> get_distance_of_trajectory_points(const tod_msgs::TrajectoryConstPtr trajectory,
                                                            const geometry_msgs::Point& point) {
    std::vector<float> distances;
    for (const auto &traj_point : trajectory->points) {
        distances.push_back(std::abs(
            tod_helper::Geometry::calc_horizontal_distance(traj_point.pose.pose.position, point)));
    }
    return distances;
}

inline int get_closest_trajectory_point(const tod_msgs::TrajectoryConstPtr& trajectory,
                                        const geometry_msgs::Pose pose) {
    if (trajectory->points.size() == 0 ) {return -1; }
    std::vector<float> distances = get_distance_of_trajectory_points(trajectory, pose.position);
    return std::distance(distances.begin(), std::min_element(distances.begin(), distances.end()));
}

inline void get_trajectory_in_child_frame_2d(const tod_msgs::TrajectoryConstPtr traj_parent,
                                             const geometry_msgs::TransformStamped& transformToChild,
                                             tod_msgs::TrajectoryConstPtr traj_child) {
    std::cout << "Implement get_trajectory_in_child_frame_2d in VehicleModelHelpers"  << std::endl;
}

inline int get_closest_trajectory_point_in_x_dir(const tod_msgs::TrajectoryConstPtr& trajectory,
                                                 const geometry_msgs::Pose& pose) {
    if (trajectory->points.size() == 0 ) { return -1; }
    std::vector<float> distances = get_distance_of_trajectory_points(trajectory, pose.position);
    auto firstPointAhead = std::find_if(trajectory->points.begin(), trajectory->points.end(),
                                        [&pose](const auto &point) {
                                            return is_point_ahead_in_x_dir(point.pose.pose.position, pose);
                                        });

    if (firstPointAhead == trajectory->points.end()) {
        return -1;
    }

    // get min distance of all points ahead
    return std::distance(
        distances.begin(), std::min_element(
                               distances.begin()
                                   + std::distance(trajectory->points.begin(), firstPointAhead), distances.end()));
}

}; // namespace tod_helper::Trajectory
