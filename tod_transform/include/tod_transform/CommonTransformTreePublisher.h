// Copyright 2020 Feiler
#pragma once

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <vector>

class CommonTransformTreePublisher {
public:
    static std::vector<geometry_msgs::TransformStamped> getTransformTreeFromParamServer(
        const std::string& transformTreePublisherNodeName, ros::NodeHandle& nodeHandle)  {
        std::vector<geometry_msgs::TransformStamped> vec_transformStamped;
        int maximum_amount_of_considered_transforms{ 30 };

        for (int it=0; it != maximum_amount_of_considered_transforms; ++it) {
            std::string base{ transformTreePublisherNodeName + "/Transform" + std::to_string(it) };
            std::string from;
            if (nodeHandle.getParam(std::string(base + "/from"), from)) {
                geometry_msgs::TransformStamped transformStamped;
                transformStamped.header.stamp = ros::Time::now();
                transformStamped.header.frame_id = from;
                if (!nodeHandle.getParam(std::string(base + "/to"),
                                         transformStamped.child_frame_id))
                        ROS_WARN("Did not get /to at %s", base.c_str());
                std::vector<double> translation_x_y_z;
                if (!nodeHandle.getParam(std::string(base + "/translation_x_y_z"),
                                         translation_x_y_z)) {
                    ROS_WARN("Did not get translation at %s", base.c_str());
                } else {
                    transformStamped.transform.translation.x = translation_x_y_z.at(0);
                    transformStamped.transform.translation.y = translation_x_y_z.at(1);
                    transformStamped.transform.translation.z = translation_x_y_z.at(2);
                }
                std::vector<double> rot_ypr;
                if (!nodeHandle.getParam(std::string(base + "/rotation_yaw_pitch_roll"),
                                         rot_ypr)) {
                    ROS_WARN("Did not get rotation at %s", base.c_str());
                } else {
                    tf2::Quaternion quat;
                    quat.setRPY(rot_ypr.at(2), rot_ypr.at(1), rot_ypr.at(0));
                    transformStamped.transform.rotation.x = quat.x();
                    transformStamped.transform.rotation.y = quat.y();
                    transformStamped.transform.rotation.z = quat.z();
                    transformStamped.transform.rotation.w = quat.w();
                }
                vec_transformStamped.push_back(transformStamped);
            }
        }
        return vec_transformStamped;
    }
};
