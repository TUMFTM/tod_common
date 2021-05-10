// Copyright 2020 Feiler
#include "ros/ros.h"
#include "tod_transform/CommonTransformTreePublisher.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "CommonTransformTreePublisherNode");
    ros::NodeHandle nodeHandle;

    static tf2_ros::StaticTransformBroadcaster tf_broadcaster;
    std::vector<geometry_msgs::TransformStamped> vec_transformStamped;
    std::string nodeName = ros::this_node::getName();
    vec_transformStamped = CommonTransformTreePublisher::
                getTransformTreeFromParamServer(nodeName, nodeHandle);

    ros::Rate r{100};
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
        tf_broadcaster.sendTransform(vec_transformStamped);
    }

    return 0;
}
