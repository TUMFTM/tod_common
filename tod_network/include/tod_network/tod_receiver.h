// Copyright 2020 Feiler
#pragma once
#include <ros/ros.h>
#include <tod_msgs/Status.h>
#include <tod_network/udp_receiver.h>
#include <memory>

namespace tod_network {

template <typename T>
class Receiver {
private:
    struct RosMsgReceiver{
        ros::Publisher recvMsgPubs;
        std::unique_ptr<tod_network::UdpReceiver> udpReceiver{nullptr};
        bool printedInfo{false};
        std::unique_ptr<std::thread> thread{nullptr};
        T msg;
    };

public:
    Receiver(ros::NodeHandle &n, int port) : _n(n) { add_processer("/received_topic", port); }
    explicit Receiver(ros::NodeHandle &n) : _n(n) {}

    void add_processer(const std::string &topic, const int port) {
        auto receiver = _rosMsgReceivers.emplace_back(std::make_shared<RosMsgReceiver>());
        receiver->udpReceiver = std::make_unique<tod_network::UdpReceiver>(port);
        receiver->recvMsgPubs = _n.advertise<T>(topic, 1);
    }

    void run() {
        for (auto receiver : _rosMsgReceivers)
            receiver->thread = std::make_unique<std::thread>(&Receiver::receive_msgs, this, receiver);
        ros::spin();
    }


private:
    ros::NodeHandle &_n;
    std::vector<std::shared_ptr<RosMsgReceiver>> _rosMsgReceivers;

    void receive_msgs(std::shared_ptr<RosMsgReceiver> receiver) {
        while (ros::ok()) {
            receiver->udpReceiver->receive_ros_msg(receiver->msg);
            receiver->msg.header.stamp = ros::Time::now();
            receiver->recvMsgPubs.publish(receiver->msg);
            if (!receiver->printedInfo) {
                receiver->printedInfo = true;
                ROS_INFO("%s: receiving topic %s", ros::this_node::getName().c_str(),
                         receiver->recvMsgPubs.getTopic().c_str());
            }
        }
    }
};

}; // namespace tod_network
