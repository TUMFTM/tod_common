#ifndef TOD_RECEIVER_H
#define TOD_RECEIVER_H

#include "ros/ros.h"
#include "tod_msgs/StatusMsg.h"
#include "tod_network/tod_network.h"
#include "tod_msgs/connectionStatus.h"
#include <memory>

namespace tod_network {

template <typename T>
class Receiver {
private:
    struct RosMsgReceiver{
        ros::Publisher recvMsgPubs;
        tod_network::UdpReceiver* udpReceiver;
        bool printedInfo{false};
        std::thread* thread;
        T msg;
    };

public:
    Receiver(ros::NodeHandle &n, int port) : _n(n) { add_processer("/received_topic", port); }
    Receiver(ros::NodeHandle &n) : _n(n) { }

    void add_processer(const std::string &topic, const int port) {
        _rosMsgReceivers.push_back(new RosMsgReceiver);
        _rosMsgReceivers.back()->udpReceiver = new tod_network::UdpReceiver(port);
        _rosMsgReceivers.back()->recvMsgPubs = _n.advertise<T>(topic, 1);
        ROS_INFO("created receiver for topic %s on port %d", topic.c_str(), port);
    }

    void run() {
        for (RosMsgReceiver *receiver : _rosMsgReceivers)
            receiver->thread = new std::thread(&Receiver::receive_msgs, this, receiver);
        ros::spin();
    }

private:
    ros::NodeHandle &_n;
    std::vector<RosMsgReceiver*> _rosMsgReceivers;

    void receive_msgs(RosMsgReceiver *receiver) {
        while (ros::ok()) {
            receiver->udpReceiver->receiveRosMsg(receiver->msg);
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

#endif // TOD_RECEIVER_H
