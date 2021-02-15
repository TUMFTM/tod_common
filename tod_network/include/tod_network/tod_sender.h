#ifndef TOD_SENDER_H
#define TOD_SENDER_H

#include "ros/ros.h"
#include "tod_msgs/StatusMsg.h"
#include "tod_network/tod_network.h"
#include "tod_msgs/connectionStatus.h"
#include <memory>

namespace tod_network
{
template <typename T>
class Sender
{
private:
    struct RosMsgSender {
        ros::Subscriber sendMsgSubs;
        tod_network::UdpSender* udpSender;
        bool printedInfo{false};
    };

public:
    Sender(ros::NodeHandle &n, int port, bool senderInVehicle)
        : _n(n), _senderInVehicle(senderInVehicle) {
        std::string statusTopic =
            (_senderInVehicle) ? "/Vehicle/Manager/status_msg" : "/Operator/Manager/status_msg";
        _statusSubs = _n.subscribe(statusTopic, 1, &Sender::statusMessageReceived, this);
        add_processer("/topic_to_send", port);
    }
    Sender(ros::NodeHandle &n, bool senderInVehicle)
        : _n(n), _senderInVehicle(senderInVehicle) {
        std::string statusTopic =
            (_senderInVehicle) ? "/Vehicle/Manager/status_msg" : "/Operator/Manager/status_msg";
        _statusSubs = _n.subscribe(statusTopic, 1, &Sender::statusMessageReceived, this);
    }

    void run() { ros::spin(); }

    void add_processer(const std::string &topic, const int port) {
        _rosMsgSenders.push_back(new RosMsgSender);
        _rosMsgSenders.back()->udpSender = new tod_network::UdpSender(_receiverIp, port);
        _rosMsgSenders.back()->sendMsgSubs = _n.subscribe<T>(topic, 1,
                                                             boost::bind(&Sender::sendMessageReceived,
                                                                         this, _1, _rosMsgSenders.back()));
        ROS_INFO("%s: created sender for topic %s on port %d",
                 ros::this_node::getName().c_str(), topic.c_str(), port);
    }

private:
    std::string _receiverIp{"127.0.0.1"};
    std::vector<RosMsgSender*> _rosMsgSenders;
    int _connectionStat{ConnectionStatus::IDLE};
    ros::Subscriber _statusSubs;
    ros::NodeHandle &_n;
    bool _senderInVehicle;

    bool connected() {
        return _connectionStat != ConnectionStatus::IDLE;
    }

    void statusMessageReceived(const tod_msgs::StatusMsg &msg) {
        _connectionStat = msg.tod_status;
        if (connected()) {
            if (_senderInVehicle && _receiverIp != msg.operator_operator_ip_address) {
                _receiverIp = msg.operator_operator_ip_address;
                ROS_INFO("%s: sending to operator ip address %s",
                         ros::this_node::getName().c_str(), _receiverIp.c_str());
                for (RosMsgSender *rosMsgSender : _rosMsgSenders)
                    rosMsgSender->udpSender->changeDestSpecs(_receiverIp);
            } else if (!_senderInVehicle && _receiverIp != msg.vehicle_vehicle_ip_address) {
                _receiverIp = msg.vehicle_vehicle_ip_address;
                ROS_INFO("%s: sending to vehicle ip address %s",
                         ros::this_node::getName().c_str(), _receiverIp.c_str());
                for (RosMsgSender *rosMsgSender : _rosMsgSenders)
                    rosMsgSender->udpSender->changeDestSpecs(_receiverIp);
            }
        }
    }

    void sendMessageReceived(const ros::MessageEvent<T const>& event, RosMsgSender *rosMsgSender) {
        if (connected()) {
            rosMsgSender->udpSender->sendRosMsg(*event.getMessage());
            if (!rosMsgSender->printedInfo) {
                ROS_INFO("%s: sending topic %s", ros::this_node::getName().c_str(),
                         rosMsgSender->sendMsgSubs.getTopic().c_str());
                rosMsgSender->printedInfo = true;
            }
        }
    }
}; // class Sender

}; // namespace tod_network

#endif // TOD_SENDER_H
