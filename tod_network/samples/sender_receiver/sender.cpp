#include "ros/ros.h"
#include "include/tod_test.h"
#include "todnetworklibrary/tod_network.h"
#include <iostream>

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "Sender");
    ros::NodeHandle nh;
    ros::Rate r(10);
    MsgData msgData{};
    msgData.dNum = 1.0;
    msgData.fNum = 2.0;
    msgData.iNum = 3;
    tod_network::UdpSender client("127.0.0.1", 1122);
    while(ros::ok()){
        msgData.dNum *= 1.1;
        msgData.fNum *= 1.1;
        msgData.iNum += 2;
        client.send((char *) &msgData, sizeof(msgData));
        std::cout << "msgData.dNum :" << msgData.dNum << " msgData.fNum :" << msgData.fNum 
                << " msgData.int :" << msgData.iNum << "\n";
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}