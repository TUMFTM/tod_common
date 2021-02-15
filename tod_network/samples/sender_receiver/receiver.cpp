#include "ros/ros.h"
#include "include/tod_test.h"
#include <iostream>
#include "todnetworklibrary/tod_network.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "Receiver");
    ros::NodeHandle n;
    tod_network::UdpReceiver server(1122);
    MsgData msgDataRec;
    while (ros::ok())
    {
        int recvBytes = server.recvData((char *)&msgDataRec);   
        std::cout << "msgDataRec.dNum :" << msgDataRec.dNum << 
            " fNum: " << msgDataRec.fNum <<
            " iNum: " << msgDataRec.iNum << "\n";
    }
    
    // free the receiver by giving it a message
    tod_network::UdpSender freeServer("127.0.0.1", 1122);
    MsgData freeMsg;
    freeMsg.dNum = 2.2; 
    freeMsg.fNum = 1.0;
    freeMsg.iNum = 5;
    for (int it = 0; it < 5; ++it){
        freeServer.send((char *)&freeMsg, sizeof(freeMsg));
    }

    return 0;
} 