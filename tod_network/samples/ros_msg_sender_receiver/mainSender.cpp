// Server side implementation of UDP client-server model
#include <iostream>

#include "ImageExposureStatistics.h"
#include "encoderConfig.h"

#define PORT    8080
#define MAXLINE 1024

#include "todnetworklibrary/tod_network.h"

int main() {

// --------------------- Example udpsender.sendCfgMsg --------------------------
    std::string IP{"0.0.0.0"};
    tod_network::UdpSender udpSender(IP, PORT);
    udpSender.printSpecs();
//tod::UdpSender test("45.45.45.45");

    image_exposure_msgs::ImageExposureStatistics msg;
    msg.gaindb = 1.324;
    msg.underExposed = 4;
    msg.time_reference = "ab";

    while (true) {
        msg.gaindb *= 1.5;
        msg.underExposed +=1;
        msg.time_reference += "cd";

        int sent = udpSender.sendRosMsg(msg);
        std::cout << "Sender sent: " << sent << " Bytes. " << std::endl;
        std::cout << "gaindb = " << msg.gaindb << " - underExposed = " << msg.underExposed <<
                     " - time_reference = " << msg.time_reference << std::endl << std::endl;

        usleep(1000000);
    }

    return 0;

}

