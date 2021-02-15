// UDP Receiver for ROS messages

#include "ImageExposureStatistics.h"
#include "todnetworklibrary/tod_network.h"

#define PORT    8080
#define MAXLINE 1024

int main() {

// --------------------- Example receiveROSMsg --------------------------
    tod_network::UdpReceiver udpReceiver(PORT);
    int recvBytes{0};
    image_exposure_msgs::ImageExposureStatistics msg;

    while (true) {
        recvBytes = udpReceiver.receiveRosMsg(msg);
        std::cout << "Receiver received " << recvBytes << " Bytes." << std::endl;
        std::cout << "gaindb = " << msg.gaindb << " - underExposed = " << msg.underExposed <<
                     " - time_reference = " << msg.time_reference << std::endl << std::endl;
    }

    return 0;
}
