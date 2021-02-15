#include "tod_network/tod_sender.h"
#include "tod_network/tod_network.h"
#include "geometry_msgs/PointStamped.h"
int main(int argc, char **argv) {
    ros::init(argc, argv, "BitratePredictionSender");
    ros::NodeHandle n;
    tod_network::Sender<geometry_msgs::PointStamped> sender(n, true);
    sender.add_processer("bitratePredOnGps", tod_network::OperatorPorts::RX_BITRATE_PREDICTIONS);
    sender.run();
    return 0;
}
