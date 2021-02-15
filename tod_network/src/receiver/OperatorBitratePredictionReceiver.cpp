#include "tod_network/tod_receiver.h"
#include "tod_network/tod_network.h"
#include "geometry_msgs/PointStamped.h"
int main(int argc, char **argv) {
    ros::init(argc, argv, "BitratePredictionReceiver");
    ros::NodeHandle n;
    tod_network::Receiver<geometry_msgs::PointStamped> receiver(n);
    receiver.add_processer("bitratePredOnGps", tod_network::OperatorPorts::RX_BITRATE_PREDICTIONS);
    receiver.run();
    return 0;
}
