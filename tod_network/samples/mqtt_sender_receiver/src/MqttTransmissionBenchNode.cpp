// Copyright 2020 Johannes Feiler
#include "ros/ros.h"
#include <string>
#include <cstring>

// todo: delete unnecessary header
#include <thread>
#include <chrono>
#include "tod_network/tod_network.h"

void sender() {
    const std::string DFLT_SERVER_ADDRESS	{ "tcp://localhost:1883" };
    const std::string DFLT_CLIENT_ID		{ "async_publish" };
    const std::string TOPIC { "hello" };
    const char* PAYLOAD1 = "Hello World!";
    const int  QOS = 1;
    const auto TIMEOUT = std::chrono::seconds(10);

    // create client
    tod_network::MqttClient client1(DFLT_SERVER_ADDRESS, DFLT_CLIENT_ID);
    ros::Rate r(1);
    while (ros::ok()) {

        // publish data to mqtt broker
        client1.publish(TOPIC, QOS, PAYLOAD1, std::strlen(PAYLOAD1));
        r.sleep();
    }

    // disconnect when finished
    client1.disconnect();
}

void myCallback(mqtt::const_message_ptr msg) {
    printf("myCallback received %s \n", msg->get_payload().c_str());
}

void receiver() {
    std::string serverAddress {"tcp://localhost:1883"};
    std::string clientName {"async_subscribe"};

    // create client
    tod_network::MqttClient client1(serverAddress, clientName);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 

    // subscribe to topic 'hello' -> myCallback holds the received data
    client1.subscribe("hello", 1, &myCallback);
    while (ros::ok()) {
    }

    // disconnect when finished
    client1.disconnect();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "MQTTNode");
    ros::NodeHandle nh;

    // create a thread, which sends integers to a topic
    std::thread t1(sender);
    std::thread t2(receiver);
    ros::spin();
    t1.join();
    t2.join();
    return 0;
}
