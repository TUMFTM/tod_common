//
// Created by feiler on 15.01.20.
//

#include "tod_network/tod_network.h"

namespace tod_network
{

// --------------------------------------------------------------------------------------------------
// ----------------------------------- UDP ----------------------------------------------------------
// --------------------------------------------------------------------------------------------------

UdpSender::UdpSender(const std::string& destIPAddress, const int& destPort){
    // Set destIPAddress and destPort
    changeDestSpecs(destIPAddress, destPort);

    // Create sender socket file descriptor
    if ( (sockfd = socket(servaddr.sin_family, SOCK_DGRAM, 0)) < 0 ) {
        perror("cannot create socket");
        printf("%s:%i", destIPAddress.c_str(), destPort);
        exit(EXIT_FAILURE);
    }
};

void UdpSender::changeDestSpecs(const std::string &destIPAddress, const int &destPort){
    // convert IP string to char
    char cdestIP[destIPAddress.size()+1];
    std::copy(destIPAddress.begin(), destIPAddress.end(), cdestIP);
    cdestIP[destIPAddress.size()] = '\n';

    // fill servaddr with values
    inet_aton(cdestIP, &(servaddr.sin_addr));   // ip
    if (destPort >= 0) servaddr.sin_port = htons(destPort);        // port
    servaddr.sin_family = AF_INET;              // address family, AF_INET for IPv4.
}

void UdpSender::printSpecs(){
    printf("Destination IP Address %s\n", inet_ntoa(servaddr.sin_addr));
    auto port = ntohs(servaddr.sin_port);
    printf("Destination Port %i\n", port);
}

UdpReceiver::UdpReceiver(const int& destPort) :
    anti_block_thread{}, anti_block_sender("127.0.0.1", destPort)
    {
    // Filling server information
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(destPort);
    servaddr.sin_addr.s_addr = INADDR_ANY;

    // Creating socket file descriptor
    if ( (sockfd = socket(servaddr.sin_family, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        printf("port %i", destPort);
        exit(EXIT_FAILURE);
    }

    // Bind the socket with the server address
    if ( bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 ) {
        perror("bind failed");
        printf("port %i", destPort);
        exit(EXIT_FAILURE);
    }

    // create anti_block_thread() to send some msgs to 
    anti_block_thread = std::thread(&UdpReceiver::send_data_for_anti_block, this);
}

void UdpReceiver::send_data_for_anti_block() {
    if (!ros::ok()) {
        printf("\033[31m ERROR @ UdpReceiver::send_data_for_anti_block() port %i: "
            " UDP-RECEIVER NEEDS TO BE initialized AFTER ros::init() \033[0m \n", servaddr.sin_port);
    }

    while (ros::ok())
    {
        // do nothing
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    
    // if ros is quit, start to send data from sender for freeing receiver
    // problem: msg-type muss identisch zu dem sein, was der beim recv 
    //          erwartet -> woher bekomme ich den msg-typ?
    int integer { 0 };
    // 20 is just a best guess
    for( int it = 0; it != 20; ++it ) {
        anti_block_sender.send((char *) &integer, sizeof(integer));
    }
}

int UdpReceiver::waitUntilUdpReceiverClosed() {
    anti_block_thread.join();
    return 0;
}

// --------------------------------------------------------------------------------------------------
// ----------------------------------- MQTT ---------------------------------------------------------
// --------------------------------------------------------------------------------------------------

unsigned int MqttClient::connectMqttClient() {

    // some code just taken from PahoMqttCpp sample: async_subscribe.cpp
    _connOpts.set_keep_alive_interval(20);
    _connOpts.set_clean_session(true);
    _connOpts.set_connect_timeout(4);

    // set the callback to the client
    _client.set_callback(*this);

    // ROS_INFO("Client %s is connecting to MQTT server %s", 
    //     _clientId.c_str(), _serverAddress.c_str());
    try {
        // try to connect for 2000 milliseconds. _clientIsConnected is changed respectively in _iactionListener
        _client.connect(_connOpts, nullptr, _iactionListener)->wait_for(2000);
    }
    catch (const mqtt::exception&) {
        ROS_ERROR("Client %s unable to connect to MQTT server %s", 
            _clientId.c_str(), _serverAddress.c_str());
        return -1;
    }
    return 0;
}

int MqttClient::subscribe(const std::string& topic /*"/Vehicle/VehicleConnection/Manager/connection_status"*/, 
const int& qos, void (*fp)(mqtt::const_message_ptr)/*&callbackFoo*/) {
    _topic = topic;
    _qos = qos;
    _fp = fp;  

    // listen to the mqtt broker -> 'void connected' gets triggered, if something arrives
    try {
        _subTokenPtr=_client.subscribe(_topic, _qos);
        _subTokenPtr->wait();
    }
    catch (const mqtt::exception& exc) {
        std::cerr << exc.what() << "\nClient " << _clientId.c_str() << "\n" << std::endl;
        return 1;
    }
    return 0;
};

int MqttClient::publish(const std::string& topic /*"/Vehicle/VehicleConnection/Manager/connection_status"*/, 
const int& qos /*0, 1 or 2*/, const char* msg, size_t length) {
    _topic = topic;
    _qos = qos;

    mqtt::message_ptr pubmsg = mqtt::make_message(_topic, msg, length);//sizeof(int));
    pubmsg->set_qos(_qos);
    _client.publish(pubmsg);//->wait();

    return 0;
};

int MqttClient::disconnect() {
    try {
        // printf("Client %s is disconnecting from MQTT server %s\n", 
        //     _clientId.c_str(), _serverAddress.c_str());
        _client.connect(_connOpts);
        _client.disconnect()->wait();
        _clientIsConnected = false;
        // printf("Client %s is disconnected from MQTT server %s\n", 
        //     _clientId.c_str(), _serverAddress.c_str());
    }
    catch (const mqtt::exception& exc) {
        std::cerr << exc.what() << "\nClient " << _clientId.c_str() << "\n" << std::endl;
        return 1;
    }
    return 0;
};

} // namespace tod

