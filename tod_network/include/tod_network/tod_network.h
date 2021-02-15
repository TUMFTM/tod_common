//
// Created by feiler on 15.01.20.
//

#ifndef TOD_NETWORK_H
#define TOD_NETWORK_H

#include <ros/ros.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <vector>
#include <mqtt/async_client.h>
#include <memory>
#include <thread>
#include <chrono>
#include <iostream>


#define MAXPACKETSIZE 4096
#define MAXLINE 100*1024

namespace tod_network
{

static const int RX_MQTT = 1883; // same for operator and vehicle
static const std::string AutoboxIp = "192.168.140.6";

enum VehiclePorts { // only what is sent via udp
    RX_DIRECTCONTROL_COMMAND = 70000,
    RX_CLOTHOIDCONTROL_COMMAND = 70001,
    RX_SHAREDCONTROL_COMMAND = 70002,
    RX_SAFETY_DRIVER_STATUS_AUTOBOX = 60000,
    RX_VEHICLEDATA_AUTOBOX = 60001,
    RX_VIDEO_RTSP = 8554,
    RX_VEHICLE_AUDIO = 62000
};

enum AutoboxPorts {
    RX_DIRECTCONTROL_AUTOBOX_COMMAND = 30000,
};

enum OperatorPorts { // only what is sent via udp
    RX_LIDAR_OBJECTLIST = 50000,
    // 50001 occupied on CarPC4
    RX_VEHICLESTATE_VEHICLEDATA = 50002,
    // 50003
    RX_VEHICLESTATE_ODOMETRY = 50005,
    RX_VEHICLESTATE_GPS = 50006,
    RX_SHAREDCONTROL_PREDICTIONS_LEFT = 50007,
    RX_SHAREDCONTROL_PREDICTIONS_RIGHT = 50008,
    RX_SHAREDCONTROL_PREDICTIONS_STOP = 50009,
    RX_SHAREDCONTROL_OBJECTS = 50010,
    RX_BITRATE_PREDICTIONS = 50011,
    RX_LIDAR_DATA_RANGE_FROM = 50100,
    RX_LIDAR_DATA_RANGE_TO = 50199,
    RX_LIDAR_OBJECTS_RANGE_FROM = 50200,
    RX_LIDAR_OBJECTS_RANGE_TO = 50299,
    RX_LIDAR_OBJECT_MARKER_RANGE_FROM = 50300,
    RX_LIDAR_OBJECT_MARKER_RANGE_TO = 50399,
    RX_AUTOWARE_LANE = 51010,
    RX_OPERATOR_AUDIO = 52000
    // ...
};

namespace MqttTopics {
static const std::string DesiredVideoConfig{"/Operator/Video/DesiredVideoConfig"};
static const std::string ActualVideoConfig{"/Vehicle/Video/ActualVideoConfig"};
static const std::string DesiredBitrateConfig{"/Operator/Video/DesiredBitrateConfig"};
static const std::string ActualBitrateConfig{"/Vehicle/Video/ActualBitrateConfig"};
}



// --------------------------------------------------------------------------------------------------
// ----------------------------------- UDP ----------------------------------------------------------
// --------------------------------------------------------------------------------------------------


class UdpSender {
public:

    /*
     * \brief: Creates udp sender with destIPAddress (f.e. ip address "158.54.5.15") and destPort (f.e. port 80).
     *          destIPAddress and destPort can be changed afterwards with changeDestSpecs()
     */
    UdpSender(const std::string& destIPAddress, const int& destPort);

    // TODO: check if 'rule of five' is necessary here
    ~UdpSender(){ close(sockfd); }

    /*
     * \brief: sends ROS message. Returns number of sent bytes.
     *
     * Due to the fact, that it's a template function, code is put into the header file
     *
     */
    template <class Msg>
    int sendRosMsg(const Msg& msg){
        // serialize ROS message
        ros::SerializedMessage serMsg = ros::serialization::serializeMessage(msg);

        // send
        return sendto(sockfd, serMsg.message_start, serMsg.num_bytes, 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
    };

    /*
     * \brief: sends ROS dynamic reconfigs. Returns number of sent bytes.
     *
     * Due to the fact, that it's a template function, code is put into the header file
     *
     */
    template <class Cfg>
    int sendCfgMsg(const Cfg& cfg){
        // create ros message from dynamic reconfig
        dynamic_reconfigure::Config msg;
        cfg.__toMessage__(msg);

        // serialize ROS message
        ros::SerializedMessage serMsg = ros::serialization::serializeMessage(msg);

        // send
        return sendto(sockfd, serMsg.message_start, serMsg.num_bytes, 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
    };

    /*
     * \brief: Send some bytes to dest server (created in constructor). 
     *          Returns the number sent, or -1 for errors.
     */
    int send(const char *msg /* (char *) &msg */, size_t size /* sizeof(msg) */){
        return sendto(sockfd, msg, size, 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
    }; 
     
    /*
     * \brief: change destination IP address and destination port
     */
    void changeDestSpecs(const std::string& destIPAddress, const int& destPort = -1);

    /*
     *  \brief: prints UDP specs to console
     */
    void printSpecs();

private:
    struct sockaddr_in servaddr;        // server infos
    int sockfd;                         // sender infos
};

class UdpReceiver {
public:
    UdpReceiver(const int& destPort);
    // TODO: check if 'rule of five' is necessary here
    ~UdpReceiver(){     close(sockfd);  }

    /*
     * \brief: writes incoming messages to msg. msg is of type ROS message. Returns the number of received bytes.
     */
    template <class Msg>
    int receiveRosMsg(Msg& msg){
        char buffer[MAXLINE];
        int recvBytes = recv(sockfd, (char *)buffer, MAXLINE, MSG_WAITALL);
        // try-catch in order to suppress termination message when shutting udp-receiver
        // down with not-matching msg from send_data_for_anti_block()
        try {
            // reconstruct message object
            ros::serialization::IStream stream((uint8_t*) buffer, recvBytes);
            ros::serialization::Serializer<Msg>::read(stream, msg);
        } catch (const ros::serialization::StreamOverrunException& e ) {
            // printf("\033[31mreceiveRosMsg got different msg-type\033[0m \n");
        }
        return recvBytes;
    };

    /*
     * \brief: Receive some bytes to dest server (created in constructor). 
     *          Returns the number received, or -1 for errors.
     */
    int recvData(char *msg /* (char *) &msg */){
        return recv(sockfd, msg, MAXLINE, MSG_WAITALL);
    }; 

    /*
     * \brief: closes the udp receiver. Waits until thread closed.
     * 
     * \background: udp receiver waits for msgs and can only be closed after 
     *              there it received some msgs -> sends some msgs
     *              to local host. 
     **/
    int waitUntilUdpReceiverClosed();
     
private:
    struct sockaddr_in servaddr;    // server info
    int sockfd;                     // server info
    std::thread anti_block_thread;  // used in constructor and waitUntilUdp…
    UdpSender anti_block_sender;    // used for anti block thread

    void send_data_for_anti_block();
};

// --------------------------------------------------------------------------------------------------
// ----------------------------------- MQTT ---------------------------------------------------------
// --------------------------------------------------------------------------------------------------

class Client {
public:
    virtual ~Client() =default;
    virtual int subscribe(const std::string& topic, const int& qos, void (*fp)(mqtt::const_message_ptr)) =0;
    virtual int publish(const std::string& topic, const int& qos, const char* msg, size_t length) =0;
    virtual int disconnect() =0;
    virtual bool is_connected() =0;
};

template <class T>
class ClientTemplated {
public:
    virtual ~ClientTemplated() =default;
    virtual int subscribe(const std::string& topic, const int& qos,
                          void (T::*fp)(mqtt::const_message_ptr), T* t) =0;
    virtual int publish(const std::string& topic, const int& qos, const char* msg, size_t length) =0;
    virtual int disconnect() =0;
    virtual bool is_connected() =0;
};

class MqttClient : public Client, public virtual mqtt::callback {
public:
    MqttClient() =delete;
    MqttClient(const std::string& serverAddress, const std::string& clientId)
    : _serverAddress(serverAddress), _clientId(clientId), _client(_serverAddress, _clientId), 
        _clientIsConnected(false), _iactionListener(*this) {
        connectMqttClient();
    }

    /* 
     * \brief: subscribes to a mqtt topic with specified qos.
     *      User has to write a callback function similar to a ROS subscriber callback. Example:
     * 
     *      void callback_function(mqtt::const_message_ptr msg) {
     *          "Your code comes here"
     *      } 
     * 
     *      Last Will and Testament (LWT) are ignored at the moment
     * */
    int subscribe(const std::string& topic /*"/Vehicle/VehicleConnection/Manager/connection_status"*/, 
        const int& qos /*0, 1 or 2*/,
        void (*fp)(mqtt::const_message_ptr)/*&callback_function*/) override;

    /*
     * \brief: publishes a msg with specified length to mqtt topic at specified quality of service (qos)
     *      level.
     * */
    int publish(const std::string& topic /*"/Vehicle/VehicleConnection/Manager/connection_status"*/, 
        const int& qos /*0, 1 or 2*/, const char* msg, size_t length) override;

    /*
     * \brief: disconnects the client from the broker
     * */
    int disconnect() override;

    /*
     * \brief: getter for the _clientIsConnected status
     * */
    bool is_connected() override { return _clientIsConnected; }

    /*
     * \brief: holds the status, if client connected to broker successfully or not.
     *          Needs to be public, because otherwise, iactionListener callback would not have access
     * */
    bool _clientIsConnected;

private:
    /*
     * \brief: Connects to the server 
     *          Returns -1 if no connection is possible
    */
    unsigned int connectMqttClient();

    // void on_failure(const mqtt::token& tok) override { /*add manually by overriding if necessary*/}
    // void on_success(const mqtt::token& tok) override {/*add manually by overriding if necessary*/}

    void connected(const std::string& cause) override { 
        // ROS_INFO("Client %s connected to MQTT server %s successfully", 
        // _clientId.c_str(), _serverAddress.c_str());
    }
    void connection_lost(const std::string& cause) override {/*add manually by overriding if necessary*/}
    void message_arrived(mqtt::const_message_ptr msg) override {
        if(!(msg->get_payload_str()=="Last will and testament.")) {
            _fp( msg );
        }
    }
    void delivery_complete(mqtt::delivery_token_ptr tok) override {/*add manually by overriding if necessary*/}

    std::string _serverAddress;
    std::string _clientId;
    mqtt::connect_options _connOpts;
    mqtt::async_client _client;
    std::string _topic;
    mqtt::token_ptr _subTokenPtr;
    class iactionListener : public virtual mqtt::iaction_listener {

        void on_failure(const mqtt::token& tok) override { 
            _parent._clientIsConnected = false; 
            // printf("on_failure_parent._clientIsConnected: %d \n", _parent._clientIsConnected); 
        }
        void on_success(const mqtt::token& tok) override { 
            _parent._clientIsConnected = true; 
            // printf("on_success _parent._clientIsConnected: %d \n", _parent._clientIsConnected); 
        }

        // hold an address to enclosing parent class in order to change _clientIsConnected state
        MqttClient& _parent;
    public:
        iactionListener(MqttClient& parent) : _parent(parent) {};
    } _iactionListener;
    int _qos;
    void (*_fp)(mqtt::const_message_ptr msg);
};

template <class T>
class MqttClientTemplated : public ClientTemplated<T>, public virtual mqtt::callback {
public:
    MqttClientTemplated() =delete;
    MqttClientTemplated(const std::string& serverAddress, const std::string& clientId)
    : _serverAddress(serverAddress), _clientId(clientId), _client(_serverAddress, _clientId),
        _clientIsConnected(false), _iactionListener(*this) {
        connectMqttClient();
    }

    /*
     * \brief: subscribes to a mqtt topic with specified qos.
     *      User has to write a callback function similar to a ROS subscriber callback. Example:
     *
     *      void callback_function(mqtt::const_message_ptr msg) {
     *          "Your code comes here"
     *      }
     *
     *      Last Will and Testament (LWT) are ignored at the moment
     * */
    int subscribe(const std::string& topic /*"/Vehicle/VehicleConnection/Manager/connection_status"*/,
        const int& qos /*0, 1 or 2*/,
        void (T::*fp)(mqtt::const_message_ptr)/*&T::callback_function*/,
        T* t /* 'this' or '&T' */) {
        _topic = topic;
        _qos = qos;
        _fp = fp;
        _t = t;

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
    }

    /*
     * \brief: publishes a msg with specified length to mqtt topic at specified quality of service (qos)
     *      level.
     * */
    int publish(const std::string& topic /*"/Vehicle/VehicleConnection/Manager/connection_status"*/,
        const int& qos /*0, 1 or 2*/, const char* msg, size_t length) {
        _topic = topic;
        _qos = qos;

        mqtt::message_ptr pubmsg = mqtt::make_message(_topic, msg, length);//sizeof(int));
        pubmsg->set_qos(_qos);
        _client.publish(pubmsg);//->wait();

        return 0;
    }

    /*
     * \brief: disconnects the client from the broker
     * */
    int disconnect() {
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

    /*
     * \brief: getter for the _clientIsConnected status
     * */
    bool is_connected() override { return _clientIsConnected; }

    /*
     * \brief: holds the status, if client connected to broker successfully or not.
     *          Needs to be public, because otherwise, iactionListener callback would not have access
     * */
    bool _clientIsConnected;

private:
    /*
     * \brief: Connects to the server
     *          Returns -1 if no connection is possible
    */
    unsigned int connectMqttClient() {

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

    // void on_failure(const mqtt::token& tok) override { /*add manually by overriding if necessary*/}
    // void on_success(const mqtt::token& tok) override {/*add manually by overriding if necessary*/}

    void connected(const std::string& cause) override {
        // ROS_INFO("Client %s connected to MQTT server %s successfully",
        // _clientId.c_str(), _serverAddress.c_str());
    }
    void connection_lost(const std::string& cause) override {/*add manually by overriding if necessary*/}
    void message_arrived(mqtt::const_message_ptr msg) override {
        if(!(msg->get_payload_str()=="Last will and testament.")) {
            (*_t.*_fp)(msg);
        }
    }
    void delivery_complete(mqtt::delivery_token_ptr tok) override {/*add manually by overriding if necessary*/}

    std::string _serverAddress;
    std::string _clientId;
    mqtt::connect_options _connOpts;
    mqtt::async_client _client;
    std::string _topic;
    mqtt::token_ptr _subTokenPtr;
    class iactionListener : public virtual mqtt::iaction_listener {

        void on_failure(const mqtt::token& tok) override {
            _parent._clientIsConnected = false;
            // printf("on_failure_parent._clientIsConnected: %d \n", _parent._clientIsConnected);
        }
        void on_success(const mqtt::token& tok) override {
            _parent._clientIsConnected = true;
            // printf("on_success _parent._clientIsConnected: %d \n", _parent._clientIsConnected);
        }

        // hold an address to enclosing parent class in order to change _clientIsConnected state
        MqttClientTemplated<T>& _parent;
    public:
        iactionListener(MqttClientTemplated<T>& parent) : _parent(parent) {}
    } _iactionListener;
    int _qos;
    void (T::*_fp)(mqtt::const_message_ptr msg) = nullptr;
    T* _t = nullptr;
};

}   //namespace tod

#endif //TOD_NETWORK_H
