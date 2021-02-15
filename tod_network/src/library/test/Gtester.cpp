#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tod_network/tod_network.h>

TEST(MemberFunctionPointerTest, MemberFunctionPointerTestRun) {
    class Manager {
    public:
        Manager() : mqtt_client("127.0.0.1", "Manager") {}
        void set_callback_function() {
            mqtt_client.subscribe("/status_msg", 1,
                &Manager::callback_function, this);
        }
        void callback_function(mqtt::const_message_ptr msg) {
            its_msg_storage = msg->to_string();
        }
        std::string get_its_msg_storage() {
            return its_msg_storage;
        }
    private:
        std::string its_msg_storage;
        tod_network::MqttClientTemplated<Manager>
            mqtt_client;
    } manager;

    manager.set_callback_function();
    tod_network::MqttClient sender("127.0.0.1", "Sender");
    std::string topic_name{ "/status_msg" };
    std::string message_str{ "message" };
    char c[message_str.size() + 1];
    message_str.copy(c, message_str.size() + 1);
    c[message_str.size()] = '\0';
    sender.publish(topic_name, 1, c, message_str.size());
    usleep(80000);
    EXPECT_EQ(manager.get_its_msg_storage(), message_str);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "Tester");
    return RUN_ALL_TESTS();
}
