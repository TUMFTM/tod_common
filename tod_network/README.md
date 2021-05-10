# tod_network
This package contains classes for serializing, sending and receiving ROS messages and configs. Transmission can either be done via UDP or TCP (using MQTT).

### Dependencies
  * ROS Packages:
    * roscpp
    * tod_msgs
  * [PahoMqttCpp](https://github.com/eclipse/paho.mqtt.cpp)
    ```
    sudo apt-get install doxygen graphviz -y
    cd ~/Downloads
    git clone https://github.com/eclipse/paho.mqtt.c.git
    cd paho.mqtt.c/
    git checkout v1.3.1
    sudo cmake -Bbuild -H. -DPAHO_WITH_SSL=ON -DPAHO_ENABLE_TESTING=OFF
    sudo cmake --build build/ --target install
    sudo ldconfig
    cd ~/Downloads
    sudo rm -r paho.mqtt.c/
    echo -e "${green}----- Installing PahoMqtt C++ -------${nocolor}"
    cd ~/Downloads
    git clone https://github.com/eclipse/paho.mqtt.cpp.git
    cd paho.mqtt.cpp/
    git checkout v1.1
    sudo cmake -Bbuild -H. -DPAHO_BUILD_DOCUMENTATION=TRUE -DPAHO_BUILD_SAMPLES=TRUE
    sudo cmake --build build/ --target install
    sudo ldconfig
    cd ~/Downloads
    sudo rm -r paho.mqtt.cpp/
    ```
