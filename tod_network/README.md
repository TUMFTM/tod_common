# tod_network

This package contains the source code for sending and receiving ROS msgs, ROS cfgs and (char *):
- sender and receiver node template (UDP)
- template to make use of MQTT library
- PORT definitions (application or system specific)

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

### Usage
Add to CMakeLists.txt:
```
find_package(catkin REQUIRED COMPONENTS tod_network)
find_package(PahoMqttCpp REQUIRED)

catkin_package(
    ...
    CATKIN_DEPENDS ... tod_network
)

# add_executable(...)

target_link_libraries(<node_name>
    PahoMqttCpp::paho-mqttpp3
	)
```
Add to package.xml:
```
<build_depend>tod_network</build_depend>
<build_export_depend>tod_network</build_export_depend>
<exec_depend>tod_network</exec_depend>
```

Include in *.cpp:
```
#include "tod_network/tod_network.h"
```

### Samples
Look at *samples/* folder.
In order to test the sample packages, copy the respective package folder (f. e.: samples/ros_msg_sender_receiver) to the package level of your catkin workspace (f. e.: tof/wsp/src/ros_msg_sender_receiver). Otherwise, catkin will not find the package.

