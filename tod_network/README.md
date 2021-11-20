# tod_network

This package contains classes for serializing, sending and receiving ROS messages and configs. Transmission can either be done via UDP or TCP (using MQTT).

## Connection configs

In the file `connection_configs.h`, the respective ports on vehicle and operator side
can be modified.

## Sender and receiver nodes

`tod_sender.h` has a boolean called `senderInVehicle`. This decides about the
topic name, respectively `/Vehicle/…` or `/Operator/…`.

## Dependencies

* ROS Packages:
  * roscpp
  * tod_msgs
* [PahoMqttCpp](https://github.com/eclipse/paho.mqtt.cpp)

