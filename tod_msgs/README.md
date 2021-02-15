# tod_msgs
This package contains all custom ROS message definitions used throughout the whole software stack. The structure of the package can be presented as follows:
- **/msg**: Contains all *.msg files that describe custom messages.
- **/srv**: Contains a *.srv file that describes a custom service.
- **/include/tod_msgs**: Contains helper header files where the message constants have been defined as enums.

### Other
Building the service and messages is done in a typical ROS manner (via CMakeLists.txt and package.xml).
