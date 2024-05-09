# Dependancies
[Serial Driver for Foxy](https://github.com/ros-drivers/transport_drivers/tree/foxy/serial_driver)
[UDP Messages dependancy for Serial Driver](https://github.com/flynneva/udp_msgs/tree/main/)

# Building 
Run the following to build the package `gps_package ` as defined in the CMake.txt file
```bash
colcon build --packages-select gps_package
```

# Serial, Parsing, and Publishing Node
To start the ROS Node responsible for reading the datastream from the serial input, parsing, and publishing, run the following 
```bash
ros2 run gps_package NMEA_parser
```

This script is made up of a message parser, a GPS collector class, and a Node Publisher class. Once serial data is identified, collected, and parsed, it is published for collection by a subscriber node. 

# Subscriber Node 
To start the ROS Node responsible for subscribing to the published parsed message datastream, run the following **in a new terminal**
```bash
ros2 run gps_package receive_gpsMSG
```

This subscribes to the parsing node and saves received data to the parameters defined in the ROS message file. The plotting functionality is currently commented out and not functional but can be added to the existing function.
