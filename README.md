# SERIAL_IO

This ROS package provides a simple interface for writing to and reading from a serial port (eg: Arduino).

## Nodes

* #### serial_interface:
   It has a write and read line functionality. Port name and baud rate should be provided as ROS parameter and it will try to open the specified port in the beginning. [wiki](https://github.com/behrooz-tahanzadeh/serial_io/wiki/serial_interface)

* #### smart_serial_interface:
   _(under development)_ The purpose of this node is to provide a mechanism of finding and binding a right port to specific topics.

## Installation

```bash
cd ~/catkin_ws/src/
git clone https://github.com/behrooz-tahanzadeh/serial_io.git
cd ..
catkin_make
```
