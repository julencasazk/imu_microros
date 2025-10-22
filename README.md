# __Manual Control of a CARLA Ego Vehicle with micro-ROS__
This project contains the firmware for a ESP32 app that subscribes to ROS topics through micro-ROS and publishes to corresponding topics.

It is simply a project to test basic micro-ROS functionality with CARLA sim using a custom bridge from CARLA to ROS.

The Python scripts to launch the ROS nodes to make this project work are in [this repo](https://github.com/julencasazk/carla_scripts).

## Requirements
Following the official [micro-ROS component for ESP-IDF](https://github.com/micro-ROS/micro_ros_espidf_component), install the following requirements:
```
. $IDF_PATH/export.sh
pip3 install catkin_pkg lark-parser colcon-common-extensions
```

## Setup

From the project root, run:
```
mkdir components && cd components
git clone -b humble https://github.com/micro-ROS/micro_ros_espidf_component
cd ..
```

Make sure you have docker installed on your system.

Now connect the ESP32 board to a USB port.
Check it is connected and check for the port number with:
```
ls -l /dev/ttyUSB*
```
Before building the firmware, check if there's anything using the port, as it will cause the flash to fail:
```
sudo lsof /dev/ttyUSB0
```

From the project root directory, run:
```
idf.py menuconfig build flash -p /dev/ttyUSB0 
```
- Make sure UART port is properly configured in `menuconfig`. It's under `micro-ROS Settings ---> UART Settings --->`

If docker is installed, the project can also be built and flashed from the official ESP-IDF docker container with micro-ROS. From the project root run:
```
sudo docker run -it --rm --device /dev/ttyUSB0 --user root --volume="/etc/timezone:/etc/timezone:ro" -v  $(pwd):/$(basename "$PWD") --workdir /$(basename "$PWD") microros/esp-idf-microros:latest /bin/bash  -c "idf.py menuconfig build flash"
```
- Still, building and flashing like this will prevent you from just flashing or just monitoring the board from outside the docker container, without having to clean and rebuild from outside.
## Troubleshooting

- Make sure that you are sourcing ESP-IDF and running idf.py from the Python environment you used to run the `install.sh` when installing ESP-IDF.
- If something failed when building micro-ROS libs, clean the micro-ROS build with:
```
idf.py clean-microros # Will clean uROS if built, or build it if already cleaned
```
- If when building the micro-ROS libs this error pops up:
```
  AttributeError: module 'em' has no attribute 'BUFFERED_OPT'
```
Make sure you have `empy==3.3.4` installed. This issue seems to be related to ROS Humble and newer `>=4` versions of `empy`.

    
