# __ESP32 Platooning node using micro-ROS__


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
Still from the project root run:
```
sudo docker run -it --rm --device /dev/ttyUSB0 --user root --volume="/etc/timezone:/etc/timezone:ro" -v  $(pwd):/platooning_control_esp32_micro_ros --workdir /platooning_control_esp32_micro_ros microros/esp-idf-microros:latest /bin/bash  -c "idf.py menuconfig build flash"
```
