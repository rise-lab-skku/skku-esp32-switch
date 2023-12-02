# micro-ROS on ESP32 Relay Switch

This example shows how to use micro-ROS on an ESP32 to control a relay switch. The ESP32 is connected to a relay switch via GPIO pin 26. The ESP32 is connected to a PC via USB serial.

- ESP32 will attempt to reconnect to the micro-ROS agent if the connection is lost.
- You don't need to manually reset the ESP32 to reconnect to the micro-ROS agent.
- Built-in LED is used to indicate the status of the relay switch and micro-ROS.

**Table of Contents**

1. [Build and Flash the Firmware](#build-and-flash-the-firmware)
   1. [1. Prerequisites](#1-prerequisites)
   2. [2. Install micro-ROS](#2-install-micro-ros)
   3. [3. Install micro-ROS component for ESP32](#3-install-micro-ros-component-for-esp32)
   4. [4. Clone this repository](#4-clone-this-repository)
   5. [5. Build and flash the firmware](#5-build-and-flash-the-firmware)
2. [Connect to the micro-ROS Agent](#connect-to-the-micro-ros-agent)
   1. [Agent in Docker Container (Recommended)](#agent-in-docker-container-recommended)
   2. [Agent in Host PC](#agent-in-host-pc)
3. [Usage](#usage)
4. [Topic and Service](#topic-and-service)
5. [Circuit References](#circuit-references)

## Build and Flash the Firmware

### 1. Prerequisites

1. Add your user to the `tty`, `dialout` group:

   ```zsh
   sudo usermod -a -G tty $USER
   sudo usermod -a -G dialout $USER
   sudo reboot
   ```

2. Install the required packages:

   ```zsh
   sudo apt install python3-pip python3-venv
   ```

3. Install ESP-IDF (ESP32 SDK) using one of the following methods:

   - [Automatic installation using Vscode](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/install.md) (Recommended)
   - [Manual setup for linux](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html)

4. Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). (Other ROS2 distributions are also supported)

### 2. Install micro-ROS

Note: This instruction is using ZSH. If you are using Bash, replace `.zsh` with `.bash`.

```zsh
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.zsh

# Create a workspace and download the micro-ROS tools
mkdir $HOME/microros_ws
cd $HOME/microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Build micro-ROS tools and source them
colcon build
source install/local_setup.zsh
```

### 3. Install micro-ROS component for ESP32

1. Add a convenience script to your `.zshrc`:

   ```zsh
   function setup_ros2_microros_esp32 () {
      WS_DIR=$HOME/microros_ws
      IDF_PATH=$HOME/esp/esp-idf
      . $IDF_PATH/export.sh

      source /opt/ros/humble/setup.zsh
      source $WS_DIR/install/local_setup.zsh
      complete -o nospace -o default -F _python_argcomplete "ros2"

      cd $WS_DIR
   }
   ```

2. Now, you can setup the environment by running:

   ```zsh
   setup_ros2_microros_esp32
   ```

3. Install dependencies of micro-ROS ESP-IDF:

   ```zsh
   setup_ros2_microros_esp32
   pip3 install catkin_pkg lark-parser colcon-common-extensions
   ```

4. Create a firmware workspace. This step will create a `firmware` directory in `/microros_ws`.

   ```zsh
   cd $HOME/microros_ws
   ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32
   ```

### 4. Clone this repository

```zsh
cd $HOME/microros_ws/firmware/freertos_apps/apps
git clone https://github.com/rise-lab-skku/skku-esp32-switch.git skku_esp32_switch
```

### 5. Build and flash the firmware

1. Designate the App to build:

   ```zsh
   cd $HOME/microros_ws
   ros2 run micro_ros_setup configure_firmware.sh skku_esp32_switch --transport serial
   ```

2. Build the firmware:

   ```zsh
   ros2 run micro_ros_setup build_firmware.sh
   ```

3. **Connect the ESP32 to your PC via USB.**
4. Flash the firmware:

   ```zsh
   ros2 run micro_ros_setup flash_firmware.sh
   ```

## Connect to the micro-ROS Agent

The micro-ROS Agent is a bridge between the micro-ROS world and the ROS 2 world.

### Agent in Docker Container (Recommended)

This is the recommended way to run the micro-ROS Agent. (portable, easy to setup)

```zsh
docker run -it --rm --net=host -v /dev:/dev --privileged \
  microros/micro-ros-agent:humble \
  serial --dev /dev/ttyUSB0
```

### Agent in Host PC

Without Docker, you need to manually install the micro-ROS Agent in your host PC.

1. Build:

   ```zsh
   # Download micro-ROS-Agent packages
   ros2 run micro_ros_setup create_agent_ws.sh

   # Build step
   ros2 run micro_ros_setup build_agent.sh
   source install/local_setup.bash
   ```

2. Run:

   ```zsh
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
   ```

## Usage

1. Connect the ESP32 to your PC via USB.
2. Run the micro-ROS Agent. (The ESP32 will automatically connect to the micro-ROS Agent. No need to manually reset the ESP32.)

## Topic and Service

Published Topic: `/skku_esp32_switch/switch_state`

- Type: `std_msgs/msg/Bool`
- Description: The state of the relay switch. `True` means the relay switch is on. `False` means the relay switch is off.
- Publisher: ESP32

```zsh
$ ros2 topic echo /skku_esp32_switch/switch_state
data: true
---
data: true
---
data: true
---
```

Service: `/skku_esp32_switch/set_switch`

- Type: `std_srvs/srv/SetBool`
- Description: Set the state of the relay switch. `True` means the relay switch is on. `False` means the relay switch is off.
- Server: ESP32
- Client: ROS2

```zsh
$ ros2 service call /skku_esp32_switch/set_switch std_srvs/srv/SetBool "data: true"
requester: making request: std_srvs.srv.SetBool_Request(data=True)

response:
std_srvs.srv.SetBool_Response(success=True, message='')
```

## Circuit References

![ESP32 Pinout](https://i0.wp.com/randomnerdtutorials.com/wp-content/uploads/2018/08/ESP32-DOIT-DEVKIT-V1-Board-Pinout-36-GPIOs-updated.jpg?w=750&quality=100&strip=all&ssl=1)

![ESP32 Relay Switch Circuit](https://i0.wp.com/randomnerdtutorials.com/wp-content/uploads/2019/12/relay-esp32-wiring.png?resize=1024%2C662&quality=100&strip=all&ssl=1)
