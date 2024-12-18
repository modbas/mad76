# Raspberry Pi Installation

## Raspbian OS

-   Download and start installer: <https://raspberrypi.com/software>

    -   Rasberry Pi OS with desktop

    -   Debian 12 Bookworm 64-bit

    -   Configure `<username>`

    -   Configure `<hostname>`

    -   Configure WiFi

    -   Enable SSH

-   Login: `ssh <username>@<hostname>`

-   Update Debian

                sudo apt-get update
                sudo apt-get dist-upgrade
                # reboot in case of kernel/firmware updates
                sudo shutdown -r 0 

## Rasberry Pi Configuration

-   Enable SPI

    -   `sudo raspi-config`

    -   Goto menu `3 Interface Options`

    -   Select `I4 SPI`

## VNC Server

-   Remove RealVNC

                sudo apt-get purge realvnc-vnc-server

-   Install VNC server

                sudo apt-get install tigervnc-standalone-server
                sudo apt-get install tigervnc-xorg-extension

-   Start VNC server

                vncserver -localhost no -geometry 2550x1350 -depth 24

-   Connect to VNC server: `<hostname>:1`

## WiringPi

-   Install WiringPi for GPIO access

        cd
        mkdir src
        cd src
        git clone https://github.com/WiringPi/WiringPi.git
        cd WiringPi
        ./build

## ROS2

-   Building ROS2 Iron Irwini from source:

    -   <https://docs.ros.org/en/eloquent/Installation/Linux-Development-Setup.html>

    -   <https://docs.ros.org/en/iron/Installation/Alternatives/Ubuntu-Development-Setup.html>

    ``` bash
    mkdir -p src/ros2_iron
    cd src/ros2_iron

    mkdir -p src/ros2_iron/src
    cd src/ros2_iron

    locale  # check for UTF-8
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    locale

    sudo apt-get install \
      build-essential \
      cmake \
      git \
      python3-colcon-bash \
      python3-pip \
      vcstool \
      wget


    sudo apt-get install python3-lark python3-netifaces
    sudo apt-get install python3-flake8-blind-except python3-flake8-builtins python3-flake8-class-newline python3-flake8-comprehensions    python3-flake8-deprecated    python3-flake8-import-order python3-flake8-quotes python3-pytest-repeat python3-pytest-rerunfailures
    sudo apt-get install python3-rosdep2
    sudo apt-get install python3-opencv python3-scipy python3-matplotlib
    sudo apt-get install libbullet-dev libboost-dev
    sudo apt-get install libasio-dev libtinyxml2-dev
    sudo apt-get install qtbase5-dev qtbase5-dev-tools
    sudo apt-get install libacl1-dev libcap-dev libssl-dev libxaw7-dev libogre-1.12-dev libeigen3-dev 
    sudo apt-get install libopencv-dev
    sudo apt-get install liblttng-ust-dev
    sudo apt-get install libboost-python-dev libboost-system-dev libboost-log-dev libgtest-dev libjsoncpp-dev

    wget https://raw.githubusercontent.com/ros2/ros2/iron/ros2.repos
    vcs import src < ros2.repos

    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro iron -y # --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69"

    touch src/eclipse-cyclonedds/COLCON_IGNORE
    touch src/eclipse-iceoryx/COLCON_IGNORE
    touch src/gazebo-release/COLCON_IGNORE
    touch src/ros-visualization/COLCON_IGNORE
    touch src/ros2/rviz/COLCON_IGNORE
    touch src/ros2/rmw_connextdds/COLCON_IGNORE
    touch src/ros2/rmw_cyclonedds/COLCON_IGNORE

    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

-   Install ROS2 packages for camera and diagnostics

    ``` bash
    sudo apt-get install libcamera-dev
    source ~/src/ros2_iron/install/setup.bash
    cd
    mkdir -p src/ros_ws/src
    cd src/ros_ws/src
    git clone https://github.com/ros/diagnostics.git -b ros2-iron  
    git clone https://github.com/ros-perception/vision_opencv.git -b iron
    git clone  https://github.com/christianrauch/camera_ros -b main
    ```

# MAD76 Software

<img src="./ros2nodes.png" alt="image" />

| ROS2 Node | Description |
|:---|:---|
| `rc_node` | control signals output to $2.4\,\mathrm{GHz}$ channel via SPI |
| `camera_node` | Rasberry Pi camera driver |
| `vision_node` | computer vision |
| `locate_node` | multi-object tracking |
| `carctrl_node` | motion planning and control for each individual car |

| ROS2 Topic | ROS2 Message Type | Description |
|:---|:---|:---|
| `/mad/camera/image_raw` | `sensor_msgs::msg::Image` | camera frames with sampling time $25\,\mathrm{ms}$ |
| `/mad/camera/camera_info` | `sensor_msgs::msg::CameraInfo` | camera calibration info |
| `/mad/vision/caroutputs` | `mbmadmsgs::msg::CarOutputsList` | list of car poses |
| `/mad/locate/caroutputsext` | `mbmadmsgs::msg::CarOutputsExtList` | list of car poses including velocities |
| `/mad/car?/carinput` | `mbmadmsgs::msg::CarInputs` | control signals for each individual car |

-   Build ROS2 workspace

    ``` bash
    cd ~/src
    git clone https://<token>@github.com/modbas/mad2
    cd mad2/mad_ws
    ./colcon_ignore_pi.sh
    colcon build --symlink-install ...
      --cmake-args -DCMAKE_CXX_FLAGS=-DMAD76 ... 
      -DCMAKE_BUILD_TYPE=Release
    ```

-   Add security limits

    ``` bash
    sudo addgroup mad
    sudo adduser <username> mad
    sudo -i
    echo "@mad		 -	 rtprio		 98" >> /etc/security/limits.conf
    echo "@mad		 -	 memlock	 unlimited" >> /etc/security/limits.conf
    shutdown -r 0
    ```

[^1]: frank.traenkle@hs-heilbronn.de
