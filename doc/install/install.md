# Installation Overview

![MAD76 System Architecture](sysarch.png)

The installation steps are:

  - Build the MAD76 Box including the MAD76 IO PCB (see
    Section [2](#mad76-box))

  - Install Raspberry Pi OS, drivers, and ROS2 (see
    Section [3](#raspberry-pi-installation))

  - Optionally install ROS2 on optional Linux-PC for distributed
    computing and software-in-the-loop (SiL) simulation (see
    Section [4](#linux-pc-installation))

  - Install MAD76 Driving Stack (see Section [5](#mad76-driving-stack))

# MAD76 Box

The MAD76 Box is a self-built housing for the MAD76 electronics
containing

  - Raspberry Pi (RPi)

  - MAD76 IO: self-built PCB electronics for coupling of RPi to remote
    controllers (RC) for the Turbotacing cars

  - up to 4 Turboracing remote controllers (RC)

This section first lists the bill of materials (BOM) for the MAD76 Box.
Then the MAD76 IO is described in more detail.

## Bill of Materials (BOM)

### Raspberry Pi and Camera

<div id="T:bomrpi">

|   | Description                                                               | Part Id          | Order Link                                                                                                  |
| -: | :------------------------------------------------------------------------ | :--------------- | :---------------------------------------------------------------------------------------------------------- |
| 1 | RPI5 BBDL 8GB Das Raspberry PI 5 B 8GB Black Bundle                       | RPI5 BBDL 8GB    | <https://www.reichelt.de/das-raspberry-pi-5-b-8gb-black-bundle-rpi5-bbdl-8gb-p362348.html>                  |
| 1 | microSD-Card 128 GB                                                       |                  | <https://www.rasppishop.de/Sandisk-microSDHC-UHS-I-128GB-Class10-mit-Raspberry-Pi-OS>                       |
| 1 | RASP ACTIVE COOL Raspberry Pi - Lüfter für Raspberry Pi 5                 | RASP ACTIVE COOL | <https://www.reichelt.de/raspberry-pi-luefter-fuer-raspberry-pi-5-rasp-active-cool-p360116.html>            |
| 1 | RASP CAM GS CS Raspberry Pi-Kamera, 1,6MP,Global Shutter,C-/CS-Fassung    | RASP CAM GS CS   | <https://www.reichelt.de/raspberry-pi-kamera-1-6mp-shutter-c-cs-fassung-rasp-cam-gs-cs-p345205.html>        |
| 1 | RPIZ CAM 6MM WW Raspberry Pi-Objektiv für CS-Fassung,6mm,Weitwinkel       | RPIZ CAM 6MM WW  | <https://www.reichelt.de/raspberry-pi-objektiv-fuer-cs-fassung-6mm-weitwinkel-rpiz-cam-6mm-ww-p276922.html> |
| 1 | AZDelivery Ersatz Flexkabel 50 cm kompatibel mit Raspberry Pi Zero Kamera |                  | <https://www.amazon.de/AZDelivery-Flexkabel-Raspberry-Zero-Display/dp/B07SQ3HKNF>                           |
| 1 | Joby GorillaPod 3K Kit                                                    |                  | <https://www.foto-erhardt.de/stative/joby-gorillapod/joby-gorillapod-3k-kit-black-charcoal.html>            |

BOM of Raspberry Pi and camera

</div>

### MAD76 IO

<div id="T:bommad76io">

|   | Description                                                                | Part Id       | Order Link                                                                                          |
| -: | :------------------------------------------------------------------------- | :------------ | :-------------------------------------------------------------------------------------------------- |
| 4 | MCP42010 10kOhm DIL-14                                                     | MCP 42010-I/P | <https://www.reichelt.de/digitalpoti-2-kanal-256-schritte-10-kohm-dil-14-mcp-42010-i-p-p90112.html> |
| 1 | L293B 1A DIP-16                                                            | L 293 B       | <https://www.reichelt.de/push-pull-4-kanal-treiber-1a-dip-16-l-293-b-p9660.html>                    |
| 4 | 14-poliger DIL-Sockel (IC-Sockel, 14-polig, superflach, gedreht, vergold.) | GS 14P        | <https://www.reichelt.de/ic-sockel-14-polig-superflach-gedreht-vergold--gs-14p-p8207.html>          |
| 1 | 16-poliger DIL-Sockel (IC-Sockel, 16-polig, superflach, gedreht, vergold.) | GS 16P        | <https://www.reichelt.de/ic-sockel-16-polig-superflach-gedreht-vergold--gs-16p-p8209.html>          |
| 4 | Wannenstecker, 10-polig, gerade                                            | WSL 10G       | <https://www.reichelt.de/wannenstecker-10-polig-gerade-wsl-10g-p22816.html>                         |
| 1 | Wannenstecker, 40-polig, gewinkelt                                         | WSL 40W       | <https://www.reichelt.de/wannenstecker-40-polig-gewinkelt-wsl-40w-p22836.html>                      |

BOM of MAD76 IO PCB

</div>

### Housing

<div id="T:bommad76housing">

|   | Description                                                                                                             | Part Id         | Order Link                                                                                         |
| -: | :---------------------------------------------------------------------------------------------------------------------- | :-------------- | :------------------------------------------------------------------------------------------------- |
| 1 | Industriegehäuse, 250 x 160 x 90 mm, IP65, lichtgrau                                                                    | 5U340000        | <https://www.reichelt.de/industriegehaeuse-250-x-160-x-90-mm-ip65-lichtgrau-5u340000-p324394.html> |
| 1 | 40-poliges Flachbandkabel 30cm                                                                                          | RPI GPIO40 300  | <https://www.reichelt.de/raspberry-pi-gpio-kabel-40-pin-30cm-grau-rpi-gpio40-300-p293579.html>     |
| 1 | Sechskantmuttern, Edelstahl A2, M3, 100 Stück                                                                           | SK-E M3-100     | <https://www.reichelt.de/sechskantmuttern-edelstahl-a2-m3-100-stueck-sk-e-m3-100-p72592.html>      |
| 1 | sourcing map 20Stk. M2,5x8mm+5mm Stecker Buchse Messing PCB Motherboard Abstandhalter Ständer                           |                 | <https://www.amazon.de/gp/product/B08G1TP68G>                                                      |
| 1 | 300 Stück M2.5 Schrauben Set M2.5 Hex Flach-Knopf Schraube Set, A2 Edelstahl Innensechskantschrauben Schraubensortiment |                 | <https://www.amazon.de/gp/product/B08B648WWQ>                                                      |
| 8 | Stecker (Buchse) Kontakte für Funke / JST PH3P BU JST - Buchsengehäuse, 1x3-polig -                                     | PH 571-440129-3 | <https://www.mouser.de/ProductDetail/571-440129-3>                                                 |
| 8 | JST - Buchsengehäuse, 1x3-polig - PH JST PH3P BU                                                                        |                 | <https://www.reichelt.de/jst-buchsengehaeuse-1x3-polig-ph-jst-ph3p-bu-p185042.html>                |

BOM of MAD76 Box housing

</div>

### Turboracing Cars

<div id="T:bommad76cars">

|        | Description                                                          | Part Id | Order Link                                             |
| -----: | :------------------------------------------------------------------- | :------ | :----------------------------------------------------- |
| 1 to 4 | Turbo Racing 1:76 Mini Cooper with RC <https://www.turboracing.net/> |         | <https://de.aliexpress.com/item/1005001936818767.html> |
|      1 | Turbo Racing Mat Track 50x95cm                                       |         | <https://de.aliexpress.com/item/1005006267808509.html> |

BOM of Turboracing cars

</div>

## MAD76 IO

  - MAD76 IO is the bridge from RPi to the Turboracing RCs.

  - MAD76 IO controls up to 4 cars.

  - MAD76 IO substitutes and emulates the two potentiometers for
    throttle/braking and steering by digital potis (MCP42010) for each
    car.

  - MAD76 further provides the power supply of 5V for the RCs.

  - The power supply is controlled individually for each RC by an L293B.

  - The RPi controls the digital potis via SPI.

  - The RPi controls the L293B via GPIO.

  - The MAD76 IO is connected to the RPi via a standard RPi 40-pin GPIO
    cable.

  - The MAD76 IO is connected to the RCs via 8-pin flat ribbon cables.

![MAD76 IO Schematics Page 1 (Eagle schematics
[../../pcb/MAD76.sch](../../pcb/MAD76.sch))](mad76io_schematics01.png)

![MAD76 IO Schematics Page 2 (Eagle schematics
[../../pcb/MAD76.sch](../../pcb/MAD76.sch))](mad76io_schematics02.png)

![MAD76 IO Board Layout (Eagle layout
[../../pcb/MAD76.brd](../../pcb/MAD76.brd))](mad76io_board.png)

# Raspberry Pi Installation

## Raspberry Pi OS

  - Download and start installer \[[1](#ref-raspberrypi-sw)\]
    
      - Rasberry Pi OS with desktop (Debian 12 Bookworm 64-bit)
    
      - Configure `<username>`
    
      - Configure `<hostname>`
    
      - Configure WiFi
    
      - Enable SSH

  - Login: `ssh <username>@<hostname>`

  - Update Debian
    
        sudo apt-get update
        sudo apt-get dist-upgrade
        # reboot in case of kernel/firmware updates
        sudo shutdown -r 0 

## Rasberry Pi Configuration

  - Enable SPI
    
      - `sudo raspi-config`
    
      - Goto menu `3 Interface Options`
    
      - Select `I4 SPI`

## VNC Server

  - Remove RealVNC
    
        sudo apt-get purge realvnc-vnc-server

  - Install VNC server
    
        sudo apt-get install tigervnc-standalone-server
        sudo apt-get install tigervnc-xorg-extension

  - Start VNC server
    
        vncserver -localhost no -geometry 2550x1350 -depth 24

  - Connect to VNC server: `<hostname>:1`

## WiringPi

  - Install WiringPi for GPIO access
    
        cd
        mkdir src
        cd src
        git clone https://github.com/WiringPi/WiringPi.git
        cd WiringPi
        ./build

## ROS2

  - Building <span>ROS2 Iron Irwini</span> from source
    \[[2](#ref-ros-buildonlinux)\],
    \[[3](#ref-ros-installubuntusource)\]
    
        mkdir -p ~/src/ros2_iron/src
        cd ~/src/ros2_iron
        
        locale  # check for UTF-8
        
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

  - Install ROS2 packages for camera and diagnostics
    
    ``` 
    sudo apt-get install libcamera-dev
    source ~/src/ros2_iron/install/setup.bash
    mkdir -p /src/ros_ws/src
    cd ~/src/ros_ws/src
    git clone https://github.com/ros/diagnostics.git -b ros2-iron  
    git clone https://github.com/ros-perception/vision_opencv.git -b iron
    git clone  https://github.com/christianrauch/camera_ros -b main
    cd ..
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release  
    ```

## Update ROS2

If you want to update ROS2 later on, you can do the following.

  - Update ROS2 distribution
    
        cd ~/src/ros2_iron
        vcs custom --args remote update
        vcs import src < ros2.repos
        vcs pull src
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

  - Update ROS2 packages for camera and diagnostics
    
        cd ~/src/ros_ws/src
        cd diagnostics
        git pull
        cd ../vision_opencv
        git pull
        cd ../camera_ros
        git pull
        cd ../..
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

## Xbox One Controllers

  - Enable Bluetooth Low Energy (BLE) privacy
    
      - <https://www.reddit.com/r/linux_gaming/comments/js0trh/comment/gddwyjk/>
    
      - Add line `Privacy=device` to the `[General]` section of
        `/etc/bluetooth/main.conf`

  - Follow the instructions on
    <https://pimylifeup.com/xbox-controllers-raspberry-pi/>

# Linux-PC Installation

  - Install an Ubuntu Desktop version that supports <span>ROS2 Iron
    Irwini</span>, such as Ubuntu Jammy Jellyfish 22.04
    \[[4](#ref-ubuntu-alternativedownloads)\]. <span>ROS2 Iron
    Irwini</span> (and no other ROS2 version) is required, otherwise
    distributed computing with PC and Raspberry Pi will not work.

  - Install <span>ROS2 Iron Irwini</span> binary (deb) packages
    according to \[[5](#ref-ros-installubuntudeb)\]. Make sure to
    install `ros-dev-tools` and `ros-iron-desktop` packages.

  - Install additonal Ubuntu packages required by MAD76
    
        sudo apt-get install git ros-iron-diagnostic-updater

# MAD76 Driving Stack

## Software Architecture

![ROS2 nodes of MAD76 Driving Stack](ros2nodes.png)

<div id="T:ros2nodes">

| ROS2 Node      | Description                                                          |
| :------------- | :------------------------------------------------------------------- |
| `camera_node`  | Rasberry Pi camera driver                                            |
| `vision_node`  | computer vision                                                      |
| `locate_node`  | multi-object tracking                                                |
| `carctrl_node` | motion planning and control for each individual car                  |
| `rc_node`      | remote control signals output to \(2.4\mathrm{GHz}\) channel via SPI |

ROS2 nodes of MAD76 software

</div>

<div id="T:ros2topics">

| ROS2 Topic                  | ROS2 Message Type                   | Description                                        |
| :-------------------------- | :---------------------------------- | :------------------------------------------------- |
| `/mad/camera/image_raw`     | `sensor_msgs::msg::Image`           | camera frames with sampling time \(25\mathrm{ms}\) |
| `/mad/camera/camera_info`   | `sensor_msgs::msg::CameraInfo`      | camera calibration info                            |
| `/mad/vision/caroutputs`    | `mbmadmsgs::msg::CarOutputsList`    | list of car poses                                  |
| `/mad/locate/caroutputsext` | `mbmadmsgs::msg::CarOutputsExtList` | list of car poses including velocities             |
| `/mad/car?/carinput`        | `mbmadmsgs::msg::CarInputs`         | control signals for each individual car            |

ROS2 topics of MAD76 software

</div>

## Build MAD76

  - Clone Git repository and build MAD76 workspace
    
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export ROS_DOMAIN_ID=221
        source ~/src/ros_ws/install/setup.bash # on Raspberry Pi
        #source /opt/ros/iron/install/setup.bash # on Ubuntu Linux-PC
        cd ~/src
        git clone https://<token>@github.com/modbas/mad76
        cd mad76/mad_ws
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

  - Add security limits
    
        sudo addgroup mad
        sudo adduser <username> mad # where <username> is your username
        sudo -i
        echo "@mad		 -	 rtprio		 98" >> /etc/security/limits.conf
        echo "@mad		 -	 memlock	 unlimited" >> /etc/security/limits.conf
        shutdown -r 0 # reboot

  - Add the following lines to the end of `~/.bashrc` for automatic
    setup
    
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export ROS_DOMAIN_ID=221
        source ~/src/mad76/mad_ws/install/setup.bash

## Software-in-the-Loop Simulation

  - Test MAD76 installation in software-in-the-loop (SiL) simulation
    
      - The MAD76 Driving Stack runs in the loop with vehicle dynamics
        simulations
    
      - Full operation of the driving stack is supported in SiL mode

  - Start MAD76 in SiL mode
    
        ros2 launch mbmad madpisim.launch

  - Open a new terminal and send maneuver to car 0 (yellow car)
    
        ros2 run mbmadcar send_maneuver.py 0 0.3 0.25
    
      - First argument is the car identifier (0 for yellow car, 1 for
        orange car)
    
      - Second argument is the car reference speed in \(m/s\)
    
      - Third argument is the lateral reference position (0 for right
        curb, 0.25 for right lane, 0.5 for center line, 0.75 for left
        lane, 1 for left curb)

  - Stop `send_maneuver.py` by hitting `Ctrl+c` and send maneuver to car
    1 (orange car)
    
        ros2 run mbmadcar send_maneuver.py 1 0.2 0.25

# References

<div id="refs" class="references">

<div id="ref-raspberrypi-sw">

\[1\] Raspberry Pi Foundation, “Raspberry Pi Software.” 2024. Available:
<https://raspberrypi.com/software>

</div>

<div id="ref-ros-buildonlinux">

\[2\] ROS, “Building ROS2 on Linux.” 2024. Available:
<https://docs.ros.org/en/eloquent/Installation/Linux-Development-Setup.html>

</div>

<div id="ref-ros-installubuntusource">

\[3\] ROS, “Installation Alternatives Ubuntu (Source).” 2024. Available:
<https://docs.ros.org/en/iron/Installation/Alternatives/Ubuntu-Development-Setup.html>

</div>

<div id="ref-ubuntu-alternativedownloads">

\[4\] Canonical Ubuntu, “Alternative Downloads.” 2024. Available:
<https://ubuntu.com/download/alternative-downloads>

</div>

<div id="ref-ros-installubuntudeb">

\[5\] ROS, “Installation Ubuntu (deb packages).” 2024. Available:
<https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debs.html>

</div>

</div>
