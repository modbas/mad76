# MAD76 Features

![image](turbocacing-foto.jpg)

  - Autonomous Driving in 1:76 scale

  - Multi-player robot car racing

  - Race against AI cars using XBox controllers

  - Program your own AI

  - 100 x 50cm tracks

  - Fits in a backpack for easy transport

  - @home @school @university @automotive partners @open source
    @Hochschule Heilbronn @Automotive Systems Engineering

  - \#Computer Vision \#Motion Planning \#Motion Control \#Deep Neural
    Networks \#Reinforcement Learning \#Raspberry Pi Programmming \#C++
    Coding \#Python Coding \#MATLAB/Simulink Simulation \#ROS2

  - <https://www.youtube.com/@ft7894>

# MAD76 Kit

![image](mad76.png)

  - Turboracing 1:76 RC cars (<https://www.turboracing.net>)

  - Raspberry Pi 5 runs Raspberry Pi OS (Debian Linux)

  - Raspberry Pi Global Shutter Camera is mounted on a tripod

  - MAD76 Box houses Raspberry Pi, MAD76 IO (PCB), and Turboracing
    remote controllers

  - MAD76 Driving Stack is implemented as software packages in
    middleware ROS2 (<https://docs.ros.org/en/iron/index.html>)
    
      - Computer Vision reads in frames from top-view camera and detects
        cars by ArUco markers
    
      - Multi-Object Tracking computes the cars’ velocities
    
      - Motion Planning computes the optimal trajectories for the cars
    
      - Motion Control controls the cars on the trajectories
    
      - Optional race simulation

  - MAD76 Driving Stack is programmed in C++ and Python

  - MAD76 Driving Stack may run in a distributed computer environment
    (Raspberry Pi and Linux-PCs)

  - Optional Linux-PCs are for race simulation, programming,
    MATLAB/Simulink simulation, deep neural network training

  - MAD76 IO is designed in Eagle

  - MAD76 Driving Stack and MAD76 IO are open source

  - Contributions are greatly welcome

  - User Manuals
    
      - [Installation](doc/install/install.md)
    
      - [Computer Vision Configuration](doc/vision/vision.md)
