| Hochschule Heilbronn               |
|------------------------------------|
| Model-Based Software Engineering   |
|                                    |
| **Manuscript Winter Term 2024/25** |
|                                    |
| **Prof. Dr.-Ing. Frank Tränkle**   |
| **frank.traenkle@hs-heilbronn.de** |
|                                    |
| **November 30, 2024**              |

|   |
|---|

Contents

[1 Introduction](#introduction)

[2 Model-Based Software Engineering](#model-based-software-engineering)

[3 Lab Project Mini-Auto-Drive (MAD)](#lab-project-mini-auto-drive-mad)

[3.1 System Overview](#system-overview)

[3.2 Model-Based Software Engineering](#model-based-software-engineering-1)

[3.3 Software Architecture](#_Toc177558574)

[3.4 Required Lab Results](#required-lab-results)

[3.5 Robot Operating System (ROS)](#_Toc177558576)

[3.5.1 Features](#features)

[3.5.2 ROS Nodes](#ros-nodes)

[4 Basics in Signals and Systems](#_Toc177558579)

[5 Vehicle Dynamics Simulation](#_Toc177558580)

[5.1 Longitudinal Dynamics](#_Toc177558581)

[5.2 Bicycle Model for Rear Axle](#_Toc177558582)

[5.3 Bicycle Model for Front Axle](#bicycle-model-for-front-axle)

[5.4 Bicycle Model for Arbitrary Point on Longitudinal Axis](#_Toc177558584)

[5.5 Vehicle Dynamics Model with Inertia and Tire Forces](#_Toc177558585)

[5.6 Exercises](#exercises)

[6 Speed Control](#_Toc177558587)

[6.1 PI-Controller Design for PT1-Plant](#_Toc177558588)

[6.2 Requirements of the Control Loop Dynamics](#requirements-of-the-control-loop-dynamics)

[6.3 Exercises](#exercises-1)

[7 Longitudinal Position Control](#_Toc177558591)

[7.1 Exercises](#exercises-2)

[8 Path Definition](#path-definition)

[8.1 Frenet-Serret Formulas](#frenet-serret-formulas)

[8.2 Example: Circular Arc](#example-circular-arc)

[8.3 Clothoids](#_Toc177558596)

[8.4 C++ MAD Library](#_Toc177558597)

[8.5 MATLAB MODBAS CAR Library](#matlab-modbas-car-library)

[8.6 Cubic Splines](#_Toc177558599)

[8.7 Exercises](#exercises-3)

[9 Path Following Control](#_Toc177558601)

[9.1 Reference Signal Generator](#_Toc177558602)

[9.2 MATLAB MODBAS CAR Library](#_Toc177558603)

[9.3 Control Deviation Dynamics](#_Toc177558604)

[9.4 Feedback Controller](#_Toc177558605)

[9.5 Controllability](#controllability)

[9.6 Feedforward Controller](#_Toc177558607)

[9.7 Exercises](#exercises-4)

[A. Clothoids](#_Toc177558609)

[B. Dictionary](#_Toc177558610)

[C. Literature](#_Toc177558611)

# Introduction

Only available in German book (Tränkle, Modellbasierte Entwicklung mechatronischer Systeme: mit Software- und Simulationsbeispielen für autonomes Fahren, 2021).

# Model-Based Software Engineering

Only available in German book (Tränkle, Modellbasierte Entwicklung mechatronischer Systeme: mit Software- und Simulationsbeispielen für autonomes Fahren, 2021).

# Lab Project Mini-Auto-Drive (MAD)

Mini-Auto-Drive (MAD) is a *mini plant*[^1] for

[^1]: *Mini plants* are established, widely spread means for prototyping and testing new processes in chemical and process engineering.

-   prototyping
-   and testing

automated driving functions and autonomous driving (self driving).

Self-driving cars and manually driven cars operate on the same track. The cars are radio-controlled cars (RC cars) of the MiniZ series in the 1:24 scale.

## System Overview

The cars are observed by a top-view infrared-band digital camera that is connected to a Linux PC. Each car is tagged by infrared markers. The coordinates and orientation angles of the individual cars are computed by computer vision. All routines developed in this course run on the Linux PC. The *automated driving functions* are realized as part of the *driving software* of the mini plant (see Figure ‎3.1). The accompanying lab project of this course focuses on the development of the driving functions

-   map provider,
-   motion planning,
-   motion control (longitudinal and lateral control)

![](media/824a25dd62b345b3ab0fc8d74649dc52.jpeg)

Figure ‎3.1: Mini-Auto-Drive (MAD) system

The control signals "pedals" and "steering" for each car are transmitted via Bluetooth Low Energy (BLE). For this, the PC transmits the control signals via USB to a Nordic microcontroller board, which is a gateway from serial communication on USB to BLE and vice versa. In return, the cars transmit their health state, error lists, battery status etc. over BLE and USB to the PC.

So, in MAD the whole "intelligence" of the cars is implemented on the Linux PC which controls all cars. The individual cars are "stupid", have no on-board sensors for autonomous driving, and just react on the manipulation. However, the cars implement steering and motor control including energy management and diagnostic functions. The purpose of MAD is to concentrate on the development and testing of motion control, AI (artificial intelligence) and safety functions.

Next to the real 1:24 track, the setup of MAD allows the development and testing of the functions in a *virtual simulation environment*. Without making any changes, these functions can be executed in the real environment after successful testing in the simulation environment.

## Model-Based Software Engineering

The objectives of the lab project in this course are

-   to develop automated driving functions in order to operate one car on a racetrack,
-   to test the driving functions in the simulation environment of MAD,
-   to test the driving functions on the real MAD system.

The lab project applies the model-based software engineering (MBE) process (Ansgar Meroth, 2014). Depending on modeling / programming platform, C++ or MATLAB/Simulink, two different workflows are applied. Figure Figure ‎3.2 illustrates the V process model of the MBE.

![](media/b02d4fc91c164ce3e626baaaf6a18fdf.png)

Figure ‎3.2: V process model of the model-based software engineering (MBE) process. SWC are software components developed in the V model.

For C++, the process steps of the workflow are:

1.  The vehicle dynamics are modeled and simulated in C++, Boost odeint[^2] and ROS.
2.  The control functions are designed and parameterized by means of control engineering and analyzed in closed-loop in MATLAB Control System Toolbox (CST).
3.  The control functions are implemented in C++ and ROS2 on the Linux PC.
4.  The control functions are tested in the loop with the vehicle dynamics simulation in ROS (*Software-in-the-Loop Tests SiL*).
5.  The control functions are tested on the real MAD system without any changes of code (*driving tests*).

[^2]: Boost is a C++ library that covers a whole range of routines for I/O, process management, data structures, numerics etc.

For MATLAB/Simulink, the workflow is:

1.  The vehicle dynamics are modeled and simulated in Simulink.
2.  The control functions are designed and parameterized by means of control engineering and tested in closed-loop in MATLAB Control System Toolbox (CST).
3.  The control functions are modeled in Simulink.
4.  The control functions are tested in the loop with the vehicle dynamics simulation in Simulink (*Model-in-the-Loop Tests MiL*).
5.  Embedded Coder and Robot System Toolbox of MATLAB/Simulink automatically generate C/C++ code from the Simulink model and implement the control functions in ROS.
6.  The control functions are tested on the real MAD system.

The control functions to be developed are:

-   speed control,
-   parking position control,
-   path following control,
-   including reference signal generation for path following control.

In this course, the vehicle dynamics are modeled by dynamical bicycle models. If any of the above process steps fails, e.g. verification or validation steps fail, then the workflow can be resumed at any earlier process step.

The course results will be longitudinal speed, longitdudinal position and lateral path following controllers of one car on a parking garage track depicted in Figure ‎3.3 or similar.

![](media/0ff695d2834d6e231e3aeec8d19595c9.jpeg)

Figure ‎3.3: Parking garage track of MAD

## Software Architecture

The Linux PC runs the *Robot Operating System (ROS2)*. In ROS, software components run as so-called *ROS nodes* that communicate via *ROS topics* (message-based) or *ROS services* (procedure-based).

The simulation model and the automated driving functions of MAD will be implemented as ROS nodes, as the course progresses.

Real MAD System

On the real MAD system, the *ROS nodes* in Figure ‎3.4 constitute the automated driving functionality:

-   rcnode manipulates the cars via Bluetooth-Low-Energy (BLE). The only subscribed topic of rcnode receives messages of type CarInputs on ROS topic /mad/car0/carinputs with the elements
    -   cmd to command the driving mode halt, forward drive, reverse drive or slow drive
    -   pedals to manipulate the electric motor for thrust and braking
    -   steering for steering
-   visionnode reads in the digital images of the digital camera with a *sample time* of 22 ms (approximate *sampling rate* of 45 Hz). visionnode sends messages of type CarOutputsList on ROS topic /mad/vision/caroutputs. The message type CarOutputsList contains a dynamic array of CarOutputs messages with following elements:
    -   carid for car identification
    -   position s in two Cartisian coordinates [m]
    -   yaw angle psi [rad]
-   locatenode is responsible for self-localization and environment perception. It computes the slip angle and the speed of each car. For this, carlocate reads in the CarOutputsList messages from visionnode and sends messages of type CarOutputsExtList on ROS topic /mad/locate/caroutputsext. The message type CarOutputsExtList contains a dynamic array of CarOutputsExt messages CarOutputsExt has the same elements as CarOutputs plus
    -   slip angle beta [rad]
    -   absolute car speed v [m/s] of the rear-axle center [^3]
-   tracknode defines the map of the environment which can be arbitrarily composed of track segments, such as straights and curves. tracknode advertises ROS services (e.g. /mad/get_waypoints) for the computation of occupancy grids and reference paths for motion planning and control.
-   carctrlnode implements motion control and executes the key functions of automated driving, namely
    -   longitudinal speed control,
    -   longitudinal position control,
    -   lateral path following control.

        These functions control the car on the pre-defined path. Speed control, position control and path following functions generate CarInputs messages to manipulate the car via rcnode. The relevant process variables are read in from messages of type CarOutputsExtList on topic /mad/locate/caroutputsext.

-   carctrlnode further reads in DriveManeuver messages on topic /mad/car0/ maneuver. These driving maneuver messages define the reference path, the reference speed or the reference position for the control functions. In this lab project two types driving maneuvers are implemented:
    -   speed-controlled path following,
    -   position control on references path
-   In order to facilitate the generation of DriveManeuver messages the Python script send_maneuver.py is available, which invokes the service /mad/get_waypoints of node tracknode to generate the reference paths for motion control. . This Python script substitutes the self-driving functions *navigation / global planning* and *maneuver management / local planning*. The development of these driving functions is not covered by this course.

[^3]: The additional element driven arc length x [m] is not processed in this course.

It is important to note that the ROS nodes rcnode, visionnode, locatenode and tracknode run only once as singletons, whereas the ROS carctrlnode ist multiply instantiated, one instance per each individual car.

The core software components rcnode, visionnode, locatenode and tracknode are readily available. The only software components to be developed (and tested in SiL) in this course are: carctrlnode and carsimnode (see below). The ROS node tracknode needs to be adapted in order to realize different maps.

![](media/975d90a75d257f51c4262a3e2202bbba.png)

Figure ‎3.4: ROS nodes and ROS topics of MAD. If Simulink is applied then ROS node carctrlnode is replaced by ROS node c07_car0 with identical interfaces.

The ROS nodes of Figure ‎3.4 implement a selection of the self-driving function:

-   tracknode: *map provider, traffic (race) management*
-   visionnode: *environment perception*
-   locatenode: *self-localization*
-   carctrlnode: *motion control*

ROS node carctrlnode will be implemented in C++. If MATLAB/Simulink is applied in the lab project, then carctrlnode will be replaced by ROS node c07_car0 which will be automatically generated from the Simulink model c07_car0.slx. The interfaces (receive and send messages) of c07_car0 and carctrlnode are identical.

The driving functions *navigation, maneuver management, health monitoring* and *safety management* are not covered in this course. The low-level control functions *steering / drivetrain / braking control / energy management* are implemented in the onboard control system of the car.

MAD Simulation Setup in ROS

In this course, ROS is further applied as a *virtual simulation environment*. The above setup is replaced by a simulation setup. In particular, ROS nodes rcnode and visionnode are replaced by the ROS node carsimnode that simulates vehicle dynamics. In the same way as carctrlnode, carsimnode can be multiply instantiated, one instance for each individual car.

The other ROS nodes locatenode, tracknode, and carctrlnode (or c07_car0) run without any changes. This setup makes it possible to test these nodes in Software-in-the-Loop simulations (SiL).

Figure ‎3.5: ROS simulation setup of MAD

Since the cars are now simulated, carsimnode knows the internal dynamic state of the vehicle dynamics including car speed and slip angle. For debugging purposes, carsimnode sends the message CarOutputsExt that contains this internal state. This is a great advantage of simulation. Please note that this internal state cannot not be directly measured on the real cars in the MAD setup.

MATLAB/Simulink

As an alternative to C++, MATLAB/Simulink can be applied to generate ROS node c07_car0 which replaces carctrlnode. This enables the model-based software engineering of the main self-driving functions for *motion control* in Simulink.

ROS node c07_car0 will be automatically generated by auto-code-generation from the Simulink model c07_car0.slx. This Simulink model will utilize Subscribe and Publish blocks of the Simulink ROS Toolbox blockset to receive messages on topics /mad/car0/ maneuver and /mad/locate/caroutputsext and to send messages on topic /mad/car0/carinputs in the same way as carctrlnode.

For code-generation, Embedded Coder and ROS Toolbox , which are integrated in MATLAB/Simulink, are applied. Embedded Coder automatically generates C/C++ code for real-time or embedded devices from Simulink/Stateflow models or MATLAB code.

Figure ‎3.6 depicts this code-generation workflow. The automated driving functions are modeled as part of the Simulink subsystem Motion Control. This subsystem is referenced in or copied to the top-level Simulink model c07_car0.slx. Embedded Coder together with ROS Toolbox generate C/C++ code that implements ROS node c07_car0. This C/C++ code is compiled and linked by the GNU C/C++ toolchain available on the MAD Linux PC.

Figure ‎3.6: Simulink model madctrl_d1.slx and code-generation of ROS node madctrl_d1 that replaces carctrl_node

In addition, Simulink will be applied without ROS for vehicle simulation. In order to test subsystem Motion Control solely in Simulink by Model-in-the-Loop (MiL) simulation, the Simulink model shown in Figure ‎3.7 will be created in Simulink that corresponds to the ROS simulation setup of Figure ‎3.5. Without any code generation of C/C++ code or implementation as a ROS node, the automated driving functions can be tested by running Simulink simulations on any PC with a MATLAB/Simulink installation.

Figure ‎3.7: MATLAB/Simulink simulation setup of MAD

The subsystems of this Simulink model in Figure ‎3.7 correspond to the ROS nodes in Figure ‎3.5:

-   Subsystem Motion Control models the speed, position and path following control functions corresponding to ROS node carctrlnode.
-   Subsystem Vehicle Dynamics models the vehicle dynamics model corresponding to ROS node carsimnode.
-   Subsystems Vision and Localization directly feed through the car poses, compute the car speed and side-slip angle, and collect these signals in array bus signals.

In contrast to the ROS simulation setup that uses ROS messages for communication, this Simulink model uses bus signals CARINPUTS, CAROUTPUTSEXT, and DRIVEMANEUVER for signal-based communication.

## Required Lab Results

Documents to be turned in as results of the exercises and lab projects are:

-   Approaches, key solution steps and results of all exercises documented in Word, LaTeX or Powerpoint and exported to PDF
-   C++ source code for the ROS nodes or MATLAB/Simulink models as required in the exercises
-   Simulation results of open-loop vehicle dynamics and closed-loop control simulations inserted as diagrams in the above document, as required in the exercises

The PDF document, the C++ source code files and MATLAB/Simulink models shall be zipped in a single file named

-   \<name1\>_\<name2\>.zip (e.g., maier_mueller.zip)

and uploaded to the online course.

Either C++ or MATLAB/Simulink

-   Exercise sections denoted by [C++] must be solved in the case of applying C++/ROS in the lab project,
-   whereas sections denoted by [Simulink] must be solved if Simulink is applied.
-   Sections denoted by [C++/Simulink] must be solved in both cases.

## Robot Operating System (ROS)

"The Robot Operating System (ROS2) is a flexible framework for writing robot software. It is a collection of tools, libraries and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms." (cited from <http://www.ros.org/about-ros/> ).

-   ROS is a complex framework with tons of available tools and libraries.
-   It has a huge user community in industrial robotics and self-driving applications.
-   Both universities and industrial companies apply ROS …
-   … both in research and in product development.

### Features

ROS has the following key features:

-   Tools for robot software development and execution
-   Software library of ROS nodes, topics, nodelets, …
-   ROS nodes for sensor / actuator I/O
    -   cameras, laser scanners, odometers, GPS, …
    -   robots such as Universal Robots, Kuka, …
-   ROS nodes for robot control
    -   location
    -   map generation
    -   motion planning
    -   motion control
-   Diagnostics
-   Experiment environment rqt
-   Simulation environment rviz
-   Multi-body simulation environments
-   Recording and playback of communication messages
-   ROS nodes may be implemented in C++, Python, MATLAB, Simulink or LISP
-   CMake build environment (colcon, ament)

In this course

-   Only a subset of ROS is addressed
-   No preexisting ROS nodes from the community are applied

The main reasons for not using preexisting ROS nodes are

-   No availability of existing ROS code for high-speed computer vision and control
-   No availability of suitable motion planning and following functions for self-driving cars
-   Educational purposes

For reference documentation, tutorials and user forums, in order to deepen or extend the course agenda, please visit <http://www.ros.org/> .

### ROS Nodes

A *ROS node* is the major software component in ROS. It is encapsulated and typically runs in a Linux user-space *process*. Its interfaces to the outside world are:

-   *ROS topics*: message passing by broadcasting with publisher-subscriber communication paradigm:
    -   nodes publish topics to send messages
    -   nodes subscribe to topics to receive messages
    -   broadcasting means that a message sent by one node is visible to all nodes and may be received by all nodes
    -   one to many nodes can publish on the same topic
    -   one to many nodes can subscribe on the same topic
-   *ROS services*: remote procedure calls
-   *ROS parameters*: parameters from the command line, launch XML files or YAML files
-   *ROS info* for debugging (similar to printf in C)

A ROS node may

-   create additional threads next to the process thread
-   access file I/O, memory I/O, peripheral I/O by using user-space libraries

In particular, *device drivers* in ROS are special ROS nodes that access peripherals. For instance, rcnode in Figure ‎3.4 is a device driver that communicates to a Nordic Bluetooth board via USB.

# Basics in Signals and Systems

Only available in German book (Tränkle, Modellbasierte Entwicklung mechatronischer Systeme: mit Software- und Simulationsbeispielen für autonomes Fahren, 2021).

# Vehicle Dynamics Simulation

Dynamic system models are mathematical representations of dynamic system behavior. Dynamic system models are required for two purposes

-   controller design
-   vehicle simulation for MiL / SiL / HiL testing

The design of closed-loop and open-loop control systems is based on a profound knowledge of the dynamic system behavior. Dynamic models are analyzed and simulated to gain this knowledge. Depending on the model characteristics, the architecture including the components of the controllers is designed and the control parameters are computed.

In MiL/SiL/HiL testing, control systems are tested in-the-loop with dynamic system models. MiL/SiL/HiL testing are obligatory quality gates in the development of embedded software for automotive systems.

For the development of control systems for in driving, the following types of vehicle dynamics models are required

-   longitudinal dynamics models
-   vehicle dynamics models

This section firstly discusses the derivation and formulation of a longitudinal dynamics model for the electrically propelled RC car in the lab project. Secondly, vehicle dynamics models of different granularity and accuracy are presented, namely

-   bicycle kinematics model for rear axle (Reeds & Shepp, 1990)
-   bicycle kinematics model for front axle
-   bicycle kinematics model for arbitrary points on longitudinal car axis, including center-of-gravity (COG)
-   dynamics model with inertia and tire forces

## Longitudinal Dynamics

The longitudinal dynamics model describes the dynamics of the vehicle speed. The input signal is the control signal of the motor electronics.

Signal Flow Chart

The following figure depicts the signal flow chart of the longitudinal vehicle dynamics.

Figure ‎5.1: Signal-flowchart of longitudinal vehicle dynamics

Input Signals:

| **Math. Symbol** | **Physical Data Type** | **Domain / Unit**                                | **Comment**                                                                                                                    |
|------------------|------------------------|--------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|
| $$ u_{c m d} $$  | uint8                  | { CMD_HALT, CMD_FORWARD, CMD_REVERSE, CMD_SLOW } | Command for driving mode: halt, forward drive, reverse drive for speed control or slow drive for longitudinal position control |
| $$ u_{n} $$      | real                   | $$ \lbrack - 1 ; 1 \rbrack $$                    | Normalized DC motor voltage (control variable)                                                                                 |

Depending on the driving mode, the onboard-electronics of the MAD vehicles limit the motor signal $$u_{n}$$. This limitation is required for thermal protection of the DC motor and motor electronics. The driving mode is commanded by the input signal $$u_{c m d}$$. In addition, CMD_HALT is commanded for emergency stops in case of critical faults, errors or failures in the MAD system. The logic that limits $$u_{n}$$ in dependence of the driving mode is as follows:

$$
u_{n} : = \begin{cases}0 \  \  \  ; \  \  \  u_{c m d} = C M D \_ H A L T \\ 1 \  \  ; \  \  \  u_{c m d} = C M D \_ F O R W A R D \land u_{n} > 1 \\ 0 \  \  \  ; \  \  \  u_{c m d} = C M D \_ F O R W A R D \land u_{n} < 0 \\ 0 \  \  ; \  \  u_{c m d} = C M D \_ R E V E R S E \land u_{n} > 0 \\ - 1 \  \  ; u_{c m d} = C M D \_ R E V E R S E \land u_{n} < - 1 \\ u_{n} \  \  ; \  \  o t h e r w i s e\end{cases}
$$

For instance, in case of forward drive $$u_{c m d} = C M D \_ F O R W A R D$$ the motor signal $$u_{n}$$ is limited to the interval $$u_{n} \in \lbrack 0 ; 1 \rbrack$$. Whereas for reverse drive $$u_{c m d} = C M D \_ R E V E R S E$$ the interval $$u_{n} \in \lbrack - 1 ; 0 \rbrack$$ is valid.

Output Signals:

| **Math. Symbol** | **Physical Data Type** | **Domain / Unit**               | **Comment**                                                |
|------------------|------------------------|---------------------------------|------------------------------------------------------------|
| $$ v_{r} $$      | real                   | $$ [ - 5 ; 5 ] \frac{m}{s} \ $$ | Vehicle speed in longitudinal direction (process variable) |

The plant dynamics consists of the components

-   motor electronics
-   DC motor with transmission
-   longitudinal dynamics

The individual components communicate via the following signals:

| **Math. Symbol** | **Physical Data Type** | **Physical Unit**       | **Comment**                   |
|------------------|------------------------|-------------------------|-------------------------------|
| $$ u_{m} $$      | real                   | $$ V \ $$               | DC motor voltage              |
| $$ M_{r} $$      | real                   | $$ N m \ $$             | Rear axle torque              |
| $$ \omega_{r} $$ | real                   | $$ \frac{r a d}{s} \ $$ | Angular velocity of rear axle |

The auxiliary variables $$M_{r}$$ and $$\omega_{r}$$ are needed during the modeling of the vehicle and do not occur in the final model.

Physical Architecture Diagrams

The following physical architecture diagrams form the basis of the model derivation:

-   the electrical network of the DC motor,
-   the powertrain
-   and the longitudinal movement

    Figure ‎5.2: Electrical network of DC motor

Figure ‎5.3: Mechanical architecture of powertrain

Figure ‎5.4: Mechanical architecture of longitudinal dynamics

Modeling Assumptions

For the plant dynamics model the following assumptions are made. The modeling assumptions lead to a simplified plant model for the subsequent controller design. The plant model represents the real vehicle with required precision.

-   The vehicle has a rear wheel drive.
-   The front wheels are rotating without friction.
-   The transmission friction $$M_{w}$$ is neglegted.
-   The total resistance force $$F_{w} = F_{w r} + \rho c_{w} A \overline{v}_{r} v_{r} \$$is constituted of
    -   wheel friction and curve resistance $$F_{w r}$$ which will be neglected in the following sections
    -   air resistance force that is linearized at constant speed $$\overline{v}_{r} = c o n s t$$: $$\frac{1}{2} \rho c_{w} A \  v_{r}^{2} s i g n \  v_{r} \approx \rho c_{w} A \overline{v}_{r} v_{r}$$
-   All bodies and axles are rigid.
-   The rear axle torque is distributed equally to both rear wheels.
-   There is no slip between the wheels and the road surface such that the rear axle torque $$M_{r}$$ is directly transformed to the propulsion force $$F_{r}$$.
-   The DC motor is a permanent magnet DC motor.
-   The inductance of the electrical network is neglected.
-   The motor controller is a linear step-down converter.
-   The manipulation of the motor has an overall dead time of $$T_{t}$$.
-   The DC motor voltage is limited by the negative and positive battery voltage $$- u_{\max}$$ and $$u_{\max}$$.
-   The battery is an ideal voltage source with $$u_{\max}$$.
-   The moments of inertia of the motor, transmission and wheels are neglected.

Model Parameters of the DC Motor

| **Math. Symbol** | **Physical Data Type** | **Comment**                                                |
|------------------|------------------------|------------------------------------------------------------|
| $$ R $$          | real                   | Ohm resistance of DC motor                                 |
| $$ c_{n} $$      | real                   | Speed constant of DC motor                                 |
| $$ c_{m} $$      | real                   | Torque constant                                            |
| $$ u_{g} $$      | real                   | Total transmission ratio of motor axle to rear wheel axle  |
| $$ u_{\max} $$   | real                   | Battery voltage / maximal DC motor input voltage           |
| $$ T_{t} $$      | real                   | Overall dead time of motor electronics and RC transmission |

Modell Parameters of Vehicle

| **Math. Symbol** | **Physical Data Type** | **Comment**  |
|------------------|------------------------|--------------|
| $$ m_{t o t} $$  | real                   | Total mass   |
| $$ r $$          | real                   | Wheel radius |

In Exercise 5.1 the linear dynamics model is derived and analyzed in MATLAB based on these given assumptions and parameters. The result of Exercise 5.1 is the following linear, ordinary differential equation (ODE) and initial condition (IC) of the longitudinal speed $$v_{r}$$:

|   | $$ \dot{v}_{r} ( t ) = - \frac{1}{T} v_{r} ( t ) - \frac{k_{d}}{T} F_{w r} ( t ) + \frac{k_{u}}{T} u_{n} \left( t - T_{t} \right) \  \  \  \  ; \  \  \  \  t > 0 $$ | (‎5.1) |
|---|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------|
|   | $$ v_{r} ( 0 ) = 0 \frac{m}{s} $$                                                                                                                                    |       |

This ODE denotes a PT1Tt dynamics with time constant $$T = 3 1 6 m s$$, gain $$k_{u} = 2 . 5 1 \frac{m}{s}$$ and deadtime $$T_{t} = 1 1 0 m s$$. The input signal $$u_{n} \in [ - 1 ; 1 ]$$ is the normalized control signal of the electric drive for thrust and braking. The delays of the remote-control via Bluetooth Low Energy and of the motor electronics are modeled as the total delay time $$T_{t}$$.

## Bicycle Model for Rear Axle

In this section, a bicycle model is introduced that is applied to design and implement path following control in Chapter ‎9. This model only considers the vehicle kinematics and not the vehicle kinetics.

Characteristics:

-   motion equations for center point of rear axle
-   front-wheel steering
-   dynamic vehicle speed
-   one rigid body for total vehicle
-   neglected wheel suspension
-   neglected pitch and roll dynamics
-   no wheel slips
-   nonlinear model
-   non-holonomic model

Applications:

-   reference path planning or trajectory planning
-   path following and tracking controller design
-   nonlinear feedforward control

This model is related to the Reeds-Shepp car (Reeds & Shepp, 1990) where the car drives either forwards or reverse at defined constant speeds.

Dubins car (Dubins, 1957) is a special case of the Reeds-Shepp car where the car only drives forward.

Figure ‎5.5: Kinematics of the Reeds-Shepp car with the input signals $$\mathbf{v}_{\mathbf{r}}$$ and $$\mathbf{\delta}$$ in red color and the state variables $$\mathbf{s}_{\mathbf{r} \mathbf{1}} \mathbf{,} \mathbf{s}_{\mathbf{r} \mathbf{2}}$$ and $$\mathbf{\psi}$$ in green color.

State-Space Model

The state-space space of Reeds-Shepp car is given as:

| ODE     | $$ \underset{\dot{\mathbf{x}}}{\overset{\frac{d}{d t} \begin{pmatrix}s_{r 1} \\ s_{r 2} \\ \psi\end{pmatrix}}{︷}} = \underset{\mathbf{f} \mathbf{(} \mathbf{x} \mathbf{,} \mathbf{u} \mathbf{)}}{\overset{\begin{pmatrix}v_{r} \cos \psi \\ v_{r} \sin \psi \\ \frac{v_{r}}{l} \tan \delta\end{pmatrix}}{︷}} \  \  \  \  ; t > 0 $$ | (‎5.2) |
|---------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------|
| IC      | $$ \mathbf{x} ( 0 ) = \left( s_{r 1 0} \  s_{r 2 0} \  \psi_{0} \right)^{T} \  \  \  \  ; \  \  t = 0 $$                                                                                                                                                                                                                              |       |
| outputs | $$ \mathbf{y} ( t ) = \mathbf{x} ( t ) \  \  \  \  ; \  \  t \geq 0 $$                                                                                                                                                                                                                                                                |       |

The signals and parameter of this model are:

| **Input Signals**                               | **Symbol**                         | **Unit**          | **(Initial) Value** |
|-------------------------------------------------|------------------------------------|-------------------|---------------------|
| speed of rear axle center point                 | $$ u_{1} = v_{r} $$                | $$ \frac{m}{s} $$ |                     |
| steering angle of front axle (Ackermann angle)  | $$ u_{2} = \delta $$               | $$ r a d $$       |                     |
|                                                 |                                    |                   |                     |
| **State Vector and Output Signals**             |                                    |                   |                     |
| first Cartesian coordinate of rear axle center  | $$ {y_{1} = \  x}_{1} = s_{r 1} $$ | $$ m $$           | $$ s_{r 10} $$      |
| second Cartesian coordinate of rear axle center | $$ y_{2} = x_{2} = s_{r 2} $$      | $$ m $$           | $$ s_{r 20} $$      |
| yaw angle of vehicle                            | $$ y_{3} = x_{3} = \psi $$         | $$ r a d $$       | $$ \psi_{0} $$      |
|                                                 |                                    |                   |                     |
| **Parameters**                                  |                                    |                   |                     |
| wheelbase                                       | $$ l $$                            | $$ m $$           | $$ 0 . 0 9 9 $$     |

Signal Flow Chart

The following signal flow chart shows the nonlinear transfer element representing Reeds-Shepp car with vector inputs and outputs.

Figure ‎5.6: Dynamic transfer element of Reeds-Shepp car with input and output signals

Model Derivation

The model derivation is based on the kinematic relations of the rear axle and front axle velocities. As the vehicle is a rigid body, the orthogonals of all velocity vectors belonging to the vehicle meet in the *instant centre of rotation* $$M$$. The orthogonals and $$M$$ are depicted in color blue in Figure ‎5.5.

In particular, the *instant centre of rotation* $$M$$ holds for the rear axle and front axle center points. Since the wheel motions are without slip, the orientation angle of velocity $$v_{r}$$ of the rear axle center point matches to the yaw angle $$\psi$$, whereas the orientation of velocity $$v_{f}$$ of the axle center is given by the *Ackermann angle* $$\delta$$. Due to kinematic constraints, the left and right front wheels follow the *Ackermann condition*, where the steering angle of the inner-circle wheel is greater than the one of the outer-circle wheel.

From trigonometrics of the right-angle triangle at $$M$$, the following condition is readily derived:

|   | $$ \tan \delta = \frac{l}{r} $$ | (‎5.3) |
|---|---------------------------------|-------|

where $$l$$ is the wheelbase, i.e. the distance of the front axle to the rear axle, and $$r$$ is the radius of the instant circle of rotation of the rear axle center point w.r.t to $$M$$.

Further, the following point kinematics of rotation holds for translation speed $$v_{r}$$ and rotational angular velocity $$\dot{\psi}$$:

|   | $$ \dot{\psi} = \frac{v_{r}}{r} $$ | (‎5.4) |
|---|------------------------------------|-------|

Equation (‎5.3) is solved for *curvature* $$\kappa$$ which is the inverse of circle radius $$r$$:

|   | $$ \kappa = \frac{1}{r} = \frac{1}{l} \tan \delta $$ | (‎5.5) |
|---|------------------------------------------------------|-------|

Please note, for straight drive $$\kappa$$ equals to zero, whereas radius $$r$$ goes to plus / minus infinity.

Inserting (‎5.5) in (‎5.4) yields

|   | $$ \dot{\psi} = \frac{v_{r}}{l} \tan \delta $$ | (‎5.6) |
|---|------------------------------------------------|-------|

which is the third ODE in state-space model (‎5.2).

The first and second ODEs of (‎5.2) are readily derived from a vector decomposition of $$v_{r}$$:

|   | $$ \dot{s}_{r 1} = v_{r} \cos \psi $$ |   |
|---|---------------------------------------|---|
|   | $$ \dot{s}_{r 2} = v_{r} \sin \psi $$ |   |

It is important to note, that resulting state-space model (‎5.2)

-   is of pure kinematic nature,
-   thus, has no kinetic part
-   and its only required vehicle parameter is wheelbase $$l$$.

Furthermore, (‎5.2) is a non-holonomic system, since it contains the non-holomic constraint

|   | $$ \left| v_{r} \right| = \sqrt{\dot{s}_{r 1}^{2} + \dot{s}_{r 2}^{2}} $$  |   |
|---|----------------------------------------------------------------------------|---|

and no holomic constains.

## Bicycle Model for Front Axle

Figure ‎5.7: Kinematics of the Reeds-Shepp car with the input signals $$\mathbf{v}_{\mathbf{f}}$$ and $$\mathbf{\delta}$$ in red color and the state variables $$\mathbf{s}_{\mathbf{f} \mathbf{1}} \mathbf{,} \mathbf{s}_{\mathbf{f} \mathbf{2}}$$ and $$\mathbf{\psi}$$ in green color.

Reeds-Shepp car model of Section ‎5.2 defines states $$x_{1}$$ and $$x_{2}$$ as the Cartesian coordinates of the center point of the rear axle whose speed is given as input $$u_{1}$$. The following state-space model describes kinematics identical to Reeds-Shepp car, but considers $$x_{1}$$ and $$x_{2}$$ as the Cartesian coordinates of the center point of the front axle whose speed is now given as the input $$u_{1} = v_{f}$$ (see Figure ‎5.7). The other input $$u_{2}$$ is the steering angle $$\delta$$. The third state $$x_{3}$$ is the yaw angle $$\psi$$.

State-Space Model

| ODE | $$ \underset{\dot{\mathbf{x}}}{\overset{\frac{d}{d t} \begin{pmatrix}s_{f 1} \\ s_{f 2} \\ \psi\end{pmatrix}}{︷}} = \underset{\mathbf{f} \mathbf{(} \mathbf{x} \mathbf{,} \mathbf{u} \mathbf{)}}{\overset{\begin{pmatrix}v_{f} \cos ( \psi + \delta ) \\ v_{f} \sin ( \psi + \delta ) \\ \frac{v_{f}}{l} \sin \delta\end{pmatrix}}{︷}} \  \  \  \  ; t > 0 $$ | (‎5.7) |
|-----|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------|
| IC  | $$ \mathbf{x} ( 0 ) = \left( s_{f 10} \  s_{f 20} \  \psi_{0} \right)^{T} \  \  \  \  ; \  \  t = 0 $$                                                                                                                                                                                                                                                          |       |

**Model Derivation**

This variant of the bicycle model considers the kinematics of the front wheel. The third ODE is derived by vector decomposition of the front wheel speed $$u_{1} = v_{f}$$:

|   | $$ v_{r} = v_{f} \cos \delta = u_{1} \cos u_{2} $$ | (‎5.8) |
|---|----------------------------------------------------|-------|

where $$v_{r}$$ is the rear axle speed of the original model (‎5.2). Inserting (‎5.8) in (‎5.2) yields (‎5.7).

For constant $$u_{1}$$ and $$u_{2}$$, the front wheel moves on a circle with radius

|   | $$ r_{f} = \sqrt{r_{r}^{2} + l^{2}} = \sqrt{\frac{l^{2}}{\tan^{2} u_{2}} + l^{2}} = \frac{l}{\tan u_{2}} \sqrt{1 + \tan^{2} u_{2}} = \frac{l}{\tan u_{2} \cos u_{2}} = \frac{l}{\sin u_{2}} $$ |   |
|---|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

## Bicycle Model for Arbitrary Point on Longitudinal Axis

Figure ‎5.8: Kinematics of the Reeds-Shepp car with the input signals $$\mathbf{v}$$ and $$\mathbf{\delta}$$ in red color and the state variables $$\mathbf{s}_{\mathbf{1}} \mathbf{,} \mathbf{s}_{\mathbf{2}}$$ and $$\mathbf{\psi}$$ in green color.

Once more the kinematics of Reeds-Shepp car are considered. Now, the following variant of the bicycle model considers the kinematics of an arbitrary point on the longitudinal center line of the car whose total speed $$v \$$is given by the input $$u_{1}$$. The other input $$u_{2}$$ is still the steering angle $$\  \delta$$ and the third state $$x_{3}$$ is the yaw angle $$\psi$$.

State-Space Model

| ODE     | $$ \underset{\dot{\mathbf{x}}}{\overset{\frac{d}{d t} \begin{pmatrix}s_{1} \\ s_{2} \\ \psi\end{pmatrix}}{︷}} = \underset{\mathbf{f} \mathbf{(} \mathbf{x} \mathbf{,} \mathbf{u} \mathbf{)}}{\overset{\begin{pmatrix}v \cos \left\lbrack \psi + {a t a n} \left( \frac{a}{l} \tan \delta \right) \right\rbrack \\ v \sin \left\lbrack \psi + {a t a n} \left( \frac{a}{l} \tan \delta \right) \right\rbrack \\ \frac{v}{l} \frac{\tan \delta}{\sqrt{1 + \frac{a^{2}}{l^{2}} \tan^{2} \delta}}\end{pmatrix}}{︷}} \  \  \  \  ; t > 0 $$ | (‎5.9) |
|---------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------|
| IC      | $$ \mathbf{x} ( 0 ) = \left( s_{10} \  s_{20} \  \psi_{0} \right)^{T} \  \  \  \  ; \  \  t = 0 $$                                                                                                                                                                                                                                                                                                                                                                                                                                       |       |
| outputs | $$ \mathbf{y} ( t ) = \begin{pmatrix}s_{1} - a \cos \psi \\ s_{2} - a \sin \psi \\ \psi\end{pmatrix} \  \  \  \  ; \  \  t \geq 0 $$                                                                                                                                                                                                                                                                                                                                                                                                     |       |

The distance of this point to the rear axle is denoted by the constant parameter $$a$$. Models (‎5.2) and (‎5.7) are special cases of (‎5.9) with $$a = 0$$ or $$a = l$$, respectively. This arbitrary point may, for instance, be the center of gravity (COG) or even a virtual point in front of the car that is commonly considered as the control state in path following control applications.

Model Derivation

The third ODE in (‎5.9) is derived by considering the *slip angle* $$\beta$$ of this point:

|   | $$ \tan \beta = \frac{a}{r} = \frac{a}{v_{r}} \dot{\psi} = \frac{a}{l} \tan \delta = \frac{a}{l} \tan u_{2} $$ | (‎5.10) |
|---|----------------------------------------------------------------------------------------------------------------|--------|

here $$r$$ is the radius of the circle on which the center point of the rear axle is temporarily moving.

Vector decomposition of the total point speed $$u_{1 \ } = v$$ leads to

|   | $$ v_{r} = v \cos \beta = \frac{v}{\sqrt{1 + \tan^{2} \beta}} = \frac{v}{\sqrt{1 + \frac{a^{2}}{l^{2}} \tan^{2} u_{2}}} $$ | (‎5.11) |
|---|----------------------------------------------------------------------------------------------------------------------------|--------|

Inserting (‎5.11) into (‎5.2) yields (‎5.9).

State-Space Model Extended by Longitudinal Dynamics

Now, this kinematics model and the longitudinal dynamics model are combined to one single state-space model, where the longitudinal dynamics as a result of Exercise 5.1 by differential equation (‎5.1).

The mini car in the Mini-Auto-Drive (MAD) lab experiment has the normalized steering input $$\delta_{n} \in [ - 1 ; 1 ]$$ as the second control signal next to the normalized motor control signal $$u_{n}$$of ODE (‎5.1). The steering angle $$\delta$$ is obtained by scaling $$\delta_{n}$$ by the maximum steering angle $$\delta_{\max}$$. Further, the steering delay $$T_{t} = 1 0 0 m s$$ is considered:

|   | $$ \delta ( t ) = \delta_{\max} \cdot \delta_{n} \left( t - T_{u t} \right) $$ | (‎5.12) |
|---|--------------------------------------------------------------------------------|--------|

By integrating rear axle speed $$v_{r}$$, the path length of the driven trajectory of the rear axle center, i.e., the driving distance, is defined as:

|   | $$ \dot{x} = v \cos \beta = v_{r} \  \  \  ; \  \  \  \  t > 0 $$ | (‎5.13) |
|---|-------------------------------------------------------------------|--------|

Next to the car position $$s_{1} , s_{2} ,$$ yaw angle $$\psi$$ and vehicle speed $$v$$, path length $$x$$ and slip angle $$\beta$$ are generated as output signals. Slip angle $$\beta$$ is computed from (‎5.10):

|   | $$ \beta = {a t a n} \left\lbrack \frac{a}{l} \tan \left( \delta_{\max} \delta_{n} \left( t - T_{u t} \right) \right) \right\rbrack $$ | (‎5.14) |
|---|----------------------------------------------------------------------------------------------------------------------------------------|--------|

Augmenting above state-space model (‎5.9) by equations (‎5.1) to (‎5.14) yields the complete state-space model for the vehicle dynamics:

| ODE     | $$ \underset{\dot{\mathbf{x}}}{\overset{\frac{d}{d t} \begin{pmatrix}\begin{matrix}v_{r} \\ s_{1} \\ s_{2} \\ \psi \\ x\end{matrix}\end{pmatrix}}{︷}} = \underset{\mathbf{f} \mathbf{(} \mathbf{x} \mathbf{,} \mathbf{u} \mathbf{)}}{\overset{\begin{pmatrix}\begin{matrix}- \frac{1}{T} v_{r} ( t ) + \frac{k_{u}}{T} u_{n} \left( t - T_{u t} \right) \\ \frac{v_{r}}{\cos \beta} \cos ( \psi + \beta ) \\ \frac{v_{r}}{\cos \beta} \sin ( \psi + \beta ) \\ \frac{v_{r}}{l} \tan \left( \delta_{\max} \delta_{n} \left( t - T_{u t} \right) \right)\end{matrix} \\ v_{r}\end{pmatrix}}{︷}} \  \  \  \  ; t > 0 $$  with $$\beta ( t ) = {a t a n} \left\lbrack \frac{a}{l} \tan \left( \delta_{\max} \delta_{n} \left( t - T_{u t} \right) \right) \right\rbrack$$ | (‎5.15) |
|---------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| IC      | $$ \mathbf{x} ( 0 ) = \left( v_{0} \  s_{10} \  s_{20} \  \psi_{0} \  x_{0} \right)^{T} \  \  \  \  ; \  \  t = 0 $$                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |        |
| outputs | $$ \mathbf{y} ( t ) = \begin{pmatrix}s_{1} \left( t - T_{y t} \right) - a \cos {\psi \left( t - T_{y t} \right)} \\ s_{2} \left( t - T_{y t} \right) - a \sin \psi \left( t - T_{y t} \right) \\ \psi \left( t - T_{y t} \right)\end{pmatrix} \  \  \  \  ; \  \  t \geq 0 $$                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |        |

The second and third ODEs of (‎5.15) can be reformulated as:

$$
\dot{s}_{1} = \frac{v_{r}}{\cos \beta} \cos ( \psi + \beta ) = \frac{v_{r}}{\cos \beta} \left( \cos \psi \cos \beta - \sin {\psi \sin \beta} \right) = v_{r} \cos \psi - v_{r} \sin \psi \tan \beta = \underset{v_{c 1}}{\overset{v_{r}}{︷}} \cos \psi - \underset{v_{c 2}}{\overset{v_{r} \frac{a}{l} \tan \left( \delta_{\max} \delta_{n} \left( t - T_{u t} \right) \right)}{︷}} \sin \psi
$$

$$
\dot{s}_{2} = \frac{v_{r}}{\cos \beta} \sin ( \psi + \beta ) = \frac{v_{r}}{\cos \beta} \left( \sin \psi \cos \beta + \cos \psi \sin \beta \right) = v_{r} \sin \psi + v_{r} \cos \psi \tan \beta = \underset{v_{c 1}}{\overset{v_{r}}{︷}} \sin \psi + \underset{v_{c 2}}{\overset{v_{r} \frac{a}{l} \tan \left( \delta_{\max} \delta_{n} \left( t - T_{u t} \right) \right)}{︷}} \cos \psi
$$

The first two outputs equations compute the Cartesian rear axle center position by geometric relations.

The parameters of the mini cars in the MAD lab experiment are:

| **Parameters**                                                            | **Symbol**          | **C++ Symbol** | **Simulink Symbol** | **Unit**          | **Value**                                             |
|---------------------------------------------------------------------------|---------------------|----------------|---------------------|-------------------|-------------------------------------------------------|
| maximum steering angle                                                    | $$ \delta_{\max} $$ | deltaMax       | P_p_delta_max       | $$ r a d $$       | $$ \frac{2 1 . 5 8 {^\circ}}{1 8 0 {^\circ}} \pi \ $$ |
| wheelbase                                                                 | $$ l $$             | L              | P_p_l               | $$ m $$           | $$ 0 . 0 9 9 $$                                       |
| distance of COG to rear axle                                              | $$ a = l_{r} $$     | Lr             | P_p_lr              | $$ m $$           | $$ 0 . 0 5 0 $$                                       |
| gain of longitudinal dynamics                                             | $$ k_{u} $$         | K              | P_p_k               | $$ \frac{m}{s} $$ | $$ 2 . 5 1 $$                                         |
| time constant of longitudinal dynamics                                    | $$ T $$             | T              | P_p_T               | $$ m s $$         | $$ 316 $$                                             |
| dead time of inputs (mainly, Bluetooth communication delay)               | $$ T_{u t} $$       | uTt            | P_p_uTt             | $$ m s $$         | $$ 44 $$                                              |
| dead time of outputs (mainly, camera acquisition and communication delay) | $$ T_{y t} $$       | outputTt       | P_p_output_Tt       | $$ m s $$         | $$ 66 $$                                              |

The input signals are:

| **Input Signal**                                         | **Symbol**       | **ROS Message Element / Simulink Bus Element** | **Unit**                                     |
|----------------------------------------------------------|------------------|------------------------------------------------|----------------------------------------------|
| gas and braking pedals: normalized input signal of motor | $$ u_{n} $$      | pedals                                         | $$ u_{n} \in \lbrack - 1 ; 1 \rbrack $$      |
| normalized steering angle                                | $$ \delta_{n} $$ | steering                                       | $$ \delta_{n} \in \lbrack - 1 ; 1 \rbrack $$ |

In MAD both input signals are transmitted in the ROS message mbmadmsgs::CarInputs on ROS topic /mad/car0/carinputs (see Figure ‎3.5) with a sample time of $$2 2 m s$$. In Simulink, the signal bus CARINPUTS is modeled that contains the same elements (see Figure ‎3.7).

The five state variables of the dynamic state-space-model of $$5^{t h}$$ order are:

| **State Variable**                     | **Symbol**          | **C++ Symbol** | **Unit**          |
|----------------------------------------|---------------------|----------------|-------------------|
| rear axle speed                        | $$ v_{r} = x_{1} $$ | x.at(0)        | $$ \frac{m}{s} $$ |
| first Cartesian coordinate             | $$ s_{1} = x_{2} $$ | x.at(1)        | $$ m $$           |
| second Cartesian coordinate            | $$ s_{2} = x_{3} $$ | x.at(2)        | $$ m $$           |
| yaw angle                              | $$ \psi = x_{4} $$  | x.at(3)        | $$ r a d $$       |
| driven path length or rear axle center | $$ x = x_{5} $$     | x.at(4)        | $$ m $$           |

The output signals are transmitted in the ROS message mbmadmsgs::CarOutputs on topic /mad/car0/caroutputs (or in the corresponding Simulink signal bus CAROUTPUTS) with a sample time of $$2 2 m s$$:

| **Output Signal**                               | **Symbol / Equation**                                                      | **ROS Message Element / Simulink Bus Element** | **Unit**    |
|-------------------------------------------------|----------------------------------------------------------------------------|------------------------------------------------|-------------|
| first Cartesian coordinate of rear axle center  | $$ {y_{1} ( t ) = s}_{1} ( t - T_{y t} ) - a \cos \psi ( t - T_{y t} ) $$  | s.at(0)                                        | $$ m $$     |
| second Cartesian coordinate of rear axle center | $$ y_{2} ( t ) = \  s_{2} ( t - T_{y t} ) - a \sin \psi ( t - T_{y t} ) $$ | s.at(1)                                        | $$ m $$     |
| yaw angle                                       | $$ y_{3} ( t ) = \  \psi ( t - T_{y t} ) $$                                | psi                                            | $$ r a d $$ |

## Vehicle Dynamics Model with Inertia and Tire Forces

In contrast to the previous, purely kinematic bicycle models this section derives a dynamic bicycle model that considers the vehicle’s rotational inertia and wheel forces. This model is related to the well-known bicycle model of Riekert and Schunk (Riekert & Schunk, 1940).

Before introducing this model, the characteristics of Riekert and Schunk’s model are summarized:

-   the vehicle is one rigid body whose center of gravity (COG) is on the longitudinal vehicle axis
-   the vehicle moves on a horizontal plane
-   vertical, roll and pitch movements are neglected
-   the wheels are aggregated on the longitudinal vehicle axis
-   non-zero wheel slips
-   constant speed
-   linear model

The model of Riekert and Schunk is applied for:

-   stability analyses in static circular driving
-   parameter identification in static circular driving
-   control design at constant speed

Nonlinear Vehicle Dynamics Model

The linear bicycle model based on Riekert and Schunk (Riekert & Schunk, 1940) is not applied in this course, since

-   the vehicle speed in Mini-Auto-Drive (MAD) is not constant
-   the nonlinearity of the vehicle dynamics cannot be neglected in path following control

The underlying assumptions of the nonlinear vehicle dynamics model are:

-   the vehicle is one rigid body whose center of gravity (COG) is on the longitudinal vehicle axis
-   the vehicle moves on a horizontal plane
-   vertical, roll and pitch movements are neglected
-   the wheels are aggregated on the longitudinal vehicle axis
-   non-zero wheel slips
-   the longitudinal dynamics is a linear PT1 dynamics including latency

The *kinematics* of the center of gravity (COG) located on the center axis of the car are given by:

|   | $$ \dot{s}_{1} = v_{1} = v \cos ( \psi + \beta ) $$                                                                                                                                                                                                                                           |   |
|---|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|
|   | $$ \dot{s}_{2} = v_{2} = v \  s i n  ( \psi + \beta ) $$                                                                                                                                                                                                                                      |   |
|   | $$ \begin{pmatrix}v_{1} \\ v_{2}\end{pmatrix} = \begin{pmatrix}v \cos ( \psi + \beta ) \\ v \sin ( \psi + \beta )\end{pmatrix} $$                                                                                                                                                             |   |
|   | $$ \begin{pmatrix}\dot{v}_{1} \\ \dot{v}_{2}\end{pmatrix} = \begin{pmatrix}\dot{v} \cos ( \psi + \beta ) - v \left( \dot{\psi} + \dot{\beta} \right) \sin ( \psi + \beta ) \\ \dot{v} \sin ( \psi + \beta ) + v \left( \dot{\psi} + \dot{\beta} \right) \cos ( \psi + \beta )\end{pmatrix} $$ |   |
|   | $$ a_{c 1} = \begin{pmatrix}\cos \psi & \sin \psi\end{pmatrix} \cdot \begin{pmatrix}\dot{v}_{1} \\ \dot{v}_{2}\end{pmatrix} = \dot{v} \cos \beta - v \left( \dot{\psi} + \dot{\beta} \right) \sin \beta $$                                                                                    |   |
|   | $$ a_{c 2} = \begin{pmatrix}- \sin \psi & \cos \psi\end{pmatrix} \cdot \begin{pmatrix}\dot{v}_{1} \\ \dot{v}_{2}\end{pmatrix} = \dot{v} \sin \beta + v \left( \dot{\psi} + \dot{\beta} \right) \cos \beta $$                                                                                  |   |

or alternatively computed by superposing the translational and rotational kinematics of the of the center of gravity (COG):

|   | $$ \begin{pmatrix}a_{c 1} \\ a_{c 2} \\ 0\end{pmatrix} = \frac{d}{d t} \begin{pmatrix}v \cos \beta \\ v \sin \beta \\ 0\end{pmatrix} + \begin{pmatrix}0 \\ 0 \\ \dot{\psi}\end{pmatrix} \times \begin{pmatrix}v \cos \beta \\ v \sin \beta \\ 0\end{pmatrix} = \begin{pmatrix}\dot{v} \cos \beta - v \dot{\beta} \sin \beta - v \dot{\psi} \sin \beta \\ \dot{v} \sin \beta + v \dot{\beta} \cos \beta + v \dot{\psi} \cos {\beta \ } \\ 0\end{pmatrix} = \begin{pmatrix}\dot{v} \cos \beta - v \left( \dot{\psi} + \dot{\beta} \right) \sin \beta \\ \dot{v} \sin \beta + v \left( \dot{\psi} + \dot{\beta} \right) \cos \beta \\ 0\end{pmatrix} $$ |   |
|---|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

where $$a_{c 2}$$ is the lateral acceleration of the COG and $$a_{c 1}$$ its longitudinal acceleration in vehicle frame coordinates. The two angles $$\psi$$ and $$\beta$$ are the yaw and the slip angle of the vehicle, respectively.

Figure ‎5.9: The vehicle reduced to a bicycle as one rigid body. $$\mathbf{F}_{\mathbf{f}}$$ and $$\mathbf{F}_{\mathbf{r}}$$ are the lateral wheel forces that depend on the wheel slip angles $$\mathbf{\alpha}_{\mathbf{f}}$$ and $$\mathbf{\alpha}_{\mathbf{r}}$$. $$\mathbf{F}_{\mathbf{r x}}$$ is the propulsion force of the rear wheels.

The *conservation of momentum* in lateral direction is derived from above figure as:

|   | $$ m a_{c 2} = F_{f} \cos \delta + F_{r} $$                                                                                              |   |
|---|------------------------------------------------------------------------------------------------------------------------------------------|---|
|   | $$ m \left\lbrack \dot{v} \sin \beta + v \left( \dot{\psi} + \dot{\beta} \right) \cos \beta \right\rbrack = F_{f} \cos \delta + F_{r} $$ |   |

Note: Riekert and Schunk consider the special case of constant speed $$v = c o n s t$$, such that

|   | $$ m v \left( \dot{\psi} + \dot{\beta} \right) \cos \beta = F_{f} \cos \delta + F_{r} $$ |   |
|---|------------------------------------------------------------------------------------------|---|

1.  The *conservation of moment of momentum* is derived from above figure as:

|   | $$ J \  \ddot{\psi} = F_{f} l_{f} \cos \delta - F_{r} l_{r} $$ |   |
|---|----------------------------------------------------------------|---|

Alternative Formulation of Nonlinear Vehicle Dynamics Model

An alternative formulation of this vehicle dynamics model is obtained by introducing the velocity vector in vehicle frame coordinates:

|   | $$ v_{c 1} = v_{r} = v \cos \beta $$ |   |
|---|--------------------------------------|---|
|   | $$ v_{c 2} = v \sin \beta $$         |   |

where $$v \$$is the total speed of COG. Please note: the longitudinal component of the rear axle speed $$v_{r} \$$is equal to the longitudinal component $$v_{c 1}$$ of $$v$$.

From above equations follows:

|   | $$ \dot{s}_{1} = v_{1} = v_{c 1} \cos \psi - v_{c 2} \sin \psi $$                                                                                                                   |        |
|---|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
|   | $$ \dot{s}_{2} = v_{2} = v_{c 1} \sin \psi + v_{c 2} \cos \psi $$                                                                                                                   |        |
|   | $$ \dot{v}_{c 1} = \dot{v} \cos \beta - v \dot{\beta} \sin \beta = a_{c 1} + v \dot{\psi} \sin {\beta \ } = \frac{1}{m} ( - F_{f} \sin \delta + F_{r x} + m v_{c 2} \dot{\psi} ) $$ | (‎5.16) |
|   | $$ \dot{v}_{c 2} = \dot{v} \sin \beta + v \dot{\beta} \cos \beta = a_{c 2} - v \dot{\psi} \cos \beta = \frac{1}{m} ( F_{f} \cos \delta + F_{r} - m v_{c 1} \dot{\psi} ) $$          |        |
|   | $$ J \  \ddot{\psi} = F_{f} l_{f} \cos \delta - F_{r} l_{r} $$                                                                                                                      |        |

**Side slip angles of rear and front wheels**

For the computation of the wheel forces $$F_{r}$$ and $$F_{f}$$ the slip angles of the wheels are needed. In the case of forward drive $$v_{c 1} > 0$$, the slip angles are computed as:

|   | $$ \alpha_{f} = - {a t a n} \frac{v_{c 2} + l_{f} \dot{\psi}}{v_{c 1}} + \delta $$ |   |
|---|------------------------------------------------------------------------------------|---|
|   | $$ \alpha_{r} = - {a t a n} \frac{v_{c 2} - l_{r} \dot{\psi}}{v_{c 1}} $$          |   |

and in the case of reverse drive $$v_{c 1} < 0$$:

|   | $$ \alpha_{f} = {a t a n} \frac{v_{c 2} + l_{f} \dot{\psi}}{v_{c 1}} - \delta $$ |   |
|---|----------------------------------------------------------------------------------|---|
|   | $$ \alpha_{r} = {a t a n} \frac{v_{c 2} - l_{r} \dot{\psi}}{v_{c 1}} $$          |   |

In the case of stillstand $$\mathbf{v}_{\mathbf{c} \mathbf{1}} \mathbf{= 0} \frac{\mathbf{m}}{\mathbf{s}}$$, the slip angles are undefined, since the wheels have no moving direction.

Pacejka's Magic Formula Model for the wheel forces

For the computation of the wheel forces $$F_{r}$$ and $$F_{f} \$$in relation to the slip angles $$\alpha_{r}$$ and $$\alpha_{f}$$, the Magic Formula tire model of Pacejka (Pacejka & Bakker, 1992) is applied:

|   | $$ F_{r , f} = D_{r , f} \sin \left( C_{r , f} {a t a n} \left( B_{r , f} \left\lbrack 1 - E_{r , f} \right\rbrack \alpha_{r , f} - E_{r , f} {a t a n} \left( B_{r , f} \alpha_{r , f} \right) \right) \right) $$ |   |
|---|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

Parameters $$E_{r , f}$$ of the original tire model are assumed to be zero.

Combination of the Vehicle Dynamics Model and the Longitudinal Dynamics Model

As a result of Exercise 5.1 the longitudinal dynamics of the non-turning vehicle is given by the following differential equation:

|   | $$ \dot{v}_{r} ( t ) = - \frac{1}{T} v_{r} ( t ) + \frac{k_{u}}{T} u_{n} \left( t - T_{u t} \right) \  \  \  \  ; \  \  \  \  t > 0 $$ | (‎5.17) |
|---|----------------------------------------------------------------------------------------------------------------------------------------|--------|

This equation defines the dynamics of the real wheel speed $$v_{r} .$$

In case of the turning vehicle, the longitudinal real wheel force $$F_{r x}$$ is related to differential equation (‎5.17):

|   | $$ F_{r x} = m \left\lbrack - \frac{1}{T} v_{c 1} ( t ) + \frac{k_{u}}{T} u_{n} \left( t - T_{u t} \right) \right\rbrack $$ | (‎5.18) |
|---|-----------------------------------------------------------------------------------------------------------------------------|--------|

where the rear wheel speed $$v_{r}$$ equals to the longitudinal speed $$v_{c 1}$$ of COG.

Inserting (‎5.18) in (‎5.16) and combining the vehicle dynamics model with Pacejka's Magic Formula model yields the overall equation system of the longitudinal and lateral vehicle dynamics:

|   | $$ \delta ( t ) = \delta_{\max} \cdot \delta_{n} ( t - T_{u t} ) $$                                                                                                                                                                               | (‎5.19) |
|---|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
|   | $$ \alpha_{f} = \begin{cases}- {a t a n} \frac{v_{c 2} + l_{f} \dot{\psi}}{v_{c 1}} + \delta \  \  ; \  \  \  v_{c 1} > 0 \\ {a t a n} \frac{v_{c 2} + l_{f} \dot{\psi}}{v_{c 1}} - \delta \  \  \  \  ; \  \  \  \  v_{c 1} < 0\end{cases} $$    | (‎5.20) |
|   | $$ \alpha_{r} = \begin{cases}- {a t a n} \frac{v_{c 2} - l_{r} \dot{\psi}}{v_{c 1}} \  \  \  \  \  ; \  \  \  v_{c 1} > 0 \\ {a t a n} \frac{v_{c 2} - l_{r} \dot{\psi}}{v_{c 1}} \  \  \  \  \  \  \  \  \  ; \  \  \  v_{c 1} < 0\end{cases} $$ | (‎5.21) |
|   | $$ F_{f} = D_{f} \sin \left( C_{f} {a t a n} \left( B_{f} \left\lbrack 1 - E_{f} \right\rbrack \alpha_{f} - E_{f} {a t a n} \left( B_{f} \alpha_{f} \right) \right) \right) $$                                                                    | (‎5.22) |
|   | $$ F_{r} = D_{r} \sin \left( C_{r} {a t a n} \left( B_{r} \left\lbrack 1 - E_{r} \right\rbrack \alpha_{r} - E_{r} {a t a n} \left( B_{r} \alpha_{r} \right) \right) \right) $$                                                                    | (‎5.23) |
|   | $$ \beta = {a t a n 2} {( v_{c 2} , v_{c 1} )} $$                                                                                                                                                                                                 | (‎5.24) |
|   | $$ \dot{s}_{1} = v_{c 1} \cos \psi - v_{c 2} \sin \psi $$                                                                                                                                                                                         | (‎5.25) |
|   | $$ \dot{s}_{2} = v_{c 1} \sin \psi + v_{c 2} \cos \psi $$                                                                                                                                                                                         | (‎5.26) |
|   | $$ \dot{v}_{c 1} = \frac{1}{m} ( - F_{f} \sin \delta + m v_{c 2} \dot{\psi} ) - \frac{1}{T} v_{c 1} ( t ) + \frac{k_{u}}{T} u_{n} \left( t - T_{u t} \right) $$                                                                                   | (‎5.27) |
|   | $$ \dot{v}_{c 2} = \frac{1}{m} ( F_{f} \cos \delta + F_{r} - m v_{c 1} \dot{\psi} ) $$                                                                                                                                                            | (‎5.28) |
|   | $$ \dot{\psi} = \omega $$                                                                                                                                                                                                                         | (‎5.29) |
|   | $$ \  \dot{\omega} = \frac{1}{J} \left( F_{f} l_{f} \cos \delta - F_{r} l_{r} \right) $$                                                                                                                                                          | (‎5.30) |
|   | $$ \dot{x} = v_{c 1} \ $$                                                                                                                                                                                                                         | (‎5.31) |

The last equation (‎5.31) defines the driven path length $$x$$ by integrating the speed $$v_{c 1} \$$ of the rear axle center.

Equations (‎5.19) to (‎5.24) are algebraic that need to be evaluated before evaluating the right-hand sides of the differential equations (‎5.25) to (‎5.31).

The parameters of the mini cars in the Mini-Auto-Drive (MAD) lab experiment are:

| **Parameters**                                                            | **Symbol**              | **C++ Symbol** | **Simulink Symbol** | **Unit**              | **Value**                                             |
|---------------------------------------------------------------------------|-------------------------|----------------|---------------------|-----------------------|-------------------------------------------------------|
| maximum steering angle                                                    | $$ \delta_{\max} $$     | deltaMax       | P_p_delta_max       | $$ r a d $$           | $$ \frac{2 1 . 5 8 {^\circ}}{1 8 0 {^\circ}} \pi \ $$ |
| wheel base                                                                | $$ l = l_{r} + l_{f} $$ | l              | P_p_l               | $$ m $$               | $$ 0 . 0 9 9 $$                                       |
| distance of COG to rear axle                                              | $$ l_{r} $$             | lr             | P_p_lr              | $$ m $$               | $$ 0 . 0 5 0 $$                                       |
| distance of COG to front axle                                             | $$ l_{f} $$             | lf             | P_p_lf              | $$ m $$               | $$ 0 . 0 4 9 $$                                       |
| Magic Formula coefficient                                                 | $$ B_{r} $$             | Br             | P_p_Br              |                       | $$ 0 . 7 $$                                           |
| Magic Formula coefficient                                                 | $$ C_{r} $$             | Cr             | P_p_Cr              |                       | $$ 2 $$                                               |
| Magic Formula coefficient                                                 | $$ D_{r} $$             | Dr             | P_p_Dr              |                       | $$ 2 . 5 $$                                           |
| Magic Formula coefficient                                                 | $$ E_{r} $$             | Er             | P_p_Er              |                       | $$ - 0 . 0 5 $$                                       |
| Magic Formula coefficient                                                 | $$ B_{f} $$             | Bf             | P_p_Bf              |                       | $$ 0 . 7 $$                                           |
| Magic Formula coefficient                                                 | $$ C_{f} $$             | Cf             | P_p_Cf              |                       | $$ 2 $$                                               |
| Magic Formula coefficient                                                 | $$ D_{f} $$             | Df             | P_p_Df              |                       | $$ 2 $$                                               |
| Magic Formula coefficient                                                 | $$ E_{f} $$             | Ef             | P_p_Ef              |                       | $$ - 0 . 1 $$                                         |
| vehicle mass                                                              | $$ m $$                 | m              | P_p_m               | $$ k g $$             | $$ 0 . 1 3 2 $$                                       |
| moment of inertia w.r.t. COG                                              | $$ J $$                 | J              | P_p_J               | $$ k g \cdot m^{2} $$ | $$ 1 9 2 \cdot 10^{- 6} $$                            |
| gain of longitudinal dynamics                                             | $$ k_{u} $$             | k              | P_p_k               | $$ \frac{m}{s} $$     | $$ 2 . 5 1 $$                                         |
| time constant of longitudinal dynamics                                    | $$ T $$                 | T              | P_p_T               | $$ m s $$             | $$ 316 $$                                             |
| dead time of inputs (mainly, Bluetooth communication delay)               | $$ T_{u t} $$           | uTt            | P_p_uTt             | $$ m s $$             | $$ 44 $$                                              |
| dead time of outputs (mainly, camera acquisition and communication delay) | $$ T_{y t} $$           | outputTt       | P_p_output_Tt       | $$ m s $$             | $$ 66 $$                                              |

The input signals are:

| **Input Signal**                                                                                                               | **Symbol**       | **Element** | **Unit**                                                                                                                                  |
|--------------------------------------------------------------------------------------------------------------------------------|------------------|-------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| command for driving mode: halt, forward drive, reverse drive for speed control or slow drive for longitudinal position control | $$ u_{c m d} $$  | cmd         | $$ u_{c m d} \in \{ \  C M D \_ H A L T , \ $$ $$ C M D \_ F O R W A R D , \  \  C M D \_ R E V E R S E , \  \  C M D \_ S L O W \  \} $$ |
| normalized motor signal                                                                                                        | $$ u_{n} $$      | pedals      | $$ u_{n} \in \lbrack - 1 ; 1 \rbrack $$                                                                                                   |
| normalized steering angle                                                                                                      | $$ \delta_{n} $$ | steering    | $$ \delta_{n} \in \lbrack - 1 ; 1 \rbrack $$                                                                                              |

In MAD the input signals are transmitted as elements of the ROS message mbmadmsgs::CarInputs on ROS ropic /mad/car0/carinputs (see Figure ‎3.5) with a sample time of $$2 2 m s$$. In Simulink, a signal bus of type CARINPUTS is modeled that contains the same elements (see Figure ‎3.7).

The seven state variables of the dynamic state-space model of 7th order are:

| **State Variable**          | **Symbol**            | **C++ Symbol** | **Unit**              |
|-----------------------------|-----------------------|----------------|-----------------------|
| longitudinal speed          | $$ v_{c 1} = x_{1} $$ | x.at(0)        | $$ \frac{m}{s} $$     |
| first Cartesian coordinate  | $$ s_{1} = x_{2} $$   | x.at(1)        | $$ m $$               |
| second Cartesian coordinate | $$ s_{2} = x_{3} $$   | x.at(2)        | $$ m $$               |
| yaw angle                   | $$ \psi = x_{4} $$    | x.at(3)        | $$ r a d $$           |
| angular yaw rate            | $$ \omega = x_{5} $$  | x.at(4)        | $$ \frac{r a d}{s} $$ |
| lateral speed               | $$ v_{c 2} = x_{6} $$ | x.at(5)        | $$ \frac{m}{s} $$     |
| driven path length of COG   | $$ x = x_{7} $$       | x.at(6)        | $$ m $$               |

The output signals are transmitted in the ROS message mbmadmsgs::CarOutputs on topic /mad/car0/caroutputs (or in the corresponding Simulink signal bus CAROUTPUTS) with a sample time of $$2 2 m s$$:

| **Output Signal**                               | **Symbol / Equation**                                                      | **ROS Message Element / Simulink Bus Element** | **Unit**    |
|-------------------------------------------------|----------------------------------------------------------------------------|------------------------------------------------|-------------|
| first Cartesian coordinate of rear axle center  | $$ {y_{1} ( t ) = s}_{1} ( t - T_{y t} ) - a \cos \psi ( t - T_{y t} ) $$  | s.at(0)                                        | $$ m $$     |
| second Cartesian coordinate of rear axle center | $$ y_{2} ( t ) = \  s_{2} ( t - T_{y t} ) - a \sin \psi ( t - T_{y t} ) $$ | s.at(1)                                        | $$ m $$     |
| yaw angle                                       | $$ y_{3} ( t ) = \  \psi ( t - T_{y t} ) $$                                | psi                                            | $$ r a d $$ |

## Exercises

Exercise ‎5.1 Derivation of the Longitudinal Dynamics Model [C++/Simulink]

1.  Derive the mathematical model of the longitudinal plant dynamics described in Section ‎5.1. Represent the model in state space with the input signal $$u_{n}$$ and the output signal $$y = v_{r}$$.
2.  Simplify the model by reducing all model parameters to the following three parameters:
    -   time constant $$T = 3 1 6 m s$$
    -   combined dead time $$T_{t} = T_{u t} + T_{y t} = 1 1 0 m s$$ of the motor electronics and RC transmission plus computer vision
    -   static plant dynamics gain $$k_{u} = 2 . 5 1 \frac{m}{s}$$

        The parameter values have been identified on the real system by step responses.

3.  Derive the transfer function of the plant dynamics $$G_{S} ( s ) = \frac{Y ( s )}{U ( s )} .$$
4.  Display the Bode diagram and the step response of the above transfer function in c. by using MATLAB Control System Toolbox.

Required lab results:

-   Dynamic state-space model of a.
-   Mathematical expressions for parameters $$T$$ and $$k$$ in terms of given parameters in Section ‎5.1
-   Transfer function $$G_{S} ( s )$$
-   Bode diagram and step response of $$G_{S}$$
-   MATLAB script of d. named ex5_1.m

Exercise ‎5.2 Vehicle Dynamics Simulation [Simulink]

In this exercise, the Simulink subsystem Vehicle Dynamics of Figure ‎3.7 will be modeled and tested in simulation runs. This subsystem

-   represents the vehicle dynamics model including lateral and longitudinal dynamics of Section ‎5.5
-   applies Runge-Kutta of 4th order (Simulink solver ode4) to solve the dynamic state-space model
    -   the sample time of the Runge-Kutta solver needs to be set to $$2 m s$$ to achieve numerically stable and accurate simulation results
-   receives the bus signal CARINPUTS to manipulate the car
-   generates the bus signal CAROUTPUTS for the control functions
-   generates the bus signal CAROUTPUTSEXT for debugging purposes

The following steps are recommended for this exercise:

1.  Download and unpack mad2.zip from your e-learning platform.

    This repository contains among others:

-   MATLAB files matlab/vertical2/mbc/mbc\*.m of the MODBAS CAR library with routines required for constructing maps and modeling controllers later in Chapters ‎7 and ‎9
-   MATLAB script file d01_data.m defining the model parameters of Section ‎5.5
-   MATLAB script file s06_data.m as the data file of the Simulink model s06_sig_template.slx. Running s06_data.m executes d01_data.m and t02_race.m and creating and displayin map in a MATLAB figure. The Simulink model s06_sig_template.slx will display the moving car in this figure.
-   Simulink model s06_sig_template.slx is the template of the model to be developed in this exercise
1.  Model the state-space model of the vehicle dynamics of Section ‎5.5 as part of the subsystem Vehicle Dynamics in s06_sig_template.slx

    For modeling the vehicle dynamics of Section ‎5.5, follow the lines of simulating state-space models in Simulink of Section ‎4.

    Limit the input signals $$u_{n}$$ and $$\delta_{n}$$ to $$u_{n} \in [ - 1 ; 1 ]$$ or $$\delta_{n} \in [ - 1 ; 1 ]$$, respectively, by using Simulink block Saturation in category Simulink/Discontinuities.

    Motor signal $$u_{n}$$ shall be further limited in dependence of driving mode command $$u_{c m d}$$ according to Section ‎5.1. Valid values of $$u_{c m d}$$ are the following constants in i01_data.m: CarInputsCmdHalt, CarInputsCmdForward, CarInputsCmdReverse, CarInputsCmdSlow.

    You may use the Simulink block Transport Delay in category Simulink/Continuous to model the dead times $$T_{u T}$$ and $$T_{y T}$$.

    Use the parameters of d01_data.m. Do not enter further parameters.

    For low speeds ($$\left| v_{c 1} \right| < 0 . 2 m / s )$$ the vehicle dynamics model fo Section ‎5.5 leads to simulation errors, since the mathematical formulas of the wheels’ side slip angles have singularities at speed $$v = 0 \frac{m}{s}$$. For these reasons, the model must be extended by if/then/else conditions, that distinguish between the two cases: high speed and low speed.

    In case of low speed, the kinematics model of Section ‎5.4 including the longitudinal dynamics model for $$v_{c 1}$$ of Exercise 5.1 shall be applied. This model is sufficiently accurate at low speed and has no singularities.

    In case of high speeds, the dynamics model of Section ‎5.5 shall be applied as in c.

    In order to implement the if/then/else conditions, modify the right-hand sides of the differential equations only.

2.  Test s06_sig_template.slx by
    -   manipulating the car by setting different constant values for driving mode $$u_{c m d}$$, for motor signal ($$u_{n} \in [ - 1 ; 1 ]$$) and steering ($$\delta_{n} \in [ - 1 ; 1 ]$$)
    -   measuring the vehicle dynamics responses by recording and analyzing the elements of bus signals CAROUTPUTS and CAROUTPUTSEXT in the Simulink Data Inspector
    -   analyzing the vehicle movement in the MATLAB figure of the road track
    -   after running the simulation and recording a signal, e.g., speed $$v$$, a signal-time diagram can be created in a MATLAB figure by running

        figure(2) , clf;

        plot(logsout.get('v').Values.Time, logsout.get('v').Values.Data);

        grid on;

        xlabel('t / s');

        ylabel('v / m/s');

        title('Exercise 5.2');

Required lab results:

-   s06_sig_template.slx
-   signal-time diagrams of step responses of vehicle speed $$v$$
-   signal-time diagrams of step responses of yaw angle $$\psi$$

# Speed Control

*Speed control* is required for *path following control* which will be designed in subsequent chapters.

Synonyms of *speed control* are *cruise control* or *Tempomat*.

-   The *controlled process variable* is the longitudinal vehicle speed.
-   The *reference variable* *(set point)* is the target speed.
-   The *manipulated control variable* is the normalized DC motor voltage in case of the MAD lab experiment.

For the design of the speed controller

-   the longitudinal dynamics and no lateral dynamics of the vehicle are considered
-   the vehicle is driving on a planar surface

Figure ‎6.1: Control loop of speed control for longitudinal dynamics

The signal-flow chart of the resulting control loop is depicted in Figure ‎6.1:

-   *plant* is the longitudinal dynamics of the vehicle
-   *controller* is the speed controller that will later be implemented in software on the Linux PC of MAD
-   *process variable* $$y ( t )$$ is the actual vehicle speed $$v_{r} ( t )$$, i.e., the element v of ROS topic /mad/caroutputsext or of bus signal caroutputsext in Simulink (see Figure ‎3.4 and Figure ‎3.7)
-   *control (manipulated) signal* $$u ( t )$$ is the input signal of the motor, i.e., the element pedals of ROS topic /mad/carinputs or of bus signal carinputs in Simulink
-   *disturbance signal* $$d ( t )$$ denotes any disturbance acting on the vehicle, e.g., friction neglected in the plant model or counteracting wheel forces in curvatures. Disturbance signals cannot be manipulated by controllers or software, in general.
-   *reference signal* $$w ( t )$$ is the target speed of the vehicle, i.e., the element vmax of ROS topic /mad/ctrlinputs or the constant scalar signal vref in Simulink
-   *control deviation* $$e ( t )$$ is the error of the actual speed $$y ( t )$$ w.r.t. the target speed $$w ( t )$$:

|   | $$ e ( t ) = w ( t ) - y ( t ) $$ |   |
|---|-----------------------------------|---|

The main task of the controller is to reduce the control deviation $$e ( t )$$ by generating $$u ( t )$$. Next to *asymptotic stability, steady-state accuracy* (no *permanent control deviation*) is the main requirement of any control loop:

|   | $$ \lim_{t \rightarrow \infty} {e ( t )} = \lim_{t \rightarrow \infty} \left\lbrack w ( t ) - y ( t ) \right\rbrack = 0 $$ |   |
|---|----------------------------------------------------------------------------------------------------------------------------|---|

The actual design of the speed controller is part of Exercise 6.1. The speed controller will be implemented as part of the new ROS node carctrl_node or alternatively as part of the Simulink subsystem Motion Control in Exercise 6.2. Please refer once again to Figure ‎3.4 and Figure ‎3.5 or Figure ‎3.7.

In order to repeat basic knowledge in designing linear PI/PID controllers, Section ‎6.1 discusses the design of a speed controller under the assumption of simplified longitudinal dynamics. Section ‎6.1 then specifies the requirements to the control loop of the actual longitudinal dynamics which forms the basis for Exercises 6.1 and 6.2.

## PI-Controller Design for PT1-Plant

In this section, basic knowledge of Bachelor-level control engineering is repeated. First, the plant model is defined and analyzed. Second, the PI controller is designed, i.e., its parameters are computed. Third, the closed-loop dynamics are analyzed in Bode diagrams.

Plant Model

In order to simplify the controller design, the longitudinal vehicle dynamics of Section ‎5.1 and Exercise 5.1 is simplified to:

| ODE | $$ T \  \dot{v}_{r} ( t ) + v_{r} ( t ) = k_{u} \  u_{n} ( t ) \  \  \  \  \  \  ; \  \  \  \  t > 0 $$ | (‎6.1) |
|-----|---------------------------------------------------------------------------------------------------------|-------|
| IC  | $$ v_{r} ( 0 ) = v_{0} $$                                                                               |       |

with gain $$k_{u} = 2 . 5 1 \frac{m}{s}$$ and time constant $$T = 3 1 6 m s$$, i.e., the dead time $$T_{t} = 1 1 0 m s \$$of motor electronics, RC transmission and computer vision is neglected, which has a significant impact on the longitudinal dynamics.

In control engineering, common notation is to introduce the following signals:

-   process variable $$y = v$$
-   control (manipulated) signal $$u = u_{n}$$

Using these common signals, (‎6.1) becomes:

| ODE | $$ T \  \dot{y} ( t ) + y ( t ) = k_{u} \  u ( t ) \  \  \  \  \  \  ; \  \  \  \  t > 0 $$ | (‎6.2) |
|-----|---------------------------------------------------------------------------------------------|-------|
| IC  | $$ y ( 0 ) = v_{0} $$                                                                       |       |

This model is linear and time-invariant and, thus, can be transformed by Laplace transformation:

|   | $$ T \  \left\lbrack s Y ( s ) - v_{0} \right\rbrack + Y ( s ) = k_{u} \  U ( s ) $$ | (‎6.3) |
|---|--------------------------------------------------------------------------------------|-------|

Please note that

-   (‎6.3) represents both the ordinary differential equation and the initial condition in Laplace space
-   the signal functions $$Y ( s )$$ and $$U ( s )$$ are very different to $$y ( t )$$ and $$u ( t )$$
-   in order to distinguish signal functions in Laplace space from signal functions in time space, signal functions in Laplace space are denoted in upper-case letters, whereas signal functions in time space are denoted in lower-case letters

One of the advantages of Laplace space is that the system model (‎6.3) can be algebraically solved:

|   | $$ Y ( s ) = \underset{G_{S} ( s )}{\overset{\frac{k_{u}}{T s + 1}}{︷}} U ( s ) + \underset{G_{S 0} ( s )}{\overset{\frac{T}{T s + 1}}{︷}} v_{0} $$ | (‎6.4) |
|---|-------------------------------------------------------------------------------------------------------------------------------------------------------|-------|

The factors $$G_{S} ( s )$$ and $$G_{S 0} ( s )$$ denote the two *transfer functions* of the plant, where only $$G_{S} ( s )$$ is relevant to the subsequent controller design because this transfer function represents the I/O dynamics of the plant.

The simplified longitudinal dynamics is of type PT1: $$G_{S} ( s ) = \frac{k_{u}}{T s + 1}$$ with gain $$k_{u} = 2 . 5 1 \frac{m}{s} \triangleq 8 d B$$ and time constant $$T = 3 1 6 m s$$.

PI Controller Design

The first step in controller design is to choose an appropriate controller type. In case of a PT1 plant, a PI controller is optimal whose transfer function is given by

|   | $$ G_{R} ( s ) = k_{p} + \frac{k_{i}}{s} $$ | (‎6.5) |
|---|---------------------------------------------|-------|

This is the so-called *sum notation* of the PI controller with

-   *proportional gain* $$k_{p}$$
-   *integral gain* $$k_{i}$$

By applying the distributive law, the PI controller can be formulated in *polynomial notation*:

|   | $$ G_{R} ( s ) = k_{p} \left( 1 + \frac{1}{T_{i} s} \right) = k_{p} \  \frac{T_{i} s + 1}{T_{i} s} $$ | (‎6.6) |
|---|-------------------------------------------------------------------------------------------------------|-------|

with

-   *controller gain* $$k_{p}$$
-   *integral time constant* $$T_{i}$$

The advantages of the polynomial notation are the direct availability of the *zeros* and *poles* of the transfer function:

-   zero $$s_{r 0 1} = - \frac{1}{T_{i}}$$
-   pole $$s_{r 1} = 0$$

and the direct availability of time constant $$T_{i}$$ which is the major design parameter in the subsequent controller design.

When applying a PI controller to control a PT1 plant, the application of the design method *dynamic compensation* is most appropriate. Dynamic compensation leads to PT1 control-loop dynamics whose time constant can be adapted by changing controller gain $$k_{p} .$$

The great advantage of PT1 control loop dynamics is that it shows no oscillations or overshoots in step responses. I.e., when modifying the target speed $$w ( t )$$ in steps, no oscillations occur which is an obligatory requirement for speed control.

Without dynamic compensation, the *open-loop transfer function* is given as:

|   | $$ G_{0} ( s ) = \frac{Y ( s )}{E ( s )} = G_{R} ( s ) G_{S} ( s ) = k_{p} \  k_{u} \frac{T_{i} s + 1}{T_{i} s ( T s + 1 )} $$ |   |
|---|--------------------------------------------------------------------------------------------------------------------------------|---|

Dynamic compensation compensates pole $$s_{s 1} = - \frac{1}{T}$$ of $$G_{S}$$ by zero $$s_{r 0 1} = - \frac{1}{T_{i}}$$ of $$G_{R}$$ . Zero $$s_{r 0 1}$$ is set equal to pole $$s_{s 1}$$:

|   | $$ s_{r 01} = s_{s 1} $$ |   |
|---|--------------------------|---|

The compensation thus defines the integral time constant $$T_{i}$$ of $$G_{R}$$ in terms of the given time constant $$T$$ of $$G_{S}$$:

|   | $$ T_{i} = T = 3 1 6 m s $$ |   |
|---|-----------------------------|---|

By dynamic compensation, the linear factors in the numerator and denominator of $$G_{0} ( s )$$ are reduced:

|   | $$ G_{0} ( s ) = k_{p} \  k_{u} \frac{T_{i} s + 1}{T_{i} s ( T s + 1 )} = \frac{k_{p} \  k_{u}}{T_{i} s} $$ |   |
|---|-------------------------------------------------------------------------------------------------------------|---|

The remaining controller parameter $$k_{p}$$ can be computed by specifying the time constant $$T_{w}$$ of the closed-loop dynamics.

In general, by negative feedback the resulting transfer function of the closed-loop dynamics is obtained from the open-loop dynamics by the following relation:

|   | $$ G_{w} ( s ) = \frac{Y ( s )}{W ( s )} = \frac{G_{0} ( s )}{1 + G_{0} ( s )} = \frac{1}{\frac{T_{i}}{k_{p} k_{u}} s + 1} $$ |   |
|---|-------------------------------------------------------------------------------------------------------------------------------|---|

This is a PT1 dynamics with time constant

|   | $$ T_{w} = \frac{T_{i}}{k_{p} k_{u}} = \frac{T}{k_{p} k_{u}} $$ | (‎6.7) |
|---|-----------------------------------------------------------------|-------|

and a static gain of $$1$$. A static gain of $$1$$ is required for the steady-state accuracy of the closed control loop.

By specifying the closed-loop time constant $$T_{w}$$ as an requirement to the control system, e.g., $$T_{w} = 1 0 0 0 m s$$, the remaining controller gain $$k_{p}$$ can be computed from (‎6.7):

|   | $$ k_{p} = \frac{T}{T_{w} k_{u}} = \frac{316 m s}{1000 m s \cdot 2 . 5 1 \frac{m}{s}} = 0 . 1 2 6 \frac{s}{m} \widehat{=} - 18 d B $$ |   |
|---|---------------------------------------------------------------------------------------------------------------------------------------|---|

Analysis

An important analysis is the frequency response analysis of the closed-loop dynamics. The following MATLAB code displays the Bode diagram of the frequency responses:

|   | $$ G_{S} ( j \omega ) = \frac{k_{u}}{1 + j \omega T} $$                       |   |
|---|-------------------------------------------------------------------------------|---|
|   | $$ G_{R} ( j \omega ) = k_{p} \  \frac{1 + j \omega T_{i}}{j \omega T_{i}} $$ |   |
|   | $$ G_{0} ( j \omega ) = \frac{k_{p} k_{u}}{j \omega T_{i}} $$                 |   |
|   | $$ G_{w} ( j \omega ) = \frac{1}{1 + j \omega T_{w}} $$                       |   |

for the case above:

%% plant

k = 2.51; % plant gain [ m/s ]

T = 316e-3; % plant time constant [ s ]

Gs = tf(k, [ T , 1 ]);

%% controller design

Tw = 1000e-3; % close-loop time constant [ s ]

Ti = T; % integral time constant by dynamic compensation [ s ]

kp = T / (Tw\*k); % controller gain [ s/m ]

Gr = tf(kp \* [ Ti , 1 ], [ Ti , 0 ]);

%% open-loop dynamics

G0 = minreal(Gr \* Gs);

%% closed-loop dynamics

Gw = G0 / (1 + G0);

%% Bode diagram

figure(1) , clf;

bode(Gs, Gr, G0, Gw);

grid on;

legend('Gs', 'Gr', 'G0', 'Gw');

This MATLAB code uses the MATLAB Control System Toolbox (MATLAB CST) and creates the Bode diagram in Figure ‎6.2.

The mathematical expressions gain and phase responses that are depicted in the Body diagram are:

|   | $$ A_{S} ( \omega ) = \left| G_{S} ( j \omega ) \right| = \frac{k_{u}}{\sqrt{1 + ( \omega T )^{2}}} $$                             |   |
|---|------------------------------------------------------------------------------------------------------------------------------------|---|
|   | $$ A_{R} ( \omega ) = \left| G_{R} ( j \omega ) \right| = \frac{k_{p} \sqrt{1 + \left( \omega T_{i} \right)^{2}}}{\omega T_{i}} $$ |   |
|   | $$ A_{0} ( \omega ) = \left| G_{0} ( j \omega ) \right| = A_{R} ( \omega ) A_{S} ( \omega ) = \frac{k_{p} k_{u}}{\omega T_{i}} $$  |   |
|   | $$ A_{0 d B} ( \omega ) = 2 0 \lg {A_{0} ( \omega )} = A_{R d B} ( \omega ) + A_{S d B} ( \omega ) $$                              |   |
|   | $$ A_{w} ( \omega ) = \left| G_{w} ( j \omega ) \right| = \frac{1}{\sqrt{1 + \left( \omega T_{w} \right)^{2}}} $$                  |   |
|   | $$ \varphi_{S} ( \omega ) = \arg {G_{S} ( j \omega )} = - \arctan ( \omega T ) $$                                                  |   |
|   | $$ \varphi_{R} ( \omega ) = \arg {G_{R} ( j \omega )} = - \frac{\pi}{2} + \arctan \left( \omega T_{i} \right) $$                   |   |
|   | $$ \varphi_{0} ( \omega ) = \arg {G_{0} ( j \omega )} = \varphi_{R} ( \omega ) + \varphi_{R} ( \omega ) = - \frac{\pi}{2} $$       |   |
|   | $$ \varphi_{w} ( \omega ) = \arg {G_{w} ( j \omega )} = - \arctan \left( \omega T_{w} \right) $$                                   |   |

The gain diagrams of the controller and the plant show identical cut-off frequencies due to dynamic compensation:

|   | $$ \omega_{1} = \frac{1}{T_{i}} = \frac{1}{T} = 3 . 1 6 \frac{r a d}{s} \  \  ; \  \  \  \  \lg \omega_{1} = 0 , 5 $$ |   |
|---|-----------------------------------------------------------------------------------------------------------------------|---|

The gains and phases at this cut-off frequency are computed as:

|   | $$ A_{S} \left( \omega_{1} \right) = \frac{k_{u}}{\sqrt{2}} $$          |   |
|---|-------------------------------------------------------------------------|---|
|   | $$ A_{S d B} \left( \omega_{1} \right) = 2 0 \log_{10} k_{u} - 3 d B $$ |   |
|   | $$ A_{R} \left( \omega_{1} \right) = \sqrt{2} {\cdot k}_{p} $$          |   |
|   | $$ A_{R d B} \left( \omega_{1} \right) = 2 0 \log_{10} k_{p} + 3 d B $$ |   |
|   | $$ \varphi_{S} \left( \omega_{1} \right) = - \frac{\pi}{4} $$           |   |
|   | $$ \varphi_{R} \left( \omega_{1} \right) = - \frac{\pi}{4} $$           |   |

![](media/621226ca270d3b3a482d8931e6b4c6b5.emf)

Figure ‎6.2: Bode diagram of the relevant frequency responses of the speed control loop

## Requirements of the Control Loop Dynamics

Now, the original longitudinal dynamics with dead time $$T_{t} = 1 1 0 m s$$ of Section ‎5.1 and Exercise 5.1 is considered. In this case, PI controller design with dynamic compensation does not lead to satisfactory results due to the large dead time $$T_{t}$$.

In Exercise 6.1 an alternative approach is taken in the design of the PI controller. In the following paragraphs, the system requirements of the closed loop for the speed control system are specified as a basis of Exercise 6.1.

[SYS-Reference-Signal]

The input signal (reference signal) of the speed controller is the reference speed $$w$$ of the car in longitudinal direction. This reference speed may be discontinuous.

The reference speed is transmitted as the element vmax of the ROS message mbmadmsgs::DriveManeuver on ROS topic /mad/car0/maneuver which may be generated by Python script send_maneuver.py.

In Simulink, subsystem Motion Control reads in bus signal DRIVEMANEUVER, which contains the scalar element vmax for the reference speed.

[SYS-Process-Variable]

The process variable is the rear-axle speed $$y = v_{r}$$ which is approximately equal to the longitudinal speed $$v_{c 1}$$ of COG.

The existing ROS node locatenode computes the actual speed $$v_{r}$$ and transmits it as the element v of the ROS message mbmadmsgs::CarOutputsExt on the ROS topic /mad/locate/caroutputsext.

In Simulink, subsystem Motion Control reads in the actual speed v from bus signal CAROUTPUTSEXT that is generated by subsystem Localization.

[SYS-Control-Variable]

The control variable is the normalized DC motor voltage $$u_{n} \in \lbrack - 1 ; 1 \rbrack$$. The control variable shall be limited to these minimum and maximum limits.

The speed controller as part of carctrlnode must transmit $$u_{n} \$$as the element pedals of the ROS message mbmadmsgs::CarInputs on the ROS topic /mad/car0/carinputs.

In Simulink, subsystem Motion Control will generate the bus signal CARINPUTS that has the same elements as mbmadmsgs::CarInputs and is read in by subsystem Vehicle Dynamics.

According to Section ‎5.1, the additional control signal $$u_{c m d}$$ (element cmd of CARINPUTS) selects the driving mode to protect the DC motor and the motor electronics against thermal runaways and to allow emergency stops in case of MAD system failures. The speed controller shall output this signal $$u_{c m d}$$ in dependence of the reference speed $$w$$ and the activation of the longitudinal position controller.

If the longitudinal position controller (see Chapter ‎7) is active then the driving mode CarInputsCmdSlow in Simulink or CMD_SLOW in C++ shall be set as the value of element cmd of CARINPUTS. In this driving mode, the vehicle can arbitrarily change its driving direction at any time. This mode is required for reaching the target longitudinal position at low speeds with high accuracy.

In contrast, if the longitudinal position controller is inactive and the reference speed $$w$$ is greater or equal to $$0 \frac{m}{s}$$ then the driving mode CarInputsCmdForward in Simulink or CMD_FORWARD in C++ shall be set. If the reference speed $$w$$ is less then $$0 \frac{m}{s}$$ then reverse drive shall be activated by setting CarInputsCmdReverse or CMD_REVERSE, respectively.

[SYS-Closed-Loop-Dynamics]

The controller design shall ensure that the process variable $$y = v_{r}$$ follows the reference signal $$w \$$ with no oscillations. For this, the control loop dynamics shall have the following characteristics:

-   The *overshoot time* of the *step response* shall be $$T_{m} = 1 0 0 0 m s$$ that corresponds to a *phase crossover frequency* of $$\omega_{D} \approx \frac{\pi}{T_{m}} = \pi \frac{r a d}{s}$$ of the open loop frequency response $$G_{0} ( j \omega )$$.
-   The *overshoot* of the *step response* shall be $$e_{m} = 4 \%$$ that corresponds to a *phase margin* of $$\varphi_{R e s} \approx 6 5 {^\circ}$$ of the open loop frequency response $$G_{0} ( j \omega )$$.

## Exercises

Exercise ‎6.1 Design of the Speed Controller [C++/Simulink]

For the controller design a continuous-time PI controller in polynomial form is applied:

|   | $$ G_{R} ( s ) = k_{r} \frac{1 + T_{i} s}{T_{i} s} $$ |   |
|---|-------------------------------------------------------|---|

The following design is based on the existing longitudinal plant dynamics $$G_{S} ( s ) = \frac{k_{u} e^{- s T_{t}}}{T s + 1}$$ (including dead time $$T_{t} = 1 1 0 m s$$) of Exercise 5.1 while neglecting the lateral dynamics. Please note, that the lateral dynamics is the main disturbance of the speed control.

1.  Derive the controller parameters $$k_{r}$$ and $$T_{i}$$ by considering the above requirements of the control loop and open loop dynamics.

    Note that the following equations hold for the open loop frequency response $$G_{0} ( j \omega )$$, the gain crossover frequency $$\omega_{D}$$ and the phase margin $$\varphi_{R}$$:

| open loop frequency response                             | $$ G_{0} ( j \omega ) = G_{R} ( j \omega ) \cdot G_{S} ( j \omega ) $$ |   |
|----------------------------------------------------------|------------------------------------------------------------------------|---|
| general open loop magnitude response                     | $$ A_{0} ( \omega ) = \left| G_{0} ( j \omega ) \right| $$             |   |
| general open loop phase response                         | $$ \varphi_{0} ( \omega ) = \arg {G_{0} ( j \omega )} $$               |   |
| phase margin                                             | $$ \varphi_{R e s} = \pi + \varphi_{0} \left( \omega_{D} \right) $$    |   |
| magnitude response at crossover frequency $$\omega_{D}$$ | $$A_{0} \left( \omega_{D} \right) = 1$$ (corresponds to $$0 d B )$$    |   |

1.  Use MATLAB command margin to determine the phase and amplitude margins of the open loop $$G_{0} ( j \omega )$$. Verify that the required phase margin $$\varphi_{R e s}$$ is achieved.
2.  Use the MATLAB command step to display the step response of control loop $$G_{w} ( s ) = G_{0} ( s ) / ( 1 + G_{0} ( s ) )$$. Verify that the required overshoot time $$T_{m}$$ and overshoot $$e_{m}$$ are achieved.
3.  The controller shall be realized as a discrete-time PI controller with parallel P and I components. The above time-continuous controller can be represented in the following parallel form:

|   | $$ G_{R} ( s ) = k_{r} \left( 1 + \frac{1}{T_{i} s} \right) $$ |   |
|---|----------------------------------------------------------------|---|

Discretize the integral part (I part) by applying the backward differences method

$$
s \approx ( 1 - z^{- 1} ) / T_{A}
$$

where $$T_{A}$$ is the sample time of the discrete controller. Setting $$T_{A} = 2 2 m s$$ is a good choice in this control loop.

Required lab results:

-   mathematical expressions and values of $$T_{i}$$ and $$k_{r}$$
-   Bode diagram of $$G_{0}$$ including margins
-   signal-time-diagram of step response of $$G_{w}$$
-   transfer function $$G_{R}^{*} ( z ) \$$of discrete-time PI controller
-   differences equations to compute I part $$u_{i k} = u_{i} ( k T_{A} )$$ and total control signal $$u_{k} = u \left( k T_{A} \right)$$ from control deviation $$e_{k} = w_{k} - y_{k}$$
-   MATLAB script of b. and c. named ex6_1.m

Exercise ‎6.2 Simulink Subsystem “Motion Control” for Speed Control [Simulink]

In this exercise, the Simulink subsystem Motion Control of Figure ‎3.7 will be modeled and put into operation. This subsystem

-   controls the vehicle speed by the discrete-time controller of Exercise 6.1
-   receives the bus signal CAROUTPUTSEXT to measure the speed v
-   receives the bus signal MANEUVER which contains the signal vmax for target / reference speed (setpoint)
-   generates the bus signal CARINPUTS with a sample time of $$2 2 m s$$ to manipulate the car
-   will be extended in subsequent exercises by path following control

The following steps are recommended for this exercise:

1.  Copy the Simulink model s06_sig_template.slx of Exercise 5.2 to a new Simulink model s07_sig_template.slx and the data file s06_data.m to s07_data.m. Only work on this new model.
2.  Model the discrete-time PI controller of Exercise 6.1 as part of the new subsystem Speed Control as part of subsystem Motion Control.
-   Limit the control signal $$u_{n}$$ to $$u_{n} \in \lbrack - 1 ; 1 \rbrack$$ (pedals in CARINPUTS).
-   Extend the model of the discrete-time PI controller by a clamping anti-windup filter which holds the numerical integration of the integral part when $$u_{n}$$ exceeds the limits: $$u_{n} \notin \lbrack - 1 ; 1 \rbrack$$. When $$u_{n}$$ is within the limits: $$u_{n} \in \lbrack - 1 ; 1 \rbrack$$, the numerical integration of the integral part is continued.
-   Set the control signal $$u_{c m d}$$ (cmd in CARINPUTS) to manipulate the driving mode according to the requirements in Section ‎6.1.
-   Set the control signal $$\delta_{n}$$ for steering (steering in CARINPUTS) to a constant value other than 0, so that the vehicle runs in circles. A good choice is $$\delta_{n} = 0 . 7$$.
1.  Extend the data file s07_data.m by statements to compute the controller parameters.
2.  Test s07_template.slx in MiL simulations in Simulink by
    -   manipulating the car by setting different constant values for vmax as the element of bus signal DRIVEMANEUVER
    -   measuring the speed responses on CAROUTPUTSEXT
3.  Test the subsystem Motion Control in SiL simulations on the simulated MAD system
    -   Login in on the MAD Linux PC
    -   Open a new terminal by hitting Ctrl+Alt+t
    -   Start MATLAB

        cd \~/mad2/matlab/vertical2/vertical

        matlab2023b

    -   Open the Simulink model c07_car0.slx that models the main driving functions (see Section ‎3.3)
    -   Download your Simulink model s07_sig_template.slx and s07_data.m
    -   Replace the subsystem Motion Control in c07_car0.slx by copying the subsystem Motion Control from your model s07_sig_template.slx to c07_car0.slx
    -   Copy the required parameter definitions from s07_data.m to c07_data.m
    -   Generate ROS node c07_car0 from c07_car0.slx by running the MATLAB script ros2build.m
    -   Display the build process by clicking View Diagnostics on the bottom window line in Simulink
    -   Build ROS including ROS node c07_car0

        cd \~/mad2/mad_ws

        colcon build –symlink-install –cmake-args -DCMAKE_BUILD_TYPE=Release

    -   Open a new terminal and run the MAD simulation

        ros2 launch mbmadcar simctrl.launch

    -   Open a further terminal and generate drive maneuvers by starting Python script send_maneuver.py

        ros2 run mbmadcar send_maneuver.py 0 0.5

    -   Manipulate the reference speed by modifying the second argument of send_maneuver.py
    -   Analyze the control loop behavior by measuring in rqt
4.  After successful SiL tests you are now ready to test speed controller Motion Control on the real MAD system
    -   Stop ROS by hitting Ctrl+c in the terminal running ros2 launch
    -   Place the car on the track at an initial position so that there is enough space to finish the circular drive
    -   Now start the real MAD system

        ros2 launch mbmad ctrl.launch

    -   All other steps are identical to e.

Required lab results:

-   s07_sig_template.slx
-   s07_data.m
-   signal-time diagrams of step responses of vehicle speed $$v_{r}$$ for different reference speeds $$w$$ in Simulink MiL simulations and on real MAD system

# Longitudinal Position Control

Only available in German book (Tränkle, Modellbasierte Entwicklung mechatronischer Systeme: mit Software- und Simulationsbeispielen für autonomes Fahren, 2021).

## Exercises

Exercise 7.1: Longitudinal Position Control Design [C++/Simulink]

For the driving maneuvers to park or stop a MAD vehicle, the longitudinal position control is designed as a cascade control, firstly without feed-forward control, and secondly with feed-forward control. The inner control loop is the speed control loop of Chapter ‎6 without any modifications. For the outer position feed-back control, a P-controller is applied.

The design of the feed-back control, i.e., the computation of controller parameter $$k_{p}$$, is based on the computation of the steady-state control deviation $$e_{y}$$:

$$
{e_{y} = \lim_{t \rightarrow \infty}} {e_{p} ( t )}
$$

in the case of a unlimited ramp signal for the reference position:

$$
w_{p} ( t ) = v^{*} \cdot t \cdot h ( t )
$$

with the assumption of an initial position $$x_{0} = 0 m$$ at $$t_{0} = 0 s$$*.*

a. Compute $$e_{y}$$ by applying the limit rules of Laplace transform for general ramp signal, plant dynamics and controller parameters. Use the approximated plant dynamics.

b. The steady-state control deviation shall be $$e_{y} = 1 0 c m$$ at a reference speed of $$v^{*} = 0 . 1 \frac{m}{s}$$ . Compute the required controller parameter $$k_{p}$$.

c. Validate your results, by simulating the ramp signal response $$y_{p} ( t )$$ of the cascaded control loop in case of an unlimited ramp signal $$w_{p} ( t ) = v^{*} t h ( t )$$ with the MATLAB-Control-System-Toolbox. Plot the signal-time-diagrams of $$w_{p} ( t )$$ and $$y_{p} ( t )$$.

d. What is the stability limit of $$k_{p}$$? There is no analytical solution. So apply MATLAB command rlocus to plot the root locus curves of the closed control loop in dependence of $$k_{p}$$.

e. Now extend the control loop by the feed-forward control. Program MATLAB function

[ c, te ] = cd_refpoly_vmax(vmax, x0, xs)

to compute the end time $$t_{E} \$$ and the polynomial coefficients $$c_{5} , c_{4} , \ldots , \  c_{0}$$ of $$w_{p} ( t )$$. This MATLAB-Funktion cd_refpoly_vmax has the following input arguments:

-   the maximum speed $$v^{*}$$ as argument vmax,
-   the initial position $$x_{0}$$ as argument x0,
-   the arc length to drive $$x^{*}$$ as argument xs.

The return arguments are

-   row vector c containing the polynomial coefficients $$c_{5} , c_{4} , \ldots , \  c_{0}$$,
-   end time te at which boundary condition $$w_{p} \left( t_{E} \right) = x_{0} + x^{*}$$ holds.

Program MATLAB function

cff = cd_refpoly_ff(c, k, T, Tt, kr, Ti)

to compute the polynomial coefficients of signal $$u_{V p 1} ( t )$$. Its input arguments are

-   row vector c of polynomial coefficients of $$w_{p} ( t )$$ that have been computed by cd_refpoly_vmax,
-   parameters $$k , T , T_{t} , k_{r} , \  T_{i}$$ of the plant dynamics and speed controller.

Its return argument is

-   row vector cff containing the polynomial coefficients of $$u_{V p 1} ( t )$$.

Extend MATLAB script ex6_1.m by tests for both new MATLAB functions cd_refpoly_vmax and cd_refpoly_ff considering the case:

-   $$v^{*} = 0 . 5 \  m / s$$ , $$x_{0} = 0 \  m$$, $$\  x^{*} = 1 \  m .$$

The tests shall compute $$w_{p} ( t ) , \  y_{p} ( t ) , \dot{w}_{p} ( t ) , \ddot{w}_{p} ( t ) \  \$$and $$u_{V p 1} ( t )$$ and plot these signals in signal-time-diagrams. Apply

-   MATLAB functions polyval and polyder to compute and evaluate the polynomials in dependence of time $$t$$,
-   MATLAB function lsim to compute position $$y_{p} ( t )$$.

f. In preparation of the following exercises, the high-pass filter $$G_{V p 1} ( s )$$ of the feed-forward controller shall be discretized in time.

Apply trapezoidal rule to approximate the continuous-time transfer function $$G_{V p 1} ( s )$$ by the discrete-time transfer function $$G_{V p 1}^{*} ( z )$$. Furthermore, compute the differences equation for the control signal $$u_{V p k}$$ in dependence of $$u_{V p 1 k}$$.

Required lab results:

-   mathematical expressions for $$Y_{p} ( s ) , \  E_{p} ( s ) , e_{y}$$, $$k_{p}$$
-   value and physical unit von $$k_{p}$$
-   signal-time-diagram of response $$y_{p} ( t )$$ to $$w_{p} ( t ) = v^{*} t h ( t )$$
-   root locus curves of control loop in dependence of $$k_{p}$$
-   MATLAB functions cd_refpoly_vmax and cd_refpoly_ff
-   signal-time-diagrams of $$w_{p} ( t ) , \  y_{p} ( t ) , \dot{w}_{p} ( t ) , \ddot{w}_{p} ( t )$$ and $$u_{V p 1} ( t )$$
-   discrete-time transfer function $$G_{V p 1}^{*} ( z ) \$$of high-pass filter of feed-forward controller
-   differences equation of control signal $$u_{V p k}$$ in dependence of $$u_{V p 1 k}$$
-   extended MATLAB script ex6_1.m containing the solutions of the individual exercise and to test the programmed MATLAB functions

Exercise 7.2: Extended Simulink Subsystem “Motion Control” for Longitudinal Position Control [Simulink]

Subsystem Motion Control in s07_sig_template.slx and c07_car0.slx shall be extended by longitudinal position control. Motion Control receives two possible types of driving maneuvers:

-   ManeuverTypePathFollow for speed-controlled free drive according to Chapter ‎6
-   ManeuverTypePark for longitudinal position control according to this chapter

This maneuver type is coded in element type of bus signal DRIVEMANEUVER. Depending on this value, Motion Control shall switch between speed control and longitudinal position control.

Please note that longitudinal position control contains speed control in its inner control loop as well as the reference signal generation and feed-forward control of Exercise 7.1. Thus, the speed control of Chapter ‎6 is always active. The outer-loop position controller $$G_{R p}$$, feed-forward controller $$G_{V p 1}$$ and reference signal generation $$\Sigma_{g e n}$$ are activated or deactivated depending on the maneuver type.

In case of driving maneuver ManeuverTypePark bus signal maneuver contains the following elements:

-   arc length $$x^{*}$$ to drive (xref)
-   maximum speed $$v^{*}$$ (vmax)

On activation of the longitudinal position controller (time $$t = t_{0} = 0 s$$), $$\Sigma_{g e n}$$ reads in the current actual position $$y_{p} ( t ) = x ( t )$$ to initialize $$x_{0} = x ( 0 )$$.

1.  Extend subsystem Motion Control in s07_sig_template.slx by
    -   reference signal generator $$\Sigma_{g e n}$$
    -   longitudinal position controller $$G_{R p}$$ with feed-forward controller $$G_{V p 1}$$
    -   detection of the driving maneuver type
    -   activation / deactivation of the longitudinal position controller depending on the maneuver type
2.  Test s07_sig_template.slx in MiL simulations by
    -   entering different values for type, vmax, xref as elements of bus signal DRIVEMANEUVER
    -   measuring position x and speed v as elements of bus signal CAROUTPUTSEXT
3.  Test subsystem Motion Control in SiL simulations on simulated MAD system in Linux by modification and execution of Python script send_maneuver.py. The reference arc length xref is entered as the third optional argument to send_maneuver.py.
4.  Test subsystem Motion Control in real drive tests on MAD system.

Required lab results:

-   extended Simulink model s07_sig_template.slx
-   extended model data file s07_data.m
-   signal-time-diagrams of reference position $$w_{p} ( t )$$ and actual position $$y_{p} ( t ) = x ( t )$$ from MiL and real drive tests

# Path Definition

*Path following* is one option for *vehicle trajectory control.* The other option would be *trajectory tracking*, where the reference path contains timing information. In trajectory tracking, the reference trajectory implicitly defines the vehicle speed at every point in time.

In *path following*, however, the vehicle speed can be specified independently of the reference path. This is a major advantage of path following over trajectory tracking in automated driving applications. The path can be followed at different speed profiles, whereas in trajectory tracking the speed profile cannot be adapted.

This and the following chapters address path following and no trajectory tracking.

## Frenet-Serret Formulas

The *reference path* is specified as a path parameterized in *arc length* $$x^{*}$$.

In two dimensions (driving on a horizontal planar surface) the reference path is defined as

|   | $$ \mathbf{s}^{\mathbf{*}} \left( x^{*} \right) = \begin{pmatrix}s_{1}^{*} \left( x^{*} \right) \\ s_{2}^{*} \left( x^{*} \right) \\ 0\end{pmatrix} \  \  ; \  \  \  \  x^{*} \in \lbrack 0 , x_{E} \rbrack $$ |   |
|---|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

For reasons of simplicity, the superscript $$*$$ is omitted which indicates the reference path.

|   | $$ \mathbf{s} ( x ) = \begin{pmatrix}s_{1} ( x ) \\ s_{2} ( x ) \\ 0\end{pmatrix} \  \  ; \  \  \  \  x \in \lbrack 0 , x_{E} \rbrack $$ | (‎8.1) |
|---|------------------------------------------------------------------------------------------------------------------------------------------|-------|

with

-   $$s_{1} ( x )$$ – x-coordinate (abscissa) in Cartesian inertial frame
-   $$s_{2} ( x )$$ – y-coordinate (ordinate) in Cartesian inertial frame
-   $$x \in \lbrack 0 , x_{E} \rbrack$$ – the *arc length*
-   $$x_{E}$$ – the total *path length*

The range of the arc length is from 0 (start of the path) to $$x_{E}$$ (end of the path = path length).

For *trajectory tracking* (in contrast to path following) the arc length $$x \$$would be further parameterized in *time* $$t$$:

|   | $$ x = x ( t ) \in \lbrack 0 , x_{E} \rbrack $$ |   |
|---|-------------------------------------------------|---|

For path following, the path is fully defined by (‎8.1).

Figure ‎8.1: Path on planar surface

Derived Quantities

The local circle with radius $$r ( x ) \$$approximates the path locally at every path point $$\mathbf{s} ( x ) = \left( s_{1} ( x ) \mathbf{, \ } s_{2} ( x ) \right)^{T}$$. The *tangential vector* $$\mathbf{t (} x \mathbf{)}$$ of the path is also the tangential vector of this circle. The *normal vector* $$\mathbf{n (} x \mathbf{)}$$ points towards the circle center.

From the definition of the reference path the following path quantities are derived which are needed for path following control.

The given formulas are widely known as the *Frenet-Serret formulas (Frenetsche Formeln)*.

*Infinitesimal arc length:*

|   | $$ d x = \sqrt{d s_{1}^{2} + d s_{2}^{2}} $$ |   |
|---|----------------------------------------------|---|

*Tangential vector*:

|   | $$ \mathbf{t} ( x ) = \frac{d \mathbf{s}}{d x} = \begin{pmatrix}\frac{d s_{1}}{d x} \\ \frac{d s_{2}}{d x} \\ 0\end{pmatrix} = \begin{pmatrix}s_{1}^{'} \\ s_{2}^{'} \\ 0\end{pmatrix} $$ |   |
|---|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

-   The tangential vector $$t$$ is the first derivative of $$\mathbf{s}$$ w.r.t. $$x$$.
-   The length of the $$\mathbf{t}$$ equals to one by definition:

|   | $$ \left| \mathbf{t} ( x ) \right| = \sqrt{\left( \frac{d s_{1}}{d x} \right)^{2} + \left( \frac{d s_{2}}{d x} \right)^{2}} = \frac{1}{d x} \sqrt{d s_{1}^{2} + d s_{2}^{2}} = 1 \ $$ |   |
|---|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

-   For the tangential, normal and rotational vectors, the third coordinate is introduced. This third coordinate is pointed upwards orthogonal to the planar surface. In case of the tangential vector, this third coordinate equals to zero.

*Local yaw angle*:

|   | $$ \  \psi ( x ) = a t a n 2 \left( d s_{2} , d s_{1} \right) = a t a n 2 \left( s_{2}^{'} , s_{1}^{'} \right) $$ | (‎8.2) |
|---|-------------------------------------------------------------------------------------------------------------------|-------|

*Local curvature*:

|   | $$ \  \kappa ( x ) = \left| \frac{d^{2} \mathbf{s}}{d x^{2}} \right| = \sqrt{{s_{1}^{' '} ( x )}^{2} + {s_{2}^{' '} ( x )}^{2}} = \frac{1}{r ( x )} $$ | (‎8.3) |
|---|--------------------------------------------------------------------------------------------------------------------------------------------------------|-------|

-   For straight lines the curvature is 0.
-   For circles the constant curvature is the inverse of the constant radius: $$\kappa = \frac{1}{r}$$.

*Normal vector:*

|   | $$ \mathbf{n} ( x ) = \frac{1}{\left| \frac{d^{2} \mathbf{s}}{d x^{2}} \right|} \frac{d^{2} \mathbf{s}}{d x^{2}} = \frac{1}{\left| \frac{d \mathbf{t}}{d x} \right|} \frac{d \mathbf{t}}{d x} \mathbf{\ } = \frac{1}{\left| \mathbf{t}^{'} \right|} \mathbf{t}^{\mathbf{'}} = \frac{1}{\sqrt{s_{1}^{' ' 2} + s_{2}^{' ' 2}}} \begin{pmatrix}\frac{d^{2} s_{1}}{d x^{2}} \\ \frac{d^{2} s_{2}}{d x^{2}} \\ 0\end{pmatrix} = \frac{1}{\sqrt{s_{1}^{' ' 2} + s_{2}^{' ' 2}}} \begin{pmatrix}s_{1}^{' '} \\ s_{2}^{' '} \\ 0\end{pmatrix} = \frac{1}{\kappa ( x )} \begin{pmatrix}s_{1}^{' '} \\ s_{2}^{' '} \\ 0\end{pmatrix} $$ |   |
|---|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

-   The normal vector $$\mathbf{n}$$ is the normalized second derivative of $$\mathbf{s}$$ w.r.t. $$x$$.
-   The normal vector $$\mathbf{n}$$ is the normalized first derivative of $$\mathbf{t}$$ w.r.t. $$x$$.
-   The normal vector $$\mathbf{n \ }$$is orthogonal to the reference path direction $$\mathbf{t}$$.
-   The length of the $$\mathbf{n}$$ equals to one.

*Binormal vector:*

|   | $$ \mathbf{t \times n} = \frac{1}{\sqrt{s_{1}^{' ' 2} + s_{2}^{' ' 2}}} \begin{pmatrix}0 \\ 0 \\ s_{1}^{'} s_{2}^{' '} - s_{1}^{' '} s_{2}^{'}\end{pmatrix} = \frac{1}{\kappa ( x )} \begin{pmatrix}0 \\ 0 \\ s_{1}^{'} s_{2}^{' '} - s_{1}^{' '} s_{2}^{'}\end{pmatrix} $$ | (‎8.4) |
|---|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------|

-   The binormal vector is the vector product (cross product) of $$\mathbf{t}$$ and $$\mathbf{n}$$.
-   The third element of this rotation vector is positive if the reference path is bound to the left (left turn) according to the *right-hand rule*.
-   It is negative if the path is bound to the right (right turn).

## Example: Circular Arc

Left-turned Circular Arc

The path of a *left-turned circular arc* with

-   radius $$r$$
-   center point $$\left( s_{c 1} , s_{c 2} \right)^{T}$$
-   initial yaw angle $$\psi_{0}$$

is defined as

|   | $$ \mathbf{s} ( x ) = \begin{pmatrix}s_{1} ( x ) \\ s_{2} ( x )\end{pmatrix} = \begin{pmatrix}s_{c 1} + r \cos \left( \frac{x}{r} + \psi_{0} - \frac{\pi}{2} \right) \\ s_{c 2} + r \sin \left( \frac{x}{r} + \psi_{0} - \frac{\pi}{2} \right) \\ 0\end{pmatrix} $$ |   |
|---|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

with $$x \in \lbrack 0 , x_{E} \rbrack$$.

Figure ‎8.2: Left-turned circular arc

For a full circle, the path length $$x_{E}$$ equals to $$2 \pi r$$.

In the special case of $$\psi_{0} = \frac{\pi}{2}$$ the derived quantities of the circular arc are:

|   | $$ s_{1} = s_{c 1} + r \cos \frac{x}{r} $$                                                                                                                                                                |   |
|---|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|
|   | $$ s_{2} = s_{c 2} + r \sin \frac{x}{r} $$                                                                                                                                                                |   |
|   | $$ s_{1}^{'} = - \sin \frac{x}{r} $$                                                                                                                                                                      |   |
|   | $$ s_{2}^{'} = \cos \frac{x}{r} $$                                                                                                                                                                        |   |
|   | $$ s_{1}^{' '} = - \frac{1}{r} \cos \frac{x}{r} $$                                                                                                                                                        |   |
|   | $$ s_{2}^{' '} = - \frac{1}{r} \sin \frac{x}{r} $$                                                                                                                                                        |   |
|   | $$ d s_{1} = \frac{d s_{1}}{d x} d x = - \sin \frac{x}{r} d x $$                                                                                                                                          |   |
|   | $$ d s_{2} = \frac{d s_{2}}{d x} d x = \cos \frac{x}{r} d x $$                                                                                                                                            |   |
|   | $$ \sqrt{d s_{1}^{2} + d s_{2}^{2}} = d x $$                                                                                                                                                              |   |
|   | $$ \psi = {a t a n} \frac{s_{2}^{'}}{s_{1}^{'}} = - {a t a n} \frac{1}{\tan \frac{x}{r}} = \frac{x}{r} + \frac{\pi}{2} $$                                                                                 |   |
|   | $$ \kappa = \sqrt{s_{1}^{' '}^{2} + s_{2}^{' ' 2}} = \sqrt{\left( \frac{1}{r} \cos \frac{x}{r} \right)^{2} + \left( \frac{1}{r} \sin \frac{x}{r} \right)^{2}} = \frac{1}{r} = c o n s t $$                |   |
|   | $$ \mathbf{t} = \begin{pmatrix}s_{1}^{'} \\ s_{2}^{'} \\ 0\end{pmatrix} = \begin{pmatrix}- \sin \frac{x}{r} \\ \cos \frac{x}{r} \\ 0\end{pmatrix} $$                                                      |   |
|   | $$ \mathbf{n} = \frac{1}{\sqrt{s_{1}^{' ' 2} + s_{2}^{' ' 2}}} \begin{pmatrix}s_{1}^{' '} \\ s_{2}^{' '} \\ 0\end{pmatrix} = \begin{pmatrix}- \cos \frac{x}{r} \\ - \sin \frac{x}{r} \\ 0\end{pmatrix} $$ |   |
|   | $$ \mathbf{t \times n} = \frac{1}{\sqrt{s_{1}^{' ' 2} + s_{2}^{' ' 2}}} \begin{pmatrix}0 \\ 0 \\ s_{1}^{'} s_{2}^{' '} - s_{1}^{' '} s_{2}^{'}\end{pmatrix} = \begin{pmatrix}0 \\ 0 \\ 1\end{pmatrix} $$  |   |

Right-Turned Circular Arc

The path of a *right-turned circular arc* is similarly defined as:

|   | $$ \begin{pmatrix}s_{1} ( x ) \\ s_{2} ( x )\end{pmatrix} = \begin{pmatrix}s_{c 1} - r \cos \left( - \frac{x}{r} + \psi_{0} - \frac{\pi}{2} \right) \\ s_{c 2} - r \sin \left( - \frac{x}{r} + \psi_{0} - \frac{\pi}{2} \right) \\ 0\end{pmatrix} $$ |   |
|---|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

Again, the special case of $$\psi_{0} = \frac{\pi}{2}$$ is considered. In this case, please note that the third element of the rotation vector $$\mathbf{t \times n}$$ is negative:

|   | $$ \begin{pmatrix}s_{1} ( x ) \\ s_{2} ( x )\end{pmatrix} = \begin{pmatrix}s_{c 1} - r \cos \left( - \frac{x}{r} \right) \\ s_{c 2} - r \sin \left( - \frac{x}{r} \right) \\ 0\end{pmatrix} = \begin{pmatrix}s_{c 1} - r \cos \left( \frac{x}{r} \right) \\ s_{c 2} + r \sin \left( \frac{x}{r} \right) \\ 0\end{pmatrix} $$ |   |
|---|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|
|   | $$ \mathbf{t} = \begin{pmatrix}s_{1}^{'} \\ s_{2}^{'} \\ 0\end{pmatrix} = \begin{pmatrix}\sin \frac{x}{r} \\ \cos \frac{x}{r} \\ 0\end{pmatrix} $$                                                                                                                                                                           |   |
|   | $$ \mathbf{n} = \frac{1}{\sqrt{s_{1}^{' ' 2} + s_{2}^{' ' 2}}} \begin{pmatrix}s_{1}^{' '} \\ s_{2}^{' '} \\ 0\end{pmatrix} = \begin{pmatrix}\cos \frac{x}{r} \\ - \sin \frac{x}{r} \\ 0\end{pmatrix} $$                                                                                                                      |   |
|   | $$ \mathbf{t \times n} = \frac{1}{\sqrt{s_{1}^{' ' 2} + s_{2}^{' ' 2}}} \begin{pmatrix}0 \\ 0 \\ s_{1}^{'} s_{2}^{' '} - s_{1}^{' '} s_{2}^{'}\end{pmatrix} = \begin{pmatrix}0 \\ 0 \\ - 1\end{pmatrix} $$                                                                                                                   |   |

The curvature, however, is positive both for left and right turns:

|   | $$ \kappa = \sqrt{\left( \frac{1}{r} \cos \frac{x}{r} \right)^{2} + \left( \frac{1}{r} \sin \frac{x}{r} \right)^{2}} = \frac{1}{r} $$ |   |
|---|---------------------------------------------------------------------------------------------------------------------------------------|---|

## Clothoids

*Clothoids* together with straight lines and circular arcs are the construction elements of roads and rail roads.

Based on clothoids, left-turned and right-turned curves are constructed.

Clothoids are the transition elements between straight lines.

The characteristic of closing (opening) clothoids is the linear increase (decrease) of the curvature $$\kappa ( x )$$ with increasing arc length $$x$$.

This characteristic is a requirement for comfortable vehicle driving:

-   The steering angle is increasing linearly with the arc length $$x$$.
-   The lateral vehicle acceleration $$a_{l a t e r a l}$$ is continuous, since the angular yaw velocity $$\dot{\psi}$$ is continuous.

Non-continuous lateral accelerations would be uncomfortable for car or train passengers and would arise when curves were based on circular arcs without clothoids.

Definition of Clothoids

The linear increase of the curvature for closing clothoids is written as:

|   | $$ \kappa ( x ) = \frac{1}{r ( x )} = \sqrt{s_{1}^{' '}^{2} + s_{2}^{' '}^{2}} = a \cdot x $$ |   |
|---|-----------------------------------------------------------------------------------------------|---|

The curvature equals to the inverse of the local radius $$r ( x )$$ that is the radius of the local circle that approximates the curve at arc length $$x$$ (see Section **Fehler! Verweisquelle konnte nicht gefunden werden.**).

Clothoids are divided into the four types:

-   left-turned closing clothoids
-   right-turned closing clothoids
-   left-turned opening clothoids
-   right-turned closing clothoids

The parameter $$a$$ specifies the shape of the clothoid.

-   For left-turned clothoids, $$a$$ is positive: $$a > 0$$
-   For right-turned clothoids, $$a$$ is negative: $$a < 0$$
-   The greater the absolute of $$a \$$is, the steeper the curve is.

Examples of clothoids are depicted in Figure ‎8.3.

-   All clothoids start in the origin $${\mathbf{s} ( 0 ) \mathbf{= s}}_{\mathbf{0}} = ( 0 , 0 )^{T}$$ of the plane and have an initial yaw angle $${\psi ( 0 ) = \psi}_{0} = 0$$.
-   The clothoid parameter equals to $$a = 1$$ or $$a = - 1$$ in the depicted cases.

![](media/2f28f80271a282da295e77b990438b09.emf)

Figure ‎8.3: The four types of clothoids. MATLAB script [clothoids\\spiro_plot.m](file:///C:\Users\ftraenkle\AppData\Roaming\Microsoft\Word\clothoids\spiro_plot.m).

From differential equations, the curve definition of clothoids can be derived. This derivation is elaborated in the appendix. The curve definition contains integrals that can be calculated by *Fresnel integral functions* or numerically in MATLAB, for instance.

In the following two paragraphs, the curve definitions of left-turned and right-turned clothoids are given as a summary of the appendix.

All curves have the following properties:

-   starting point $$\mathbf{s}_{\mathbf{0}} = \mathbf{s} ( 0 ) = \left( s_{10} , s_{20} \right)^{T}$$
-   initial yaw angle $$\psi ( 0 ) = \psi_{0}$$
-   arc length $$x \in \lbrack 0 ; x_{E} \rbrack$$ where $$x_{E}$$ is the path length

Closing Clothoids

|   | $$ s_{1} ( x ) = s_{01} + \int_{0}^{x} {\cos \left( \frac{a}{2} \xi^{2} + \psi_{0} \right)} d \xi $$ |   |
|---|------------------------------------------------------------------------------------------------------|---|
|   | $$ s_{2} ( x ) = s_{02} + \int_{0}^{x} {\sin \left( \frac{a}{2} \xi^{2} + \psi_{0} \right)} d \xi $$ |   |
|   | $$ \kappa ( x ) = a \cdot x $$                                                                       |   |
|   | $$ \psi ( x ) = \frac{a}{2} x^{2} + \psi_{0} $$                                                      |   |

Opening Clothoids

|   | $$ s_{1} ( x ) = s_{01} + \int_{0}^{x} {\cos \left( - \frac{a}{2} \xi^{2} + a x_{E} \xi + \psi_{0} \right)} d \xi $$ |   |
|---|----------------------------------------------------------------------------------------------------------------------|---|
|   | $$ s_{2} ( x ) = s_{02} + \int_{0}^{x} {\sin \left( - \frac{a}{2} \xi^{2} + a x_{E} \xi + \psi_{0} \right)} d \xi $$ |   |
|   | $$ \kappa ( x ) = a \cdot ( x_{E} - x ) $$                                                                           |   |
|   | $$ \psi ( x ) = - \frac{a}{2} \left( x_{E} - x \right)^{2} + \psi_{0} + \frac{a}{2} x_{E}^{2} $$                     |   |
|   | $$ \psi \left( x_{E} \right) = \frac{a}{2} x_{E}^{2} + \psi_{0} $$                                                   |   |

## C++ MAD Library

The C++ MAD library madlib provides C++ classes to construct tracks for path following control:

-   modbas::Track represents the complete track and provides methods to interpolate lines on the track by splines.
-   modbas::TrackSegment is the base class of track segments from which the track is constructed.
-   modbas::StraightSegment represents a straight segment.
-   modbas::CircleSegment represents a circular segment.
-   modbas::ClothoidSegment represents an opening or closing clothoid segment.
-   modbas::Spline represents a cubic spline and provides methods to interpolate the spline or to compute the a point on the spline closest to the car position. The class modbas::Spline is used to represent any path on the track. Special cases of this path are: left boundary of track, right boundary, center line.

Visit the corresponding header files in folder madlib/include for details.

Most of all, the ROS node track_node (see Figure ‎3.4) uses these classes to construct tracks from track segments and to create visualization markers to display the track with rviz. The ROS node track_node further provides the ROS service /mad/get_waypoints which will be used to retrieve waypoints of the track for path following control.

The following code as part of track_node.cpp constructs an oval track depicted in Figure ‎8.4:

const float a1total { 2.700F }; // total surface width [ m ]

const float a2total { 1.800F }; // total surface height [ m ]

const float a1boundary { 0.05F }; // boundary for markers [ m ]

const float a2boundary { 0.05F }; // boundary for markers [ m ]

const float a1 { a1total - 2.0F \* a1boundary };

const float a2 { a2total - 2.0F \* a2boundary };

const float width { 0.25F \* a2 }; // track width [ m ]

modbas::Track track { a1total, a2total, 0.01F };

std::shared_ptr\<modbas::TrackPose\> pose { new modbas::TrackPose(0.0F, a1boundary + width, a2boundary + 0.5F \* width, 0.0F) };

track.addPose(pose);

track.addSegment(std::shared_ptr\<modbas::StraightSegment\>(

new modbas::StraightSegment(pose, width, a1 - 2.0F \* width)));

track.addSegment(std::shared_ptr\<modbas::CircleSegment\>(

new modbas::CircleSegment(pose, width, 0.5F \* width, 0.5F \* modbas::Utils::pi)));

track.addSegment(std::shared_ptr\<modbas::StraightSegmnt\>(

new modbas::StraightSegment(pose, width, a2 - 2.0F \* width)));

track.addSegment(std::shared_ptr\<modbas::CircleSegment\>(

new modbas::CircleSegment(pose, width, 0.5F \* width, 0.5F \* modbas::Utils::pi)));

track.addSegment(std::shared_ptr\<modbas::StraightSegment\>(

new modbas::StraightSegment(pose, width, a1 - 2.0F \* width)));

track.addSegment(std::shared_ptr\<modbas::CircleSegment\>(

new modbas::CircleSegment(pose, width, 0.5F \* width, 0.5F \* modbas::Utils::pi)));

track.addSegment(std::shared_ptr\<modbas::StraightSegment\>(

new modbas::StraightSegment(pose, width, a2 - 2.0F \* width)));

track.addSegment(std::shared_ptr\<modbas::CircleSegment\>(

new modbas::CircleSegment(pose, width, 0.5F \* width, 0.5F \* modbas::Utils::pi)));

if (!track.finalizeCircuit()) {

ROS_ERROR("finalizing circuit failed");

}

The track surface has a total width of $$a_{1 t o t a l} =$$2700mm and a total height of $$a_{2 t o t a l} =$$1800mm. The origin of the coordinate system is the lower-left corner. The starting point of the track with arc length $$x = 0$$m is at $$\left( s_{1} , s_{2} \right) = \left( a_{1 b o u n d a r y} + w , a_{2 b o u n d a r y} + \frac{w}{2} \right)$$ where $$w$$ is the width of the track and $$a_{1 b o u n d a r y}$$, $$a_{2 b o u n d a r y}$$ are the safety boundaries.

![](media/6da18949026472ae6ad87539bfb306e3.png)

Figure ‎8.4: Oval track built from four straight lines and four circular arcs.

## MATLAB MODBAS CAR Library

The MODBAS CAR library (mbc) provides MATLAB functions to construct tracks for path following control:

-   mbc_track_create creates a new track
-   mbc_circle_create adds a circular arc segment to the track
-   mbc_straight_create adds a straight line segment to the track
-   mbc_track_display interpolates the track by splines and displays the track in a MATLAB figure

Call help on the individual MATLAB functions for more information, z.B.

help mbc_circle_create

However, MATLAB MODBAS CAR library provides fewer features than the C++ MAD library.

The following MATLAB script creates and displays the oval track similar to the one of Figure 7.4.

%% Road Surface

a1total = 2.7; % total surface width [ m ]

a2total = 1.8; % total surface height [ m ]

a1boundary = 0.05; % margin [ m ]

a2boundary = 0.05; % margin [ m ]

a1 = a1total - 2 \* a1boundary; % track width [ m ]

a2 = a2total - 2 \* a2boundary; % track height [ m ]

width = 0.25 \* a2; % track width [ m ]

%% Create Oval

track = mbc_track_create(a1boundary + width, a2boundary + 0.5 \* width, 0);

track = mbc_straight_create(track, a1 - 2 \* width, width);

track = mbc_circle_create(track, 0.5 \* width, 0.5 \* pi, width);

track = mbc_straight_create(track, a2 - 2 \* width, width);

track = mbc_circle_create(track, 0.5 \* width, 0.5 \* pi, width);

track = mbc_straight_create(track, a1 - 2 \* width, width);

track = mbc_circle_create(track, 0.5 \* width, 0.5 \* pi, width);

track = mbc_straight_create(track, a2 - 2 \* width, width);

track = mbc_circle_create(track, 0.5 \* width, 0.5 \* pi, width);

track = mbc_track_display(track, 0.1, [ 0 a1total 0 a2total ]);

path = track.center;

## Cubic Splines

*Splines* are piecewise *polynomials* to interpolate tabulated functions.

Take a look at (William Press, 2007) for a good reference of the derivation and implementation of cubic splines.

The planar reference path is defined by *waypoints (data points)*:

|   | $$ \mathbf{s}_{i} = \begin{pmatrix}s_{1 i} \\ s_{2 i} \\ 0\end{pmatrix} = \mathbf{s} \left( x_{i} \right) = \begin{pmatrix}s_{1} ( x_{i} ) \\ s_{2} ( x_{i} ) \\ 0\end{pmatrix} \  \  \  ; \  \  i = 1 , \ldots , n $$ |   |
|---|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

The domain of the arc length $$x \in \lbrack 0 ; x_{E} \rbrack$$

-   is divided into $$n - 1$$ *intervals*
-   by the $$n$$ discrete strictly increasing *sampling points* $$x_{i}$$.

Note: The sampling points $$x_{i}$$ do not have to be equidistant.

-   Both coordinates $$s_{1} ( x )$$ and $$s_{2} ( x )$$ of the path are *independently* interpolated by two piecewise polynomials (cubic splines).
-   We will first consider one coordinate $$s_{1} ( x )$$ and introduce

|   | $$ y ( x ) = s_{1} ( x ) $$                                                                                        |   |
|---|--------------------------------------------------------------------------------------------------------------------|---|
|   | $$ y_{i} = y \left( x_{i} \right) = s_{1} \left( x_{i} \right) = s_{1 i} \  \  \  \  ; \  \  i = 1 , \ldots , n $$ |   |

for reasons of abbreviation.

-   The interpolation of $$s_{2} ( x )$$ follows exactly the same rules as for $$y ( x ) = s_{1} ( x )$$.

The requirements for the piecewise polynomial (spline) are:

1.  The spline traverses the way points.
2.  The spline is *continuous* in its second derivative (*2-fach stetig differenzierbar,* $$C_{2}$$),

    i.e., the second derivative has no discontinuities at the waypoints.

The third derivative may have discontinuities at the waypoints.

A common approach to fulfill these requirements is to use *cubic splines* as discussed in the following paragraph.

Piecewise Cubic Polynomials

For each interval $$x \in \left\lbrack x_{i} ; x_{i + 1} \right\rbrack , \  \  \  i = 1 , \ldots , n - 1$$, a different piecewise cubic polynomial is constructed:

|   | $$ y ( x ) = A_{i} ( x ) y_{i} + B_{i} ( x ) y_{i + 1} + C_{i} ( x ) y_{i}^{' '} + D_{i} ( x ) y_{i + 1}^{' '} $$ | (‎8.5) |
|---|-------------------------------------------------------------------------------------------------------------------|-------|

with

|   | $$ A_{i} ( x ) = \frac{x_{i + 1} - x}{x_{i + 1} - x_{i}} $$                                                        |   |
|---|--------------------------------------------------------------------------------------------------------------------|---|
|   | $$ B_{i} ( x ) = \frac{x - x_{i}}{x_{i + 1} - x_{i}} $$                                                            |   |
|   | $$ C_{i} ( x ) = \frac{1}{6} \lbrack A_{i} ( x )^{3} - A_{i} ( x ) \rbrack \left( x_{i + 1} - x_{i} \right)^{2} $$ |   |
|   | $$ D_{i} ( x ) = \frac{1}{6} \lbrack B_{i} ( x )^{3} - B_{i} ( x ) \rbrack \left( x_{i + 1} - x_{i} \right)^{2} $$ |   |

and the values and second derivatives of $$y ( x )$$ at the sampling points $$x_{i}$$ and $$x_{i + 1} :$$

|   | $$ y_{i} = y \left( x_{i} \right) \  \  \  ; \  \  \  i = 1 , \ldots , n $$  |   |
|---|------------------------------------------------------------------------------|---|
|   | $$ y_{i}^{' '} = y^{' '} ( x_{i} ) \  \  \  ; \  \  \  i = 1 , \ldots , n $$ |   |

Figure ‎8.5 depicts a spline (blue line) interpolating 6 data points (red circles).

The dashed black line is the cubic polynomial of the interval $$\left\lbrack x_{3} , x_{4} \right\rbrack = \lbrack 2 , 3 \rbrack$$, i.e., $$i = 3$$.

Only in this interval, the approximation of this cubic polynomial is good.

Outside this interval, the performance of this polynomial is poor and other polynomials are applied.

![](media/7da55a29c13a11f90cd29e3f76938830.emf)

Figure ‎8.5: Cubic spline (blue line) and cubic polynomial for interval [2, 3] (dashed line)

The interpolation rule (‎8.5) fulfills the requirements above, which is shown in the following.

First, we consider the value of $$y ( x )$$ at sampling point $$x_{i}$$:

|   | $$ y \left( x_{i} \right) = A_{i} \left( x_{i} \right) y_{i} + B_{i} \left( x_{i} \right) y_{i + 1} + C_{i} \left( x_{i} \right) y_{i}^{' '} + D_{i} \left( x_{i} \right) y_{i + 1}^{' '} $$ |   |
|---|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

All $$B_{i} \left( x_{i} \right) , \  C_{i} \left( x_{i} \right) , \  D_{i} ( x_{i} )$$ equal to 0 except for $$A_{i} \left( x_{i} \right) = 1$$ and thus

|   | $$ y \left( x_{i} \right) = y_{i} $$ |   |
|---|--------------------------------------|---|

The piecewise polynomial in interval $$x \in \lbrack x_{i} ; x_{i + 1} \rbrack$$ exactly matches the way point at $$x_{i}$$.

At sampling point $$x_{i + 1}$$ we have the following situation:

|   | $$ y \left( x_{i + 1} \right) = A_{i} \left( x_{i + 1} \right) y_{i} + B_{i} \left( x_{i + 1} \right) y_{i + 1} + C_{i} \left( x_{i + 1} \right) y_{i}^{' '} + D_{i} \left( x_{i + 1} \right) y_{i + 1}^{' '} $$ |   |
|---|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|
|   | $$ A_{i} \left( x_{i + 1} \right) = C_{i} \left( x_{i + 1} \right) = D_{i} \left( x_{i + 1} \right) = 0 $$                                                                                                       |   |
|   | $$ B_{i} \left( x_{i + 1} \right) = 1 $$                                                                                                                                                                         |   |

and thus

|   | $$ y \left( x_{i + 1} \right) = y_{i + 1} $$ |   |
|---|----------------------------------------------|---|

The piecewise polynomial in interval $$x \in \lbrack x_{i} ; x_{i + 1} \rbrack$$ exactly matches the waypoint at $$x_{i + 1}$$. The first requirement is fulfilled.

Now we take a look at the first and second derivatives of the polynomial (‎8.5). After some intermediate steps we find:

|   | $$ y^{'} ( x ) = \frac{y_{i + 1} - y_{i}}{x_{i + 1} - x_{i}} - \frac{3 A_{i} ( x )^{2} - 1}{6} \left( x_{i + 1} - x_{i} \right) y_{i}^{' '} + \frac{3 B_{i} ( x )^{2} - 1}{6} \left( x_{i + 1} - x_{i} \right) y_{i + 1}^{' '} $$ | (‎8.6) |
|---|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------|
|   | $$ y^{' '} ( x ) = A_{i} ( x ) y_{i}^{' '} + B_{i} ( x ) y_{i + 1}^{' '} $$                                                                                                                                                       |       |

and

|   | $$ y^{' '} \left( x_{i} \right) = y_{i} ' ' $$           |   |
|---|----------------------------------------------------------|---|
|   | $$ y^{' '} \left( x_{i + 1} \right) = y_{i + 1}^{' '} $$ |   |

The second derivative of $$y ( x )$$ is thus continuous at the sampling points $$x_{i}$$. Thus, the second requirement is fulfilled.

Now we need to address the question how the second derivatives $$y_{i}^{' '}$$ at the sampling points are determined required for the interpolation equation (‎8.5). So far we only know $$y_{i}$$.

For this, we consider the requirement of continuity of the first derivative $$y ' ( x )$$. By setting (‎8.6) evaluated for $$x = x_{i}$$ in interval $$\lbrack x_{i - 1} , x_{i} \rbrack$$ equal to (‎8.6) evaluated for $$x = x_{i}$$ in interval $$\lbrack x_{i} , x_{i + 1} \rbrack$$ the required equations for $$y_{i}^{' '}$$ are obtained:

|   | $$ \frac{x_{i} - x_{i - 1}}{6} y_{i - 1}^{' '} + \frac{x_{i + 1} - x_{i - 1}}{3} y_{i}^{' '} + \frac{x_{i + 1} - x_{i}}{6} y_{i + 1}^{' '} = \frac{y_{i + 1} - y_{i}}{x_{i + 1} - x_{i}} - \frac{y_{i} - y_{i - 1}}{x_{i} - x_{i - 1}} $$ $$ \  i = 2 , \ldots , n - 1 $$ | (‎8.7) |
|---|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------|

These are $$n - 2$$ linear equations for $$n$$ unknowns $$y_{i}^{' '}$$. Thus, there are two equations missing.

There are different options to obtain the remaining two equations from the boundary conditions of the spline:

-   *not-a-knot end conditions*: the third derivative $$y ' ' ' ( x_{i} )$$ is required to be continuous for $$i = 2$$ and $$i = n - 1$$
-   *natural end conditions*: the second derivative $$y ' ' ( x_{i} )$$ is set to 0 at both ends of the spline for $$i = 1$$ and $$i = n$$
-   *given first-order end derivatives*: the first derivatives $$y^{'} ( x_{i} )$$ are specified at $$i = 1$$ and $$i = n$$.
-   $$p e r i o d i c \  e n d \  c o n d i t i o n s$$*:* $$y^{'} \left( x_{1} \right) = y ' ( x_{n} )$$ and $$y^{' '} \left( x_{1} \right) = y ' ' ( x_{n} )$$

In the case of periodic splines, the waypoints at $$x_{1}$$ and $$x_{n}$$ have to be set equal in this case: $$y \left( x_{1} \right) = y ( x_{n} )$$. *Periodic splines* are applied to define the circular paths of race tracks.

Note: The linear equation system (‎8.7) has a tridiagonal form and can be solved with little computational effort $$O ( n )$$, which is one of the great advantages of cubic splines. For natural end conditions, Thomas algorithm can be applied. For periodic end conditions, a perturbed tridiagonal system has to be solved by applying Thomas algorithm together with the Sherman-Morrison formula:

<https://www.cfd-online.com/Wiki/Tridiagonal_matrix_algorithm_-_TDMA_(Thomas_algorithm)>

The $$n$$ constant values $$y ' ' ( x_{i} )$$ obtained from (‎8.6) and the two additional equations are inserted into the piecewise interpolation equation (‎8.5).

This piecewise polynomial equation can then be rewritten as:

|   | $$ y ( x ) = c_{i 3} {( x - x_{i} )}^{3} + c_{i 2} ( {x - x_{i} )}^{2} + c_{i 1} ( x - x_{i} ) + c_{i 0} $$  for $$x \in \left\lbrack x_{i} ; x_{i + 1} \right\rbrack , \  \  i = 1 , \ldots , n - 1$$ | (‎8.8) |
|---|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------|

with the constant parameters

|   | $$ c_{i 3} = \frac{1}{6 h_{i}} \left( - y_{i}^{' '} + y_{i + 1}^{' '} \right) $$                                                                                                                                                          |   |
|---|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|
|   | $$ c_{i 2} = \frac{y_{i}^{' '}}{2} $$                                                                                                                                                                                                     |   |
|   | $$ c_{i 1} = \frac{1}{6 h_{i}} \left( - 6 y_{i} + 6 y_{i + 1} - 2 h_{i}^{2} y_{i}^{' '} - h_{i}^{2} y_{i + 1}^{' '} \right) = \frac{- y_{i} + y_{i + 1}}{h_{i}} - \frac{h_{i}}{6} \left( 2 y_{i}^{' '} + h_{i} y_{i + 1}^{' '} \right) $$ |   |
|   | $$ c_{i 0} = y_{i} $$                                                                                                                                                                                                                     |   |
|   | $$ h_{i} = x_{i + 1} - x_{i} $$                                                                                                                                                                                                           |   |

Computing Cubic Splines in MATLAB

Piecewise polynomials are available as a data type in MATLAB.

For the interpolation of data points by cubic splines the MATLAB provides the function spline, which supports *not-a-knot end conditions* or *given first-order end derivatives*.

pp = spline(xi, yi)

-   xi are the sampling points (vector)
-   yi are the values at the sampling points (vector)
-   pp is a MATLAB structure of the piecewise polynomial

For instance,

xi = [ 0 1 2 3 4 5 ];

yi = [ -1 1 -1 1 -1 1 ];

pp = spline(xi, yi);

calculates the spline depicted in Figure ‎8.5.

For this, spline solves the linear matrix equation (‎8.6) and the two additional *not-a-knot end conditions*.

The value of pp is:

pp =

form: 'pp'

breaks: [0 1 2 3 4 5]

coefs: [5x4 double]

pieces: 5

order: 4

dim: 1

The element coefs of the structure pp is the matrix representation of the polynomial coefficients $$c_{i 1}$$ in (‎8.8):

\>\> pp.coefs

ans =

2.2222 -8.6667 8.4444 -1.0000

2.2222 -2.0000 -2.2222 1.0000

\-3.1111 4.6667 0.4444 -1.0000

2.2222 -4.6667 0.4444 1.0000

2.2222 2.0000 -2.2222 -1.0000

The MATLAB function csape of the Curve Fitting Toolbox provides more control on the interpolation of splines. It can be used as an alternative to spline (if this toolbox is available).

For instance,

pp = csape(xi, yi, 'periodic')

creates a cubic spline with *periodic end conditions* for circular race tracks.

To evaluate piecewise polynomials, MATLAB provides the function ppval:

y = ppval(pp, x)

ppval evaluates the piecewise polynomial at x according to (‎8.8).

In order to save computation time, (‎8.8) is rearranged to

|   | $$ y ( x ) = \left( x - x_{i} \right) \left\{ \left( x - x_{i} \right) \left\lbrack \left( x - x_{i} \right) c_{i 3} + c_{i 2} \right\rbrack + c_{i 1} \right\} + c_{i 0} $$ | (‎8.9) |
|---|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------|

This allows the implementation of (‎8.9) as an iterative loop. See the MATLAB MODBAS CAR library function mbc_ppval for an embedded version of ppval.

In order to plot piecewise polynomials, standard plot commands may be used in connection with ppval. MATLAB further provides the function fnplt to plot piecewise polynomials

fnplt(pp)

For the options of fnplt, please refer to help fnplt

Cubic Spline Derivatives

An advantage of cubic splines is the analytical computation of derivatives.

From (‎8.8) the first and second derivatives of the cubic spline is obtained as:

|   | $$ y ' ( x ) = 3 c_{i 3} {( x - x_{i} )}^{2} + 2 c_{i 2} ( x - x_{i} ) + c_{i 1} $$ | (‎8.10) |
|---|-------------------------------------------------------------------------------------|--------|
|   | $$ y ' ' ( x ) = 6 c_{i 3} ( x - x_{i} ) + 2 c_{i 2} $$                             | (‎8.11) |

for $$x \in \left\lbrack x_{i} ; x_{i + 1} \right\rbrack , \  \  i = 1 , \ldots , n - 1$$.

MATLAB provides the function fnder to analytically compute the derivatives of a piecewise polynomial:

ppd = fnder(pp, n)

computes the $$n^{t h}$$ derivative of the piecewise polynomial pp and returns a new piecewise polynomial ppd for the derivative.

## Exercises

Exercise 8.1 Straight Line [C++, Simulink]

1.  Derive the parameterized curve definition $$\mathbf{s} ( x ) = \left( s_{1} ( x ) , s_{2} ( x ) \right)^{T}$$ of a straight line. The initial point of the straight line is at position $${\mathbf{s} ( 0 ) = \mathbf{s}_{\mathbf{0}} = \left( s_{01} , s_{02} \right)}^{T}$$. The yaw angle at the initial point is given as $$\psi ( 0 ) = \psi_{0}$$.
1.  Calculate the tangent vector, the normal vector and the curvature of the straight line.

Required lab results:

-   $$
    \mathbf{s} ( x ) , \mathbf{\  t} ( x ) , \  \mathbf{n} ( x ) , \  \kappa ( x )
    $$

Exercise 8.2 MODBAS CAR Functions for Clothoids [Simulink]

Extend the MODBAS CAR library by new MATLAB functions for clothoids:

track = mbc_clothoid_create(track, a, rad, w, opening)

points = mbc_clothoid_get_points(track, idx, xstart, dx, alpha)

Please refer to the corresponding functions mbc_circle_create and mbc_circle_get_points of circular arcs for the meanings of the function arguments and return values.

Use mbc_circle_create and mbc_circle_get_points as templates for the new functions.

Further, extend mbc_spline_create to handle clothoids next to straight lines and circles.

The parameter a specifies the shape parameter $$a$$ of the clothoid used in Section ‎8.3. This parameter is always positive.

The parameter rad is the difference angle of the starting yaw angle $$\psi_{0} = \psi ( 0 )$$ and the final yaw angle $$\psi_{E} = \psi ( x_{E} )$$.

-   rad is positive for left-turned clothoids
-   rad is negative for right-turned clothoids

The final sign of parameter $$a$$ of the clothoid as is determined by the sign of rad.

The optional parameter opening specifies an opening or closing clothoid:

-   opening is set to one for opening clothoids
-   opening is set to zero or omitted for closing clothoids

Note: You may use MATLAB function integral for necessary numerical computation of the Fresnel integrals.

Test the new functions by programming oval circuit of the BUGA track **without parking spaces**.

![](media/6726d5d216046c6f6e935a12c0eface3.png)

The specification of the oval circuit is:

-   The total size is 2700mm by 1800mm.
-   The green margin boundary is 50mm.
-   The track width is 200mm.
-   The starting position is at $$\left( s_{10} , s_{20} \right)^{T} = ( 1 5 0 m m , 9 0 0 m m )^{T}$$.
-   The circuit starts with orientation angle $$\psi_{0} = - \frac{\pi}{2}$$.
-   The circuit shall be constructed from 5 straight segments, 4 closing clothoids and 4 opening clothoids.
-   The clothoid parameter shall be $$a = 8$$.

Required lab results:

-   MATLAB script mbc_clothoid_create.m
-   MATLAB script mbc_clothoid_get_points.m
-   modified MATLAB script s07_data.m

# Path Following Control

Whereas *speed control* manipulates the normalized motor voltage $$u_{n} \$$(element pedals of ROS topic /mad/car0/carinputs in Chapter ‎2), *path following control* manipulates the normalized, lower and upper limited steering angle $$\delta_{n} = \frac{\delta}{\delta_{\max}} \in \lbrack - 1 ; 1 \rbrack$$ (element steering of ROS topic /mad/car0/carinputs).

The process variables of the path following control are:

-   the position of the rear axle center of the car in Cartesian coordinates:

    $$
    \mathbf{s} ( t ) = \left( s_{1} ( t ) , s_{2} ( t ) , 0 \right)^{T}
    $$

-   the yaw angle of the car $$\psi ( t )$$

The path following controller is a controller with two degrees of freedom:

-   nonlinear state-space feedback controller $$\Sigma_{F B}$$
-   nonlinear feedforward controller $$\Sigma_{F F}$$

The reference signals are generated by

-   reference signal generator $$\Sigma_{r e f}$$

from the given reference track and the current car position.

The signal flow diagram on system level is depicted in the following figure, where $$\Sigma$$ is the nonlinear vehicle dynamics of Sections ‎5.4 and ‎5.5.

Figure ‎9.1: Signal flow diagram of path following control.

The reference path generator $$\Sigma_{r e f}$$ generates the following reference signals:

-   reference position $$\mathbf{s}^{\mathbf{*}}$$in Cartesian coordinates
-   reference yaw angle $$\psi^{*}$$
-   reference curvature $$\kappa^{*}$$

## Reference Signal Generator

In order to distinguish the reference path from the car state, the reference path is denoted by the superscript \*, as in the beginning of Chapter ‎7:

|   | $$ \mathbf{s}^{\mathbf{*}} \left( x^{*} \right) = \begin{pmatrix}s_{1}^{*} \left( x^{*} \right) \\ s_{2}^{*} \left( x^{*} \right) \\ 0\end{pmatrix} $$ |   |
|---|--------------------------------------------------------------------------------------------------------------------------------------------------------|---|

All derived quantities of the reference path are also denoted by a $$*$$ superscript,

-   such as the reference yaw angle $$\psi^{*} ( x^{*} )$$ which is needed for feedback control

Arclength $$x^{*}$$ denotes the arclength of the point on the reference path which is closest to the current rear axle center of the vehicle.

For the nonlinear feedforward controller

-   the curvature $$\kappa^{*} ( \widehat{x}$$) of the reference path

is needed. For this feedforward control, the reference signal generator looks into the future, in order to compensate the dead time $$T_{t} \$$of the vehicle. Arc length $$\widehat{x}$$ denotes a point on the reference path that is located in front of the closest point $$\mathbf{s}^{\mathbf{*}} \left( x^{*} \right) .$$ Arc length $$\widehat{x}$$ is computed by

|   | $$ \widehat{x} = x^{*} + v^{*} T_{t} $$ |   |
|---|-----------------------------------------|---|

where $$v^{*}$$ is the reference speed for the speed controller in Chapter ‎6 and $$T_{t}$$ is the overall dead time of the vehicle dynamics.

Nearest Point on Reference Path

The nearest point on the given cubic spline of the reference path to the current car position is computed by an approximation in a two-step procedure:

1.  The nearest two sampling points $$x_{k}^{*}$$ and $$x_{k + 1}^{*}$$ are determined.
2.  The nearest point $$x^{*}$$ between these two sampling points is approximated by linear interpolation.
3.  The distance of the car position to the cubic spline is computed by a nonlinear equation. By applying a numerical, iterative gradient-based method, the resolution of arc length $$x^{*}$$ is further improved.

This nearest point is the reference point $$\mathbf{s}^{\mathbf{*}} \left( x^{*} \right) \$$for path following control. By comparing the current car position to this reference point, the control deviation is computed in Section ‎9.3.

Figure ‎9.2: Closest point with arclength $$\mathbf{x}^{\mathbf{*}}$$ on reference path. The point with arclength $$\widehat{\mathbf{x}}$$ is applied to compute curvature $$\mathbf{\kappa}^{\mathbf{*}}$$ for the feedforward controller.

Reference Signal of Yaw Angle

As soon as we know the arc length $$x^{*}$$ of the nearest point on the spline, we can readily calculate the reference yaw angle by evaluating the definition (‎8.2) of the yaw angle:

|   | $$ \  \psi^{*} \left( x^{*} \right) = a t a n 2 \left( s_{2}^{*^{'}} \left( x^{*} \right) , s_{1}^{*^{'}} ( x^{*} ) \right) $$ | (‎9.1) |
|---|--------------------------------------------------------------------------------------------------------------------------------|-------|

Curvature of Reference Path

The curvature of the reference path required by the feedforward controller is readily given by equation (‎8.3):

|   | $$ \  \kappa^{*} ( \widehat{x} ) = \sqrt{{s_{1}^{*^{' '}} ( \widehat{x} )}^{2} + {s_{2}^{*^{' '}} ( \widehat{x} )}^{2}} $$ | (‎9.2) |
|---|----------------------------------------------------------------------------------------------------------------------------|-------|

To distinguish between left and right turns, the third element of the binormal vector (‎8.4) is evaluated

|   | $$ \mathbf{t}^{\mathbf{*}} \mathbf{\times} \mathbf{n}^{\mathbf{*}} = \begin{pmatrix}0 \\ 0 \\ s_{1}^{* '} s_{2}^{* ' '} - s_{1}^{* ' '} s_{2}^{* '}\end{pmatrix} $$ | (‎9.3) |
|---|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------|

The third element of this vector determines the sign of the curvature:

|   | $$ \  \kappa^{*} ( \widehat{x} ) = s i g n ( s_{1}^{* '} s_{2}^{* ' '} - s_{1}^{* ' '} s_{2}^{* '} ) \sqrt{s_{1}^{*^{' '}}^{2} + s_{2}^{*^{' '}}^{2}} $$ | (‎9.4) |
|---|----------------------------------------------------------------------------------------------------------------------------------------------------------|-------|

## MATLAB MODBAS CAR Library

The MODBAS CAR library (mbc) provides the following MATLAB function to compute the approximated nearest point $$x^{*}$$ and the reference signals $$\mathbf{s}^{\mathbf{*}} \left( x^{*} \right)$$, $$\psi^{*} ( x^{*} )$$ and $$\kappa^{*} ( \widehat{x} )$$:

-   mbc_spline_get_reference computes $$\mathbf{s}^{\mathbf{*}} \left( x^{*} \right)$$, $$\psi^{*} ( x^{*} )$$ and $$\kappa^{*} ( \widehat{x} )$$ for a given reference spline path and the current car position $$\mathbf{s} ( t )$$

Internally, mbc_spline_get_reference invokes the following two MATLAB functions:

-   mbc_spline_get_nearest computes $$x^{*}$$ according to
-   mbc_ppval is an embedded version of ppval

All three functions are written in Embedded MATLAB and thus can be called from Simulink by MATLAB function blocks. Further, the Simulink Embedded Coder can generate embedded C code for the microcontroller.

Call help on the individual MATLAB functions for more information:

help mbc_spline_get_reference

## Control Deviation Dynamics

Before we design the nonlinear feedback and the feedforward controllers of the signal flow diagram in Figure ‎9.1, we derive the dynamics of the control deviation.

The overall control deviation is:

-   tangential distance $$s_{c 1 e}$$ of the rear axle to the reference point
-   orthogonal distance $$s_{c 2 e}$$ of the rear axle to the reference point
-   deviation $$\psi_{e} = \psi - \psi^{*}$$ of the yaw angle to the orientation angle of the reference path

In vector form, the control deviation $$\mathbf{x}_{\mathbf{e}}$$ is defined as:

|   | $$ \mathbf{x}_{\mathbf{e}} = \begin{pmatrix}s_{c 1 e} \\ s_{c 2 e} \\ \psi_{e}\end{pmatrix} $$ |   |
|---|------------------------------------------------------------------------------------------------|---|

The Distance to the Reference Path

The orthogonal distance of the car position to the nearest point on the reference path is determined by a projection of the distance vector to the normal of the reference path (see Figure ‎9.2):

|   | $$ s_{c 2 e} ( t ) = \underset{\mathbf{s}_{\mathbf{e}}}{\overset{{\lbrack \mathbf{s} ( t ) - \mathbf{s}^{\mathbf{*}} \left( x^{*} \right) \rbrack}^{T}}{︷}} \cdot \mathbf{n}^{\mathbf{*}} \left( x^{*} \right) = \left\lbrack s_{2} ( t ) - s_{2}^{*} \left( x^{*} \right) \right\rbrack \cos {\psi^{*} \left( x^{*} \right)} - \lbrack s_{1} ( t ) - s_{1}^{*} ( x^{*} ) \rbrack \sin {\psi^{*} ( x^{*} )} $$ | (‎9.5) |
|---|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------|

The normal vector of the reference path is determined from the reference orientation angle:

|   | $$ \mathbf{n}^{\mathbf{*}} \left( x^{*} \right) = \begin{pmatrix}s_{1}^{* ' '} \\ s_{2}^{* ' '}\end{pmatrix} = \begin{pmatrix}- \sin {\  \psi^{*}} \\ \cos \psi^{*}\end{pmatrix} $$ |   |
|---|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

since the tangential vector is

|   | $$ \mathbf{t}^{\mathbf{*}} \left( x^{*} \right) = \begin{pmatrix}s_{1}^{* '} \\ s_{2}^{* '}\end{pmatrix} = \begin{pmatrix}\cos {\  \psi^{*}} \\ \sin \psi^{*}\end{pmatrix} $$ |   |
|---|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

The deviation $$s_{c 1 e}$$ tangential to the reference path equals to 0, since

|   | $$ s_{c 1 e} ( t ) = \underset{\mathbf{s}_{\mathbf{e}}}{\overset{{\lbrack \mathbf{s} ( t ) - \mathbf{s}^{\mathbf{*}} \left( x^{*} \right) \rbrack}^{T}}{︷}} \cdot \mathbf{t}^{\mathbf{*}} \left( x^{*} \right) = 0 $$ | (‎9.6) |
|---|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------|

The Control Deviation of the Yaw Angle

The control deviation of the yaw angle is readily given by

|   | $$ \psi_{e} ( t ) = \psi ( t ) - \  \psi^{*} \left( x^{*} \right) $$ | (‎9.7) |
|---|----------------------------------------------------------------------|-------|

Nonlinear Error Dynamics

Now, the nonlinear dynamics of the control deviation vector $${\mathbf{x}_{\mathbf{e}} = \left( s_{c 1 e} , s_{c 2 e} , \psi_{e} \right)}^{T}$$ is derived as a prerequisite for the subsequent controller design.

First, the control deviation vector is defined from equations (‎9.5), (‎9.6) and (‎9.7):

|   | $$ \begin{pmatrix}s_{c 1 e} \\ s_{c 2 e} \\ \psi_{e}\end{pmatrix} = \begin{pmatrix}\mathbf{s}_{\mathbf{e}}^{\mathbf{T}} \mathbf{\cdot} \mathbf{t}^{\mathbf{*}} \\ \mathbf{s}_{\mathbf{e}}^{\mathbf{T}} \mathbf{\cdot} \mathbf{n}^{\mathbf{*}} \\ \psi ( t ) - \  \psi^{*} \left( x^{*} \right)\end{pmatrix} $$ |   |
|---|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

Derivation w.r.t time $$t$$ yields the differential equations of the error dynamics:

|   | $$ \begin{pmatrix}\dot{s}_{c 1 e} \\ \dot{s}_{c 2 e} \\ \dot{\psi}_{e}\end{pmatrix} = \begin{pmatrix}\dot{\mathbf{s}}_{\mathbf{e}}^{\mathbf{T}} \mathbf{\cdot} \mathbf{t}^{\mathbf{*}} \mathbf{+} \mathbf{s}_{\mathbf{e}}^{\mathbf{T}} \mathbf{\cdot} \dot{\mathbf{t}}^{\mathbf{*}} \\ \dot{\mathbf{s}}_{\mathbf{e}}^{\mathbf{T}} \mathbf{\cdot} \mathbf{n}^{\mathbf{*}} \mathbf{+} \mathbf{s}_{\mathbf{e}}^{\mathbf{T}} \mathbf{\cdot} \dot{\mathbf{n}}^{\mathbf{*}} \\ \dot{\psi} - \dot{\psi}^{*}\end{pmatrix} $$ | (‎9.8) |
|---|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------|

The individual derivatives are

|   | $$ \dot{\mathbf{s}}_{\mathbf{e}} = \dot{\mathbf{s}} \mathbf{-} \dot{\mathbf{s}}^{\mathbf{*}} $$                                                                              | (‎9.9)  |
|---|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
|   | $$ \dot{\mathbf{t}}^{\mathbf{*}} \mathbf{=} \dot{\psi}^{*} \begin{pmatrix}- \sin {\  \psi^{*}} \\ \cos \psi^{*}\end{pmatrix} = \dot{\psi}^{*} \mathbf{n}^{\mathbf{*}} $$     | (‎9.10) |
|   | $$ \dot{\mathbf{n}}^{\mathbf{*}} \mathbf{=} \dot{\psi}^{*} \begin{pmatrix}- \cos {\  \psi^{*}} \\ - \sin \psi^{*}\end{pmatrix} = - \dot{\psi}^{*} \mathbf{t}^{\mathbf{*}} $$ | (‎9.11) |

Inserting these equations in (‎9.7) yields:

|   | $$ \begin{pmatrix}\dot{s}_{c 1 e} \\ \dot{s}_{c 2 e} \\ \dot{\psi}_{e}\end{pmatrix} = \begin{pmatrix}\left( \dot{\mathbf{s}} \mathbf{-} \dot{\mathbf{s}}^{\mathbf{*}} \right)^{\mathbf{T}} \mathbf{\cdot} \mathbf{t}^{\mathbf{*}} \mathbf{+} \dot{\psi}^{*} \mathbf{s}_{\mathbf{e}}^{\mathbf{T}} \mathbf{\cdot} \mathbf{n}^{\mathbf{*}} \\ \left( \dot{\mathbf{s}} \mathbf{-} \dot{\mathbf{s}}^{\mathbf{*}} \right)^{\mathbf{T}} \mathbf{\cdot} \mathbf{n}^{\mathbf{*}} - \dot{\psi}^{*} \mathbf{s}_{\mathbf{e}}^{\mathbf{T}} \mathbf{\cdot} \mathbf{t}^{\mathbf{*}} \\ \dot{\psi} - \dot{\psi}^{*}\end{pmatrix} = \begin{pmatrix}\left( \dot{\mathbf{s}} \mathbf{-} \dot{\mathbf{s}}^{\mathbf{*}} \right)^{\mathbf{T}} \mathbf{\cdot} \mathbf{t}^{\mathbf{*}} \mathbf{+} \dot{\psi}^{*} s_{c 2 e} \\ \left( \dot{\mathbf{s}} \mathbf{-} \dot{\mathbf{s}}^{\mathbf{*}} \right)^{\mathbf{T}} \mathbf{\cdot} \mathbf{n}^{\mathbf{*}} - \dot{\psi}^{*} s_{c 1 e} \\ \dot{\psi} - \dot{\psi}^{*}\end{pmatrix} $$ | (‎9.12) |
|---|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|

From the bicycle model (‎5.2) in Section ‎5.2 we know the dynamics of the vehicle:

|   | $$ \begin{pmatrix}\dot{s}_{1} \\ \dot{s}_{2} \\ \dot{\psi}\end{pmatrix} = \begin{pmatrix}v \cos \psi \\ v \sin \psi \\ \frac{v}{l} \tan \delta\end{pmatrix} $$ | (‎9.13) |
|---|----------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|

If we assume the vehicle to be moving on the reference path, we obtain the dynamics of this reference movement as:

|   | $$ \begin{pmatrix}\dot{s}_{1}^{*} \\ \dot{s}_{2}^{*} \\ \dot{\psi}^{*}\end{pmatrix} = \begin{pmatrix}v^{*} \cos \psi^{*} \\ v^{*} \sin \psi^{*} \\ \frac{v^{*}}{l} {\tan \delta}^{*}\end{pmatrix} $$ | (‎9.14) |
|---|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|

Inserting (‎9.13) and (‎9.14) in (‎9.12) yields the nonlinear error dynamics:

|   | $$ \dot{s}_{c 1 e} = v \begin{pmatrix}\cos \psi \\ \sin \psi\end{pmatrix}^{T} \cdot \  \begin{pmatrix}\cos {\  \psi^{*}} \\ \sin \psi^{*}\end{pmatrix} - v^{*} \begin{pmatrix}\cos \psi^{*} \\ \sin \psi^{*}\end{pmatrix}^{T} \cdot \  \begin{pmatrix}\cos {\  \psi^{*}} \\ \sin \psi^{*}\end{pmatrix} + \dot{\psi}^{*} s_{c 2 e} = v \cos \left( \psi - \psi^{*} \right) - v^{*} + s_{c 2 e} \frac{v^{*}}{l} {\tan \delta}^{*} = v \cos \psi_{e} - v^{*} + s_{c 2 e} \frac{v^{*}}{l} {\tan \delta}^{*} $$ |   |
|---|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|
|   | $$ \dot{s}_{c 2 e} = v \begin{pmatrix}\cos \psi \\ \sin \psi\end{pmatrix}^{T} \cdot \  \begin{pmatrix}{- s i n \ } {\  \psi^{*}} \\ \cos \psi^{*}\end{pmatrix} - v^{*} \begin{pmatrix}\cos \psi^{*} \\ \sin \psi^{*}\end{pmatrix}^{T} \cdot \  \begin{pmatrix}{- \sin} {\  \psi^{*}} \\ \cos \psi^{*}\end{pmatrix} - \dot{\psi}^{*} s_{c 1 e} = v \sin \left( \psi - \psi^{*} \right) - s_{c 1 e} \frac{v^{*}}{l} {\tan \delta}^{*} = v \sin \psi_{e} - s_{c 1 e} \frac{v^{*}}{l} {\tan \delta}^{*} $$     |   |

and in vector form:

|   | $$ \begin{pmatrix}\dot{s}_{c 1 e} \\ \dot{s}_{c 2 e} \\ \dot{\psi}_{e}\end{pmatrix} = \begin{pmatrix}v \cos \psi_{e} - v^{*} + s_{c 2 e} \frac{v^{*}}{l} {\tan \delta}^{*} \\ v \sin \psi_{e} - s_{c 1 e} \frac{v^{*}}{l} {\tan \delta}^{*} \\ \frac{v}{l} \tan \delta - \frac{v^{*}}{l} {\tan \delta}^{*}\end{pmatrix} $$ |   |
|---|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

Because of (‎9.6) this is simplified to:

|   | $$ \underset{\dot{\mathbf{x}}_{\mathbf{e}}}{\overset{\begin{pmatrix}\dot{s}_{c 1 e} \\ \dot{s}_{c 2 e} \\ \dot{\psi}_{e}\end{pmatrix}}{︷}} = \underset{\mathbf{f}_{\mathbf{e}} \left( s_{c 1 e} , s_{c 2 e} , \  \psi_{e} , \  v , v^{*} , \delta , \delta^{*} \right)}{\overset{\begin{pmatrix}v \cos \psi_{e} - v^{*} + s_{c 2 e} \frac{v^{*}}{l} {\tan \delta}^{*} \\ v \sin \psi_{e} \\ \frac{v}{l} \tan \delta - \frac{v^{*}}{l} {\tan \delta}^{*}\end{pmatrix}}{︷}} $$ | (‎9.15) |
|---|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|

Linear Error Dynamics

Linearization of (‎9.15) about the operation point

-   $$
    \overline{s}_{c 1 e} = 0
    $$
-   $$
    \overline{s}_{c 2 e} = 0
    $$
-   $$
    \overline{\psi}_{e} = 0
    $$
-   $$\overline{v} = v^{*}$$ (reference speed)
-   $$\overline{\delta} = \delta^{*}$$ with assumption $$\delta^{*} = 0$$

yields the linearized error dynamics

|   | $$ \begin{pmatrix}\dot{s}_{c 1 e} \\ \dot{s}_{c 2 e} \\ \dot{\psi}_{e}\end{pmatrix} = \begin{pmatrix}0 & \frac{\overline{v}}{l} \tan \delta^{*} & - \overline{v} \sin \overline{\psi}_{e} \\ 0 & 0 & \overline{v} \cos \overline{\psi}_{e} \\ 0 & 0 & 0\end{pmatrix} \cdot \begin{pmatrix}s_{c 1 e} \\ s_{c 2 e} \\ \psi_{e}\end{pmatrix} + \begin{pmatrix}\cos \overline{\psi}_{e} & 0 \\ \sin \overline{\psi}_{e} & 0 \\ \frac{1}{l} \tan \overline{\delta} & \frac{\overline{v}}{l} \frac{1}{\cos^{2} \overline{\delta}}\end{pmatrix} \cdot \begin{pmatrix}v_{e} \\ \delta_{e}\end{pmatrix} $$ |   |
|---|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|
|   | $$ \begin{pmatrix}\dot{s}_{c 1 e} \\ \dot{s}_{c 2 e} \\ \dot{\psi}_{e}\end{pmatrix} = \begin{pmatrix}0 & \frac{v^{*}}{l} \tan \delta^{*} & 0 \\ 0 & 0 & v^{*} \\ 0 & 0 & 0\end{pmatrix} \cdot \begin{pmatrix}s_{c 1 e} \\ s_{c 2 e} \\ \psi_{e}\end{pmatrix} + \begin{pmatrix}1 & 0 \\ 0 & 0 \\ 0 & \frac{v^{*}}{l \  \cos^{2} \delta^{*} \ }\end{pmatrix} \cdot \begin{pmatrix}v_{e} \\ \delta_{e}\end{pmatrix} $$                                                                                                                                                                               |   |

In the case of path following control, the longitudinal control deviation $$s_{c 1 e}$$ equals to zero due to the search for the closest point on the reference path. Thus, the error dynamics is reduced to a dynamics of 2nd degree:

|   | $$ \underset{\dot{\mathbf{x}}_{\mathbf{e}}}{\overset{\begin{pmatrix}\dot{s}_{c 2 e} \\ \dot{\psi}_{e}\end{pmatrix}}{︷}} = \underset{\mathbf{A}_{\mathbf{e}} \left( v^{*} \right)}{\overset{\begin{pmatrix}0 & v^{*} \\ 0 & 0\end{pmatrix}}{︷}} \cdot \underset{\mathbf{x}_{\mathbf{e}}}{\overset{\begin{pmatrix}s_{c 2 e} \\ \psi_{e}\end{pmatrix}}{︷}} + \underset{\mathbf{b}_{\mathbf{e}} \left( v^{*} \right)}{\overset{\begin{pmatrix}0 \\ \frac{v^{*}}{l}\end{pmatrix}}{︷}} \underset{u_{e}}{\overset{\delta_{e}}{︷}} $$ |  (‎9.16) |
|---|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|

## Feedback Controller

The linearized dynamics (‎9.16) of the control deviations $$s_{c 2 e}$$ and $$\psi_{e}$$ is controlled by a nonlinear feedback controller $$\Sigma_{F B}$$. The output of this feedback controller is the steering angle $$\delta_{e}$$ which will be added to the output $$\delta^{*}$$ of feedforward controller in Section ‎9.6.

So, the total steering angle will be

|   | $$ \delta = \delta^{*} + \delta_{e} $$ |   |
|---|----------------------------------------|---|

Control Law

The control law of a *state-space controller* in case of a scalar control signal is formulated as:

|   | $$ u_{e} = \delta_{e} = - \mathbf{k}^{\mathbf{T}} \left( v^{*} \right) \cdot \mathbf{x}_{\mathbf{e}} $$ | (‎9.17) |
|---|---------------------------------------------------------------------------------------------------------|--------|

For a linear state-space controller the *controller matrix* $$\mathbf{k}^{\mathbf{T}}$$ is constant. In case of path following control, however, a nonlinear controller with matrix $$\mathbf{k}^{\mathbf{T}} \left( v^{*} \right) = \left( k_{1} \left( v^{*} \right) \  \  \  k_{2} \left( v^{*} \right) \right)$$ is applied whose elements are dependent of the reference speed $$v^{*}$$. By a suitable approach for $$\mathbf{k}^{\mathbf{T}}$$ a linear closed-loop dynamics will be obtained.

Inserting control law (‎9.17) in the control-deviation dynamics (‎9.16) yields closed-loop dynamics:

|   | $$ \dot{\mathbf{x}}_{\mathbf{e}} \mathbf{=} \underset{\mathbf{A}_{\mathbf{w}}}{\overset{\left( \mathbf{A}_{\mathbf{e}} \left( v^{*} \right) \mathbf{-} \mathbf{b}_{\mathbf{e}} \left( v^{*} \right) \mathbf{\cdot} \mathbf{k}^{\mathbf{T}} \left( v^{*} \right) \right)}{︷}} \mathbf{\cdot} \mathbf{x}_{\mathbf{e}} $$  |   |
|---|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

By specifying the eigenvalues of the closed-loop system matrix $$\mathbf{A}_{\mathbf{w}}$$ the coefficients of controller matrix $$\mathbf{k}^{\mathbf{T}}$$ are determined. For this, the *characteristic equation* of the closed-loop dynamics is formulated:

|   | $$ \left| \lambda \mathbf{I} \mathbf{-} \mathbf{A}_{\mathbf{w}} \right| = 0 $$ |   |
|---|--------------------------------------------------------------------------------|---|

This characteristic equation defines the *eigenvalues* $$\lambda_{i}$$ of the closed loop. In our case, the following equations hold:

|   | $$ \mathbf{A}_{\mathbf{w}} = \mathbf{A}_{\mathbf{e}} \left( v^{*} \right) \mathbf{-} \mathbf{b}_{\mathbf{e}} \left( v^{*} \right) \mathbf{\cdot} \mathbf{k}^{\mathbf{T}} \left( v^{*} \right) \mathbf{=} \begin{pmatrix}0 & v^{*} \\ 0 & 0\end{pmatrix} - \begin{pmatrix}0 \\ \frac{v^{*}}{l}\end{pmatrix} \cdot \begin{pmatrix}k_{1} \left( v^{*} \right) & k_{2} ( v^{*} )\end{pmatrix} $$ |   |
|---|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|
|   | $$ \mathbf{A}_{\mathbf{w}} \mathbf{=} \begin{pmatrix}0 & v^{*} \\ - \frac{v^{*}}{l} k_{1} ( v^{*} ) & - \frac{v^{*}}{l} k_{2} ( v^{*} )\end{pmatrix} $$                                                                                                                                                                                                                                      |   |
|   | $$ \left| \lambda \mathbf{I} \mathbf{-} \mathbf{A}_{\mathbf{w}} \right| = \left| \begin{matrix}\lambda & - v^{*} \\ \frac{v^{*}}{l} k_{1} ( v^{*} ) & \lambda + \frac{v^{*}}{l} k_{2} ( v^{*} )\end{matrix} \right| = \lambda^{2} + \frac{v^{*}}{l} k_{2} \left( v^{*} \right) \lambda + \frac{v^{* 2}}{l} k_{1} \left( v^{*} \right) = 0 $$                                                 |   |

By applying the following nonlinear approach for the controller matrix with constant coefficients $$k_{1}^{'} , k_{2}^{'}$$

|   | $$ \mathbf{k}^{\mathbf{T}} \left( v^{*} \right) = \left( \begin{matrix}k_{1} \left( v^{*} \right) & k_{2} ( v^{*}\end{matrix} ) \right) = \begin{pmatrix}\frac{k_{1}^{'}}{v^{* 2}} & \frac{k_{2}^{'}}{v^{*}}\end{pmatrix} $$ | (‎9.18) |
|---|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|

the characteristic equation is reduced to

|   | $$ \left| \lambda \mathbf{I} \mathbf{-} \mathbf{A}_{\mathbf{w}} \right| = \lambda^{2} + \frac{k_{2}^{'}}{l} \lambda + \frac{k_{1}^{'}}{l} = 0 $$ | (‎9.19) |
|---|--------------------------------------------------------------------------------------------------------------------------------------------------|--------|

By specifying the eigenvalues $$\lambda_{1} , \lambda_{2}$$ the characteristic equation is further formulated as:

|   | $$ \left( \lambda - \lambda_{1} \right) \left( \lambda - \lambda_{2} \right) = \lambda^{2} - \left( \lambda_{1} + \lambda_{2} \right) \lambda + \lambda_{1} \lambda_{2} = 0 $$ | (‎9.20) |
|---|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|

With given eigenvalues $$y i \$$, the controller parameters of $$\Sigma_{F B}$$ are determined by comparing the coefficients of (‎9.19) and (‎9.20):

|   | $$ k_{2}^{'} = - l \left( \lambda_{1} + \lambda_{2} \right) = \frac{2 l}{T_{w}} $$ | (‎9.21) |
|---|------------------------------------------------------------------------------------|--------|
|   | $$ k_{1}^{'} = l \lambda_{1} \lambda_{2} = \frac{l}{T_{w}^{2}} $$                  | (‎9.22) |

According to (‎9.18) the controller matrix of $$\Sigma_{F B}$$ is then given by:

|   | $$ \mathbf{k}^{\mathbf{T}} \left( v^{*} \right) = \begin{pmatrix}\frac{k_{1}^{'}}{v^{* 2}} & \frac{k_{2}^{'}}{v^{*}}\end{pmatrix} = \begin{pmatrix}\frac{l}{T_{w}^{2} v^{* 2}} & \frac{2 l}{T_{w} v^{*}}\end{pmatrix} $$ |   |
|---|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

Considering (‎9.17) yields the control signal in terms of the control deviations:

|   | $$ u_{e} = \delta_{e} = - \mathbf{k}^{\mathbf{T}} \left( v^{*} \right) \cdot \mathbf{x}_{\mathbf{e}} = - \begin{pmatrix}\frac{l}{T_{w}^{2} v^{* 2}} & \frac{2 l}{T_{w} v^{*}}\end{pmatrix} \cdot \begin{pmatrix}s_{c 2 e} \\ \psi_{e}\end{pmatrix} = - \frac{l}{T_{w}^{2} v^{* 2}} \  s_{c 2 e} - \frac{2 l}{T_{w} v^{*}} \psi_{e} $$ |   |
|---|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

It is important to note that $$\mathbf{k}^{\mathbf{T}}$$ becomes singular at a reference speed $$v^{*} = 0 \frac{m}{s}$$. Thus, $$v^{*}$$ must be limited when implementing the control law.

## Controllability

The controller matrix $$\mathbf{k}^{\mathbf{T}}$$ is singular at $$v^{*} = 0 \frac{m}{s}$$, because the nonlinear vehicle dynamics $$\Sigma$$ (‎9.13) is *locally non-controllable* at speed $$v = 0 \frac{m}{s}$$. The *local total controllability* of a nonlinear system is defined by its linearization at an operating point.

The state-space model $$\Sigma$$ (‎9.13) of the Reads-Shepp car is given as:

|   | $$ \Sigma : \  \  \  \  \  \underset{\dot{\mathbf{x}}}{\overset{\begin{pmatrix}\dot{s}_{1} \\ \dot{s}_{2} \\ \dot{\psi}\end{pmatrix}}{︷}} = \underset{\mathbf{f} \left( \mathbf{x , u} \right)}{\overset{\begin{pmatrix}v \cos \psi \\ v \sin \psi \\ \frac{v}{l} \tan \delta\end{pmatrix}}{︷}} \  \  \  \  \  ; \  \  t > 0 $$ |   |
|---|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|
|   | $$ \mathbf{x} ( 0 ) = \mathbf{x}_{\mathbf{0}} $$                                                                                                                                                                                                                                                                                  |   |

with the state vector $$\mathbf{x} = \begin{pmatrix}s_{1} & s_{2} & \psi\end{pmatrix}^{T}$$ and the input vector $$\mathbf{u} = \begin{pmatrix}v & \delta\end{pmatrix}^{T}$$.

Linearization at a given, arbitrary operating point in the state space

$$
\overline{\mathbf{x}} = \left( \begin{matrix}\overline{s}_{1} & \overline{s}_{2} & \overline{\psi}\end{matrix} \  \  \  \right)^{T} \  , \  \  \  \  \overline{\mathbf{u}} = \left( \begin{matrix}\overline{v} & \overline{\delta}\end{matrix} \  \right)^{T}
$$

yields the linear system $$\Delta \Sigma :$$

|   | $$ \Delta \Sigma : \underset{\mathbf{\Delta} \dot{\mathbf{x}}}{\  \  \overset{\begin{pmatrix}\Delta \dot{s}_{1} \\ \Delta \dot{s}_{2} \\ \Delta \dot{\psi}\end{pmatrix}}{︷}} = \underset{\mathbf{A}}{\overset{\begin{pmatrix}0 & 0 & - \overline{v} \sin \overline{\psi} \\ 0 & 0 & \overline{v} \cos \overline{\psi} \\ 0 & 0 & 0\end{pmatrix}}{︷}} \cdot \underset{\mathbf{\Delta} \mathbf{x}}{\overset{\begin{pmatrix}\Delta s_{1} \\ \Delta s_{2} \\ \Delta \psi\end{pmatrix}}{︷}} + \underset{\mathbf{B}}{\overset{\begin{pmatrix}\cos \overline{\psi} & 0 \\ \sin \overline{\psi} & 0 \\ 0 & \frac{\overline{v}}{l} \left( 1 + \tan^{2} \overline{\delta} \right)\end{pmatrix}}{︷}} \cdot \underset{\mathbf{\Delta} \mathbf{u}}{\overset{\begin{pmatrix}\Delta v \\ \Delta \delta\end{pmatrix}}{︷}} \  \  ; \  \  t > 0 $$  |   |
|---|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|
|   | $$ \mathbf{\Delta} \mathbf{x} ( 0 ) = \mathbf{x}_{\mathbf{0}} \mathbf{-} \overline{\mathbf{x}} $$                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |   |

with *system matrix* $$\mathbf{A} \mathbf{=} \left. \frac{\mathbf{\partial f}}{\mathbf{\partial} \mathbf{x}^{\mathbf{T}}} \right|_{\overline{\mathbf{x}} \mathbf{, \ } \overline{\mathbf{u}}}$$ and *input matrix* $$\mathbf{B} \mathbf{=} \left. \frac{\mathbf{\partial f}}{\mathbf{\partial} \mathbf{u}^{\mathbf{T}}} \right|_{\overline{\mathbf{x}} \mathbf{, \ } \overline{\mathbf{u}}} .$$

Quote from (Schulz, Regelungstechnik 2, 2008) p. 51: „System $$\Delta \Sigma$$ is *total state-controllable* if for any initial time $$t_{0}$$ an arbitrary initial state $$\mathbf{\Delta} \mathbf{x} ( t_{0} )$$ can be transferred to an arbitrary final state $$\mathbf{\Delta} \mathbf{x} ( t_{f} )$$ by an unlimited inputs signal $$\mathbf{\Delta} \mathbf{u} ( t )$$ in final time $$t_{f} > t_{0}$$.“

***

This is the case for the linear system $$\Delta \Sigma$$ above only for $$\overline{v} \neq 0 \frac{m}{s}$$.

This proven by the *controllability matrix* $$\mathbf{Q}_{\mathbf{S}}$$**.**

Quote from (Schulz, Regelungstechnik 2, 2008) p. 53: „System $$\Delta \Sigma$$ of degree $$n \$$is *total state controllable* if matrix $$\mathbf{Q}_{\mathbf{S}}$$ has rank $$n$$.

***

$$
\mathbf{Q}_{\mathbf{S}} \mathbf{=} \begin{pmatrix}\mathbf{B} & \mathbf{A} \mathbf{\cdot} \mathbf{B} & \mathbf{A}^{\mathbf{2}} \mathbf{\cdot} \mathbf{B} & \mathbf{\ldots} & \mathbf{A}^{\mathbf{n} \mathbf{-} \mathbf{1}} \mathbf{\cdot} \mathbf{B}\end{pmatrix}
$$

***

The linear system $$\Delta \Sigma$$ above has a degree of $$n = 3 \$$and the controllability matrix is computed as:

|   | $$ \mathbf{Q}_{\mathbf{S}} \mathbf{=} \begin{pmatrix}\cos \overline{\psi} & 0 & 0 & - \frac{\overline{v}^{2}}{l} \sin \overline{\psi} \left( 1 + \tan^{2} \overline{\delta} \right) & 0 & 0 \\ \sin \overline{\psi} & 0 & 0 & \frac{\overline{v}^{2}}{l} \cos \overline{\psi} \left( 1 + \tan^{2} \overline{\delta} \right) & 0 & 0 \\ 0 & \frac{\overline{v}}{l} \left( 1 + \tan^{2} \overline{\delta} \right) & 0 & 0 & 0 & 0\end{pmatrix} $$ |   |
|---|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

This matrix $$\mathbf{Q}_{\mathbf{S}}$$ has rank 2 for $$\overline{v} = 0 \frac{m}{s}$$ and rank 3 for $$\overline{v} \neq 0 \frac{m}{s}$$. Thus, $$\Delta \Sigma$$ is *total state controllable* only if $$v \neq 0 \frac{m}{s}$$ and system $$\Sigma$$ is *locally non-controllable* for $$v = 0 \frac{m}{s}$$.

System $$\Sigma$$ is *globally controllable*, although it is *locally non-controllable*, since every front-steering vehicle can be transferred from an arbitrary initial point $$x_{0}$$ (including initial yaw angle $$\psi_{0} )$$ to an arbitrary final point $$x ( t_{f} )$$ (including final yaw angle $$\psi_{f} )$$ by an appropriate choice of the control signals $$v ( t )$$ und $$\delta ( t )$$. This fact can be proven by nonlinear control theory using Lie brackets (Sussmann & Guoqing, 1991).

## Feedforward Controller

In order to improve the overall dynamics of the path following control, the nonlinear feedforward controller $$\Sigma_{F F}$$ generates the steering angle $$\delta^{*}$$ in addition to the steering angle $$\delta_{e}$$ of the feedback state controller $$\Sigma_{F B} .$$

The control law of $$\Sigma_{F F}$$ is obtained from the nonlinear rotation dynamics of the vehicle (see Section ‎5.2):

|   | $$ \dot{\psi} = \frac{v}{l} \tan \delta^{*} $$ |   |
|---|------------------------------------------------|---|

The control law of $$\Sigma_{F F}$$ is the inversion of this equation with given yaw rate $$\dot{\psi} = \dot{\psi}^{*}$$ and speed $$v^{*}$$.

|   | $$ \delta^{*} = {a t a n} \frac{l \dot{\psi}^{*}}{v^{*}} $$ | (‎9.23) |
|---|-------------------------------------------------------------|--------|

The yaw rate is expressed by the local radius and curvature of the reference path:

|   | $$ \dot{\psi}^{*} = \frac{v^{*}}{r^{*}} = v^{*} \kappa^{*} $$ | (‎9.24) |
|---|---------------------------------------------------------------|--------|

Inserting (‎9.24) in (‎9.23) yields the control law of the nonlinear *feed-forward controller* $$\Sigma_{F F}$$:

|   | $$ \delta^{*} = {a t a n} {\lbrack l \cdot \kappa^{*} ( \widehat{x} )} \rbrack $$ |   |
|---|-----------------------------------------------------------------------------------|---|

Note: The steering angle output of the feedforward controller is independent of the vehicle speed.

One further note: $$\Sigma_{F F}$$ only needs the reference curvature $$\kappa^{*} \left( \widehat{x} \right)$$ evaluated at arclength $$\widehat{x} = x^{*} + v^{*} T_{t}$$ as input and the wheelbase $$l$$ as parameter.

## Exercises

Exercise ‎9.1 Simulink Subsystem “Motion Control” for Path Following Control [Simulink]

In this exercise, Simulink subsystem Motion Control of Exercise 6.2 will be extended by path following control. This subsystem

-   already receives bus signal DRIVEMANEUVER to read in the reference speed vmax
-   already receives bus signal CAROUTPUTSEXT to measure speed and the elements s[0], s[1] and psi which correspond to the process variables $$s_{1} , s_{2}$$ and $$\psi$$ of path following control
-   already generates bus signal CARINPUTS to manipulate the car by normalized pedals and steering that correspond to $$u_{n}$$ and $$\delta_{n} = \frac{\delta}{\delta_{\max}}$$

Simulink subsystem Motion Control is to be extended by

-   the new subsystem Path Control that models the path following algorithm in parallel to subsystem Speed Control from Exercise 6.2
-   the new inport spline to read in the reference path on bus signal spline

It is recommended to

-   model individual subsystems for $$\Sigma_{r e f}$$, $$\Sigma_{F B}$$ and $$\Sigma_{F F}$$ as part of subsystem Path Control
-   use the target speed vmax as the speed parameter $$v^{*}$$ of the feedback controller $$\Sigma_{F B}$$ of Section ‎0 rather than the noisy actual vehicle speed
-   normalize the control deviation $$\psi_{e}$$ to the interval $$\psi_{e} \in \lbrack - \pi ; \pi \rbrack$$ in $$\Sigma_{F B}$$, for instance, by calling the MATLAB function mod. Otherwise $$\psi_{e}$$ runs out-of-bounds when the car runs in circles.
-   limit the normalized control steering signal to the interval $$\delta_{n} \in \lbrack - 1 ; 1 \rbrack$$
-   a good choice for the time constant $$T_{w}$$ of the closed-loop dynamics is $$3 0 0 m s$$ for the computation of the controller gains in equations (‎9.21) and (‎9.22)

The following development steps are proposed:

1.  Extend s07_data.m by parameter definitions required for $$\Sigma_{r e f}$$, $$\Sigma_{F B}$$ and $$\Sigma_{F F}$$
2.  Model $$\Sigma_{r e f}$$ as part of Path Control and test it
3.  Extend Path Control by $$\Sigma_{F B}$$ and test it in MiL simulations
4.  Extend Path Control by $$\Sigma_{F F}$$ and test it in MiL simulations
5.  Test the overall path following and speed control in MiL simulations on different tracks at different target speeds vmax by changing the tracks in s07_data.m and by changing the value of the Constant block for vmax
6.  Test the overall path following and speed control on the MAD system in SiL simulations
7.  Test the overall path following and speed control on the real MAD system

Required lab results:

-   s07_sig_template.slx
-   s07_data.m
-   signal-time diagrams of yaw angle $$\psi ( t )$$

Exercise ‎9.2 Differential Flatness of Reeds-Shepp Car [Simulink / C++]

Show that

|   | $$ \Sigma : \  \  \  \  \  \underset{\dot{\mathbf{x}}}{\overset{\begin{pmatrix}\dot{s}_{1} \\ \dot{s}_{2} \\ \dot{\psi}\end{pmatrix}}{︷}} = \underset{\mathbf{f} \left( \mathbf{x , u} \right)}{\overset{\begin{pmatrix}v \cos \psi \\ v \sin \psi \\ \frac{v}{l} \tan \delta\end{pmatrix}}{︷}} \  \  \  \  \  ; \  \  t > 0 $$ |   |
|---|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|
|   | $$ \mathbf{x} ( 0 ) = \mathbf{x}_{\mathbf{0}} $$                                                                                                                                                                                                                                                                                  |   |

with the state vector $$\mathbf{x} = \begin{pmatrix}s_{1} & s_{2} & \psi\end{pmatrix}^{T}$$ and the input vector $$\mathbf{u} = \begin{pmatrix}v & \delta\end{pmatrix}^{T}$$ is a *differentially flat* system and thus is globally controllable.

The *flat output* $$y = \begin{pmatrix}x_{1} & x_{2}\end{pmatrix}^{T}$$ is appropriate.

Apply the atan2 function in your analysis and treat $$u_{1} \rightarrow 0$$ as a special case.

Exercise ‎9.3 Mini-Auto-Drive Team Challenge [Simulink / C++]

In this challenge lab teams are in competition. Lab teams modify and extend the Simulink models or C++ code for speed control and path following control, such that the vehicles achieve minimal lap times.

Modifications and Extensions

The modifications and extension shall be generic and independent of the concrete track. The challenge will be run on a track, which will be provided by the lecturer and is unknown to the lab teams.

In particular, the following Simulink models or C++ code can be modified:

-   Simulink subsystem Motion Contrl as part of Simulink model s07_sig_template.slx
-   or ROS node carctrlnode will all C++ classes as part of ROS package mbmadcar

All other Simulink subsystems or C++ classes must not be changed, in particular, neither Vehicle Dynamics nor ROS node carsimnode with all C++ classes. Otherwise the physical vehicle dynamics would change.

Recommended modifications or extensions for lap time optimization are:

-   improvement of speed and path following controllers
-   reference speed adaptations in dependence of path curvature
-   braking before curves
-   acceleration while exiting curves
-   computation of ideal line for reference path under consideration of longitudinal and lateral acceleration

Placement

The placement of the lab teams is according to the lap time of the second lap which needs to be completed. The vehicle starts from standstill into the first lap.

If the vehicle does not complete the second lap, an alternative placement in dependence of the accident-free runtime from standstill in the first lap is applied. All alternative placements are added at the end of the regular placements.

Penalty Times

All time intervals, in which one single rear wheel of the vehicle exits the driving lane, are added as penalty times to the lap time.

Accident and Termination of Race

When both rear wheels exit the driving lane, the vehicle is considered to have caused a fatal accident and the race of this vehicle is terminated. In this case, the alternative placement described above is considered.

Required Lab Results

-   tar.gz or zip file
    -   with source code or ROS node carctrlnode
    -   Simulink model s07_sig_template.slx with corresponding MATLAB script s07_data.m
-   This tar.gz or zip file must be named differently to the other exercises of this chapter.
1.  Clothoids

Synonyms for clothoid are

-   Euler Curve
-   Cornu Spiral
-   spiro

Left-turned Closing Clothoids

The characteristic of closing clothoids is the linear increase of the local curvature $$\kappa ( x ) = \frac{1}{r ( x )}$$ with the arc length $$x$$.

This section illustrates the derivation of the path definition equations from differential equations for the clothoid path.

The first differential equation is the definition of the curvature linearly depending on the arc length:

|   | $$ \kappa ( x ) = \sqrt{s_{1}^{' '}^{2} + s_{2}^{' '}^{2}} = a \cdot x $$ | (‎9.25) |
|---|---------------------------------------------------------------------------|--------|

The constant $$a > 0$$ is a path parameter of the clothoide.

The second differential equation is the definition of the path speed, which is identical 1 for all paths parameterized in $$x$$:

|   | $$ \sqrt{s_{1}^{' 2} + s_{2}^{' 2}} = 1 $$ | (‎9.26) |
|---|--------------------------------------------|--------|

Squaring both equations leads to:

|   | $$ s_{1}^{' '}^{2} + s_{2}^{' '}^{2} = a^{2} x^{2} $$ | (‎9.27) |
|---|-------------------------------------------------------|--------|
|   | $$ s_{1}^{' 2} + s_{2}^{' 2} = 1 $$                   | (‎9.28) |

In the next step (‎9.28) is differentiated w.r.t. $$x$$, which leads to:

|   | $$ s_{1}^{'} s_{1}^{' '} = - s_{2}^{'} s_{2} ' ' $$ | (‎9.29) |
|---|-----------------------------------------------------|--------|

Squaring yields:

|   | $$ s_{1}^{'}^{2} s_{1}^{' '}^{2} = s_{2}^{'}^{2} s_{2}^{' ' 2} $$ | (‎9.30) |
|---|-------------------------------------------------------------------|--------|

With (‎9.27) and (‎9.28), (‎9.30) is transformed to:

|   | $$ s_{1}^{' ' 2} = a^{2} x^{2} ( 1 - s_{1}^{'}^{2} ) $$ | (‎9.31) |
|---|---------------------------------------------------------|--------|

This is the differential equation for $$s_{1}^{'} ( x )$$. The corresponding equation for $$s_{2}^{'} ( x )$$ can be deduced in the same way:

|   | $$ s_{2}^{' ' 2} = a^{2} x^{2} ( 1 - s_{2}^{'}^{2} ) $$ | (‎9.32) |
|---|---------------------------------------------------------|--------|

The approaches to solving these differential equations are:

|   | $$ s_{1}^{'} ( x ) = \cos \left( \frac{a}{2} x^{2} + \psi_{0} \right) $$ |   |
|---|--------------------------------------------------------------------------|---|
|   | $$ s_{2}^{'} ( x ) = \sin \left( \frac{a}{2} x^{2} + \psi_{0} \right) $$ |   |

This is the definition of the velocity vector at arc length $$x$$ with initial yaw angle $$\psi_{0}$$.

The path definition is obtained by integrating both derivatives:

|   | $$ s_{1} ( x ) = s_{01} + \int_{0}^{x} {\cos \left( \frac{a}{2} \xi^{2} + \psi_{0} \right)} d \xi $$ |   |
|---|------------------------------------------------------------------------------------------------------|---|
|   | $$ s_{2} ( x ) = s_{02} + \int_{0}^{x} {\sin \left( \frac{a}{2} \xi^{2} + \psi_{0} \right)} d \xi $$ |   |

with start point $$\left( s_{10} , s_{20} \right)^{T}$$ and initial yaw angle $$\psi_{0} .$$

Properties of Clothoids

In literature the parameter $$a$$ is widely replaced by the constant $$A$$, where $$A^{2}$$ is defined as the product of the loval curve radius and arc length

|   | $$ A^{2} = r ( x ) x = \frac{x}{\kappa ( x )} = c o n s t $$ |   |
|---|--------------------------------------------------------------|---|

and thus

|   | $$ \kappa ( x ) = \frac{1}{A^{2}} x $$ |   |
|---|----------------------------------------|---|

By comparing this path definition to the original definition (‎9.25) the following relation is obtained:

|   | $$ A = \frac{1}{\sqrt{a}} $$ |   |
|---|------------------------------|---|

The yaw angle at the end of the path is

|   | $$ \psi_{E} = \psi \left( x_{E} \right) = \arctan \frac{s_{2}^{'} \left( x_{E} \right)}{s_{1}^{'} \left( x_{E} \right)} = \frac{a x_{E}^{2}}{2} + \psi_{0} $$ |   |
|---|---------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

The integrals of the clothoid correspond to *Fresnel integral functions* defined as

|   | $$ f_{C} ( z ) = \int_{0}^{z} {\cos \left( \frac{\pi t^{2}}{2} \right)} d t $$ |   |
|---|--------------------------------------------------------------------------------|---|
|   | $$ f_{S} ( z ) = \int_{0}^{z} {\sin \left( \frac{\pi t^{2}}{2} \right)} d t $$ |   |

See MATLAB/MuPAD doc fresnel.

The clothoid integrals are mapped to the Fresnel functions by substituting

|   | $$ \frac{a}{2} \xi^{2} = \frac{\pi t^{2}}{2} $$ |   |
|---|-------------------------------------------------|---|

In the case of $$\psi_{0} = 0$$ the path definition can be expressed in terms of the Fresnel functions:

|   | $$ \xi = \sqrt{\frac{\pi}{a}} t $$                                                                                                                                                                               |   |
|---|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|
|   | $$ s_{1} ( x ) = s_{01} \  + \sqrt{\frac{\pi}{a}} \int_{0}^{\sqrt{\frac{a}{\pi}} x} {\cos \left( \frac{\pi t^{2}}{2} \right)} d t = s_{01} + \sqrt{\frac{\pi}{a}} f_{C} \left( \sqrt{\frac{a}{\pi}} x \right) $$ |   |
|   | $$ s_{2} ( x ) = s_{02} + \sqrt{\frac{\pi}{a}} f_{S} \left( \sqrt{\frac{a}{\pi}} x \right) $$                                                                                                                    |   |

Since $$f_{S}$$ and $$f_{C}$$ both converge to $$\frac{1}{2}$$ for $$z \rightarrow \infty$$, the limit point of left-turned closing clothoids for infinite path length $$x \rightarrow \infty$$ is:

|   | $$ \lim_{x \rightarrow \infty} {\mathbf{s} ( x )} = \begin{pmatrix}s_{01} + \frac{1}{2} \sqrt{\frac{\pi}{a}} \\ s_{02} + \frac{1}{2} \sqrt{\frac{\pi}{a}}\end{pmatrix} $$ |   |
|---|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|

Left-turned Opening Clothoids

The approach for the velocity vector of an opening clothoid with initial yaw $$\psi_{0}$$ is:

|   | $$ s_{1}^{'} ( x ) = \cos {\left( - \frac{a}{2} \left( x_{E} - x \right)^{2} + \psi_{0} + \frac{a}{2} x_{E}^{2} \right) = \cos \left( - \frac{a}{2} x^{2} + a x_{E} x + \psi_{0} \right)} $$ |   |
|---|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---|
|   | $$ s_{2}^{'} ( x ) = \sin \left( - \frac{a}{2} \left( x_{E} - x \right)^{2} + \psi_{0} + \frac{a}{2} x_{E}^{2} \right) = \sin \left( - \frac{a}{2} x^{2} + a x_{E} x + \psi_{0} \right) $$   |   |

The definition of the curve then is:

|   | $$ s_{1} ( x ) = s_{01} + \int_{0}^{x} {\cos \left( - \frac{a}{2} \xi^{2} + a x_{E} \xi + \psi_{0} \right)} d \xi $$ |   |
|---|----------------------------------------------------------------------------------------------------------------------|---|
|   | $$ s_{2} ( x ) = s_{02} + \int_{0}^{x} {\sin \left( - \frac{a}{2} \xi^{2} + a x_{E} \xi + \psi_{0} \right)} d \xi $$ |   |

The curvature of opening clothoids is calculated as:

|   | $$ s_{1}^{' '} = - a \left( x_{E} - x \right) \sin \left( \frac{a}{2} x^{2} - a x_{E} s + \psi_{0} \right) $$ |   |
|---|---------------------------------------------------------------------------------------------------------------|---|
|   | $$ s_{2}^{' '} = a \left( x_{E} - x \right) \cos \left( \frac{a}{2} x^{2} - a x_{E} x + \psi_{0} \right) $$   |   |
|   | $$ \kappa ( x ) = \sqrt{s_{1}^{' '}^{2} + s_{2}^{' '}^{2}} = a ( x_{E} - x ) $$                               |   |

Right-turned Opening and Closing Clothoids

For right-turned clothoids, the parameter $$a$$ assumes negative values. The definition formulas are identical to the left-turned opening and closing clothoids.

1.  Dictionary

| **a** | acceleration                                    | Beschleunigung                           |
|-------|-------------------------------------------------|------------------------------------------|
|       | arc length                                      | Bogenlänge                               |
|       | angular velocity                                | Winkelgeschwindigkeit                    |
| **b** | backward differences method                     | Rückwärtsdifferenzenverfahren            |
|       | belt tensioner                                  | Gurtstraffer                             |
|       | bicycle model                                   | Einspurmodell                            |
|       | blind spot                                      | toter Winkel                             |
|       | Bode plot                                       | Bode-Diagramm                            |
| **c** | calibration                                     | Applikation                              |
|       | cascaded control                                | Kaskadenregelung                         |
|       | center of gravity (COG)                         | Schwerpunkt                              |
|       | circular arc                                    | Kreisbogenabschnitt                      |
|       | closed-loop control                             | Regelung                                 |
|       | clothoid (Euler Curve, Cornu Spiral, spiro)     | Klothoide                                |
|       | control deviation                               | Regelabweichung                          |
|       | control lag                                     | Schleppabstand                           |
|       | control loop                                    | Regelkreis                               |
|       | corner frequency                                | Eckfrequenz                              |
|       | control signal (manipulated variable)           | Stellgröße                               |
|       | course attendance time                          | Präsenzzeit Vorlesung                    |
|       | course lecture                                  | Vorlesung                                |
|       | cruise control                                  | Tempomat                                 |
|       | curvature                                       | (Bahn-)Krümmung                          |
| **d** | data points                                     | Stützpunkte                              |
|       | dead time                                       | Totzeit                                  |
|       | deceleration                                    | Verzögerung, Abbremsen                   |
|       | derivative                                      | Ableitung, Differentiation               |
|       | deviation                                       | Auslenkung                               |
|       | discontinuous                                   | unstetig, kann Sprünge aufweisen         |
|       | distance control                                | Abstandshalteassistent                   |
|       | driver assistance systems                       | Fahrerassistenzsysteme                   |
| **e** | electronic parking brake                        | Elektronische Parkbremse                 |
|       | emergency brake assist                          | Notbremsassistent                        |
|       | exam                                            | Prüfung                                  |
|       | Explicit Euler                                  | expliziter Euler, Euler-Vorwärts         |
| **f** | fatalities                                      | Todesfälle                               |
|       | finite state machine                            | endlicher Zustandsautomat                |
|       | first-order derivative                          | Ableitung erster Ordnung                 |
|       | frequency response                              | Frequenzgang                             |
|       | forward differences method                      | Vorwärtsdifferenzenverfahren             |
|       | frequency domain                                | Frequenzbereich                          |
|       | friction                                        | Reibung                                  |
|       | fully automated driving                         | Vollautomatisiertes Fahren               |
| **g** | gain crossover frequency                        | Durchtrittsfrequenz                      |
|       | gain margin                                     | Amplitudenrand, Amplitudenreserve        |
|       | to glare                                        | blenden                                  |
| **h** | headlights                                      | Scheinwerfer                             |
|       | high beam                                       | Fernlicht                                |
|       | highly automated driving                        | hochautomatisiertes Fahren               |
|       | horizontal surface                              | waagerechte Ebene                        |
|       | illumination                                    | Ausleuchtung                             |
|       | Implicit Euler                                  | impliziter Euler, Euler-Rückwärts        |
| **i** | inductance                                      | Induktivität                             |
|       | inertial measurement unit IMU                   | Inertialmesseinheit                      |
|       | frame                                           | Koordinatensystem                        |
|       | initial condition (IC)                          | Anfangsbedingung                         |
|       | input (signal) vector                           | Eingangssignalvektor                     |
|       | instrument cluster                              | Kombiinstrument                          |
| **j** | jerk                                            | Ruck (Ableitung der Beschleunigung)      |
| **l** | lab project                                     | Laborprojekt                             |
|       | lane                                            | Fahrspur                                 |
|       | lane keeping assist                             | Spurhalteassistent                       |
|       | lateral acceleration                            | Beschleunigung in Fahrzeug-Querrichtung  |
|       | lateral control (by steering)                   | Querregelung (durch Lenken)              |
|       | longitudinal acceleration                       | Beschleunigung in Fahrzeug-Längsrichtung |
|       | longitudinal control (by braking, acceleration) | Längsregelung (durch Bremsen, Gasgeben)  |
| **m** | margin                                          | Rand                                     |
|       | moment of inertia                               | Massenträgheitsmoment                    |
|       | monitoring                                      | Überwachung                              |
| **o** | observer                                        | Beobachter                               |
|       | obstacle                                        | Hindernis                                |
|       | offline working hours                           | Aufwand außerhalb der Präsenzzeit        |
|       | open-loop control                               | Steuerung                                |
|       | operation point                                 | Arbeitspunkt                             |
|       | ordinary differential equation (ODE)            | gewöhnliche Differentialgleichung (Dgl.) |
|       | overshoot                                       | Überschwingweite (der Sprungantwort)     |
|       | overshoot time                                  | Überschwingzeit (der Sprungantwort)      |
|       | output (signal) vector                          | Ausgangssignalvektor                     |
| **p** | parking assist                                  | Einparkassistent                         |
|       | path following                                  | Pfadverfolgung                           |
|       | path length                                     | Pfadlänge                                |
|       | pedestrian                                      | Fußgänger                                |
|       | phase margin                                    | Phasenrand, Phasenreserve                |
|       | planar surface                                  | ebene Fläche                             |
|       | plant (dynamics)                                | Regelstrecke                             |
|       | process variable                                | Regelgröße                               |
|       | to pitch                                        | nicken                                   |
|       | plant                                           | Regelstrecke                             |
|       | power train                                     | Antriebsstrang                           |
| **r** | rear-end collision                              | Auffahrunfall                            |
|       | rear wheel drive                                | Hinterradantrieb                         |
|       | regularity framework                            | rechtlicher Rahmen                       |
|       | reference path                                  | Solltrajektorie                          |
|       | reference variable                              | Führungsgröße                            |
|       | requirement                                     | Anforderung                              |
|       | to roll                                         | rollen, wanken                           |
| **s** | sampling points                                 | Stützstellen                             |
|       | seat belt tensioner                             | Gurtstraffer                             |
|       | self-localization                               | Eigenlokalisierung                       |
|       | semi-automated driving                          | Teilautomatisiertes Fahren               |
|       | set point                                       | Sollwert                                 |
|       | side bolster                                    | Seitenwange                              |
|       | slip angle                                      | Schwimmwinkel                            |
|       | speed                                           | Geschwindigkeit                          |
|       | state estimator                                 | Zustandsschätzer                         |
|       | state-space model                               | Zustandsraumdarstellung                  |
|       | state (signal) vector                           | Zustandsvektor                           |
|       | static gain                                     | statische Verstärkung                    |
|       | steady state                                    | Ruhelage, stationärer Zustand            |
|       | steering angle                                  | Lenkwinkel                               |
|       | step-down converter                             | Tiefsetzsteller                          |
|       | step response                                   | Sprungantwort                            |
|       | straight line                                   | Gerade                                   |
|       | summer term                                     | Sommersemester                           |
|       | system architecture diagram                     | physikalisches Ersatzschaltbild          |
| **t** | teaching objectives                             | Lernziele                                |
|       | time constant                                   | Zeitkonstante                            |
|       | torque                                          | Drehmoment                               |
|       | transfer element                                | Übertragungsglied                        |
|       | transfer function                               | Übertragungsfunktion                     |
|       | trajectory control                              | Bahnregelung                             |
|       | trapezoidal rule                                | Trapezregel                              |
| **u** | ultrasonic sensor                               | Ultraschallsensor                        |
| **v** | vehicle dynamics                                | Regelstrecke                             |
|       | velocity                                        | Geschwindigkeitsvektor                   |
|       | voltage                                         | elektrische Spannung                     |
| **w** | way points                                      | Stützpunkte                              |
|       | wheel base                                      | Radstand                                 |
|       | wheel suspension                                | Radaufhängung                            |
|       | wheel slip                                      | Radschlupf                               |
| **y** | yaw angle                                       | Gierwinkel                               |
|       | yaw rate                                        | Gierrate                                 |

1.  Literature

    Angermann, A. (2011). *MATLAB - Simulink - Stateflow: Grundlagen, Toolboxen, Beispiele.* München: Oldenbourg.

    Ansgar Meroth, F. T. (2014, October 8-11). Optimization of the development process of intelligent transportation systems using Automotive SPICE and ISO 26262 . *IEEE 17th International Conference on Intelligent Transportation Systems (ITSC)*, p. 2014.

    BAST. (2012). *Rechtsfolgen zunehmender Fahrzeugautomatisierung.* Bergisch Gladbach: Bundesanstalt für Straßenwesen.

    Breymann, U. (2016). *C++: eine Einführung.* München: Hanser.

    Dubins, L. (1957). On curves of minimal length with a constraint on overage curvature, and with prescribed initial and terminal positions and tangents. *American Journal of Mathematics*, pp. 497-516.

    Effertz, J. (2009). Autonome Fahrzeugführung in urbaner Umgebung durch Kombination objekt- und kartenbasierter Umfeldmodelle. *Dissertation*. Technische Universität Braunschweig.

    Grote, C., & Rau, R. (2013). Auf dem Weg zum hochautomatisierten Fahren. *VDI-Berichte Nr. 2188*, pp. 559-570.

    Heining, N. (2017). *Autonome Modellfahrzeuge: Aufbau einer Rennstrecke und Simulation der Fahrdynamik.* Hochschule Heilbronn.

    Kowoma. (n.d.). *Kowoma-GPS*. Retrieved from http://www.kowoma.de/gps/

    Lunze, J. (2004). *Regelungstechnik 1.* Berlin, Heidelberg: Springer-Verlag.

    Lunze, J. (2013). *Regelungstechnik 2.* Heidelberg: Springer Vieweg.

    Mercedes-Benz. (2013). Webseite http://www.mercedes-benz.de.

    Mercer Management Consulting und Frauenhofer Gesellschaft. (2004). *Future Automotive Industry Structure 2015.*

    Pacejka, H., & Bakker, E. (1992). The Magic Formula Tyre Model. *Vehicle system dynamics 21*, 1-18.

    Reeds, J., & Shepp, L. (1990). Optimal paths for a car that goes both forwards and backwards. *Pacific Journal of Mathematics*, pp. 367-393.

    Reif, K. (2010). *Fahrstabilisierungssysteme und Fahrerassistenzsysteme.* Wiesbaden: Vieweg + Teubner.

    Riekert, P., & Schunk, T. (1940). Zur Fahrmechanik des gummibereiften Kraftfahrzeugs. *Ingenieurarchiv 11*.

    SAE. (2016). *Taxonomy and Definitions for Terms Related to Driving Automation Systems for On-Road Motor Vehicles, J3016_201609.* SAE International.

    Schiessle, E. (2010). *Industriesensorik.* Würzburg: Vogel.

    Schulz, G. (2008). *Regelungstechnik 2.* München: Oldenbourg.

    Schulz, G. (2010). *Regelungstechnik 1.* München: Oldenbourg.

    Stroustrup, B. (2015). *Die C++-Programmiersprache: Aktuell zu C++11.* München: Hanser.

    Stroustrup, B. (2015). *Eine Tour durch C++: Die kurze Einführung in den neuen Standard C++11.* München: Hanser.

    Sussmann, H. J., & Guoqing, T. (1991). Shortest paths for the Reeds-Shepp car: a worked out example of the use of geometric techniques in nonlinear optimal control. *Rutgers Center for Systems and Control Technical Report 10*, pp. 1-71.

    Tränkle, F. (2011). Ein Modellfahrzeug für den regelungstechnischen Laborversuch. *Tagungsband AALE, 8. Fachkonferenz.* Göppingen.

    Tränkle, F. (2014). *Modellbasierte Softwareentwicklung, Skript zur Vorlesung.* http://ilias.hs-heilbronn.de/goto.php?target=crs_1539111&client_id=hshn.

    Tränkle, F. (2021). *Modellbasierte Entwicklung mechatronischer Systeme: mit Software- und Simulationsbeispielen für autonomes Fahren.* DeGruyter Studium.

    VDI. (2004). *VDI-Richtlinie 2206: Entwicklungsmethodik für mechatronische Systeme.*

    William Press, S. T. (2007). *Numerical Recipes in C.* Cambridge Press.

    Winner, H., Hakuli, S., & Wolf, G. (2012). *Handbuch Fahrerassistenzsysteme.* Wiesbaden: Vieweg + Teubner.
