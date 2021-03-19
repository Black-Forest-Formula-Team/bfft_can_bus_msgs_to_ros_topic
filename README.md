# bfft_formula-student_driverless

<p align="center">
  <a href="https://blackforestformula.hs-offenburg.de/">
    <img alt="BFFT_Logo" title="BFFT" src="https://scontent-frt3-1.xx.fbcdn.net/v/t1.0-9/69419451_117866062911797_4569414645357477888_o.jpg?_nc_cat=107&ccb=1-3&_nc_sid=973b4a&_nc_ohc=b5rqMomf8_AAX8x_CMD&_nc_ht=scontent-frt3-1.xx&oh=7ab30784f93fdf5ad846196156f856e6&oe=606D20C4" width="1000">
  </a>
</p>

# Black Forest Formula Team - Formula Student Driverless 2021
This package provides the CAN-bus support for the infrastructure of our future autonomous system which is planned for the task to read in all sensor and VCU messages of our first electric racecar we currently develop. This code is created and maintained by the [Black Forest Formula Team](https://blackforestformula.hs-offenburg.de/) at [University of Applied Sciences Offenburg](https://www.hs-offenburg.de/). As this is a subrepository of the overall system you will find more information about the system in the main repository and its [Wiki](https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/wiki).

____________________


## Repository organisation
<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->


- [Introduction](#introduction)
- [Hardware & Software Preconditions](#hardware--software-preconditions)
  - [Hardware Stack](#hardware-stack)
  - [Software Stack](#software-stack)
- [Setup](#setup)
- [Getting started](#getting-started)
  - [Clone packages](#clone-packages)
  - [Build Process](#build-process)
  - [Connecting IMU and sensors via CAN bus](#connecting-imu-and-sensors-via-can-bus)
- [Code Repository Conventions](#code-repository-conventions)
  - [Python](#python)
  - [ROS Python](#ros-python)
  - [ROS naming conventions](#ros-naming-conventions)
    - [Work packages](#work-packages)
    - [ROS packages](#ros-packages)
    - [ROS nodes](#ros-nodes)
    - [ROS topics](#ros-topics)
    - [ROS messages](#ros-messages)
- [Feedback](#feedback)
- [Our Developers](#our-developers)
- [Release History](#release-history)
- [Meta](#meta)
- [Contributing to one of our Repos](#contributing-to-one-of-our-repos)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->
____________________
## Introduction
To be able to one day drive our future electric racecar without a driver in one of the Formula Student (FSAE) competitions we have to set up a baseline infrastructure reading in all relevant sensor data (perception layer). We are using the CAN-bus to communicate inside the vehicle and use several different sensors and one vehicle control unit (VCU, STM32-board) which we have to communicate with our autonomous control unit (ACU, NVIDIA Jetson AGX).

Our overall autonomous setup includes the Jetson AGX, two D455 cameras, one IMU from Genesys (ADMA Slim) as well as several CAN-Sensors and actors (for example two motors, inverters, wheelspeed sensors, BMS, ...) as can partly be seen in the image below.
<p align="center">
  <img src = "https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/blob/main/img/bfft_autonomous_driving_setup_20210311.png" width=1000>
</p>

Right now we support in this package all (for us) relevant CAN messages and their extraction and conversion we receive of the [Genesys Adma Slim IMU](https://www.genesys-offenburg.de/en/products/adma-slim-mini-gnssinertial-system/). A great thank you goes out to our sponsor [GeneSys Elektronik GmbH](https://www.genesys-offenburg.de/) for enabling us to work with this state-of-the-art IMU as a long-term rental! :-)
____________________
## Hardware & Software Preconditions
We are using the following hardware and software stack:
### Hardware Stack
* [NVIDIA Jetson AGX Xavier](https://elinux.org/Jetson_AGX_Xavier) with 32 GB RAM, upgraded 512 GB SSD and upgraded Intel Wifi 8265 
* [Intel D455 Cameras (x2)](https://www.intelrealsense.com/depth-camera-d455/)
* [Genesys Adma Slim IMU](https://www.genesys-offenburg.de/en/products/adma-slim-mini-gnssinertial-system/)
* Some kind of [CAN Transceiver](https://medium.com/@ramin.nabati/enabling-can-on-nvidia-jetson-xavier-developer-kit-aaaa3c4d99c9), DB9 connectors, cables, pins and other basic electrical hardware

### Software Stack
* Ubuntu 18.04 LTS
* Python 3.X
* ROS 1 Melodic
* JetPack 4.4
____________________
## Setup
For our Hardware and Software setup please visit the page [Setup of NVIDIA Jetson AGX Xavier](https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/wiki/01-Setup-of-NVIDIA-Jetson-AGX-Xavier) in our Wiki. There you will find a guide how to install ROS Melodic, relevant drivers and libraries. You can also find a guide there how to enable CAN on the NVIDIA Jetson Board and how to wire it to a CAN sensor using a CAN-transceiver.
____________________
## Getting started
Assuming you are used to ROS1, have your hardware wired and/or followed the Wiki pages mentioned above you should now be able to clone this package into your catkin workspace (we assume it lays under ```cd ~/catkin_ws/```, you might have to adjust this to your needs). 

### Clone packages
To do so clone this package as well as the ```ros_canopen``` package into your workspace and make sure you installed the necessary libaries ([Link to Wiki](https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/wiki/01-Installation-Libraries-for-Workspace-(Python-and-ROS))).

* [ros_canopen](https://github.com/ros-industrial/ros_canopen): Forward incoming and outgoing CAN Messages to and from ROS topics, interface between logic and CAN-hardware.
```
cd ~/catkin_ws/src/
git clone https://github.com/ros-industrial/ros_canopen.git
```
* [bfft_can_bus_msgs_to_ros_topic](https://github.com/Black-Forest-Formula-Team/bfft_can_bus_msgs_to_ros_topic): Decode incoming CAN messages and publish them to corresponding topics
```
cd ~/catkin_ws/src/
git clone https://github.com/Black-Forest-Formula-Team/bfft_can_bus_msgs_to_ros_topic.git
```

For more input please refer to the [Catkin Docs](https://wiki.ros.org/catkin/workspaces)

### Build Process
Now we are able to build the workspace (if we have all libraries installed) with the packages downloaded above. 
```
catkin_make
```
If a library is missing make sure to install it via ```sudo apt-get install ros-melodic-libraryname``` if its a ROS library or via ```pip3 install libraryname``` if its a python3 lib.

Source setup file to be able to execute ros commands from every terminal
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Connecting IMU and sensors via CAN bus
For setup refer to this [page](https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/wiki/02-Setup-ADMA-Slim-IMU-from-Genesys-using-CAN).

Start the ROS node to listen to incoming CAN messages with:
```
roslaunch bfft_CAN_msgs_to_ROS_topic Start_Data_Collection.launch 
```
If you would like to see data comming in you can try one of the following as long as the IMU is attached:
```
rostopic echo /imu/imu_data
rostopic echo /imu/gps_data
```
It is possible to get a list of all available topics by typing ```rostopic list```.

When using another IMU you will have to adjust the CAN-IDs in the ```can_to_topic file```.


________________________________
## Code Repository Conventions
For our coding conventions please visit the wiki page [ROS & Python Conventions](https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/wiki/00-Coding-Conventions)!

____________________
## Feedback

Feel free to send us feedback! 

If there's anything you'd like to chat about, please feel free to text us on one of our social media plattforms: 
* [Instagram](https://www.instagram.com/black_forest_formula/) 
* [Facebook](https://www.facebook.com/blackforestformula/) 
* [LinkedIN](https://linkedin.com/company/20527126)!

Support this project by becoming a sponsor. Your logo will show up on our website with a link to your website. [[Become a sponsor](https://blackforestformula.hs-offenburg.de)]

____________________
## Our Developers
Dev-Team Vehicle Control Unit & Autonomous Driving in alphabetical order
* [Alex](https://github.com/AlexSperka) - Initial work
* [Benedikt](https://github.com/newtop95) - Initial work
* [Steffi](https://github.com/steffistae) - Initial work
* [Tizian](https://github.com/tdagner) - Initial work

____________________
## Release History
* 0.0.1
    * Initial setup, work in progress

____________________
## Meta
Distributed under the MIT license. See ``LICENSE.md`` for more information.

____________________
## Contributing to one of our Repos
1. Fork it (<https://github.com/Black-Forest-Formula-Team/bfft_can_bus_msgs_to_ros_topic/fork>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request

____________________


<p align="left">
  <a href="https://www.hs-offenburg.de/">
      <img alt="HSO_Logo" title="HSO_Logo" src="https://static.onthehub.com/production/attachments/15/66edb074-5e09-e211-bd05-f04da23e67f6/7978f7db-e206-4cd7-b7b2-6d9696e98885.png" width="1000">
  </a>
</p>