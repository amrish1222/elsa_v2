# ELSA_V2 - Explore, Localize, Map Simultaeneously and Act - Version 2.0

Elsa V2.0 is the extension for Elsa V1.0. This package is built to boost the capabilities of elsa in terms of Improved Localization, Powerful Actuation, Reduced Wheel Slippage and Drift, Boosted Payload capacity, Better choice of Electronics

For the Version 1.0 of elsa project visit [this](https://github.com/srujanpanuganti/elsa)

## Project Dependencies

* To install teleop-twist-keyboard

`sudo apt-get install ros-melodic-teleop-twist-keyboard`
  
* To install `rosserial`

`sudo apt-get install ros-melodic-rosserial-arduino`

`sudo apt-get install ros-melodic-rosserial`


## Docs:
* Coordinate Transformations and Robot Dimensions can be found at `docs/Robot dimensions and coordinate transformations.pdf`
* More info about the references used is provided in the reference section


## Finished tasks:

* Assemble Robot Platform, Motors, Encoders
* Interface encoders with Arduino Mega - Arduino Nano doesn't work when all encoders and all motors are interfaced
* Interface motors to Arduino Mega - Arduino Nano doesn't work when all encoders and all motors are interfaced
* Publish encoder ticks to the topics `/flwheel`, `/rlwheel`, `/frwheel`, `/rrwheel` for FrontLeft, RearLeft, FrontRight, RearRight
* Subscribe to `/cmd_vel` topic and move motors - Differential drive
* Use `teleop_twist_keyboard` node to move the mobile base

## To Do:

* Setup Raspberry Pi 4B to run remote roscore
* Calculate Odometry using the `/flwheel`, `/rlwheel`, `/frwheel`, `/rrwheel` and publish the Odometry to `/odom` topic
* Interface imu and Publish imu to `/imu/raw_data` topic 
* Subscribe to `/cmd_vel` topic and move motors - mechanum drive
* interface YDLIDAR X4 to the Robot and Publish LaserScan to `/scan` topic
* Construct chassis and fix imu and YDLIDAR properly
* Construct TF for the `/odom`, `/base_link`, `/base_laser` coordinate frames
* use robot_localization Extended-Kalman-Filter to filter and fuse `/imu/raw_data` and `/odom` to obtain `/odom_filtered`
* use gmapping package to do Simultaeneous Localization and Mapping


## Results:


## References:

* The arduino based base_controller takes some of the ideas from [here](https://github.com/panagelak/4WD-drive-arduino-code-with-rosserial-encoders-pid) 
* Assembling the robot platform can be followed from [here](https://github.com/MoebiusTech/MecanumRobot-ArduinoMega2560) 
