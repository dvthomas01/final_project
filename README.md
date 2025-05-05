# 2.12 Robot Navigation

Mobile Robot Team Th 11-1

This repository will host the navigation code and control systems for our 2.12 robot project.

<details>
  <summary>Table of Contents</summary>

- [1 Robot](#1-robot)
- [2 Controller](#2-controller)
- [3 Jetson](#3-jetson)

</details>

## 1 Robot

Upload the MR_1_robot environment to the ESP on the robot.
To do this:
  1. Put the ESP in to upload mode
  2. Upload the code
  3. Press the reset button on the ESP.
Once the code has been uploaded, disconnect the computer from the ESP. The board power (red button on robot chassis) can now be turned on.

## 2 Controller

Upload the MR_1_controller environment to the ESP on the controller.
Follow the same steps as the robot to upload code to this ESP.
The joystick must stay connected to power while in use. 

Pressing the red button furthest from the ESP will switch the robot from autonomous mode into manual control mode.
Once in manual control mode, the robot will be fully controlled by the joystick and buttons.

The joystick controls the drive wheels of the robot.
The green button spins the flywheels to suck in bins.
The red button next to the green button spins the flywheels to spit out the bins.

Importantly, once in joystick mode, there is no way to return to autonomy without reseting the robot ESP and ctrl+c'ing and restarting the high-level code on the Jetson Nano.

## 3 Jetson

From a terminal, type the following commands in order to connect wirelessly to the Jetson Nano and run the appropriate file:

ssh robot212@10.31.167.202

Type password: admin

cd Desktop/final_project/jetson/

MR_1_jetson_main.py

Once the jetson is running, turning on motor power (green button on motor chassis) will make the robot start moving and following the Jetson's commands.