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

Upload the MR_1_robot environment to the ESP on the robot chassis.
Do this by putting the ESP in to upload mode, uploading the code, and then pressing the reset button on the ESP.
Once the code has been uploaded, disconnect the computer from the ESP.

## 2 Controller

Upload the MR_1_controller environment to the ESP that controls the joystick and buttons.
Follow the same steps from the robot to upload code to this ESP.
The joystick must stay connected to power while in use. 

Pressing the red button furthest from the ESP will put the robot from autonomous mode into manual control mode.
Once in manual control mode, the robot will be fully controlled by the joystick and buttons.

The joystick controlls the driving wheels of the robot.
The green button spins the flywheels to suck in the bins.
The red button directly above to the green button spins the flywheels to spit out the bins.


## 3 Jetson

From the terminal, type the following commands in order to connect wirelessly to the Jetson Nano and run the appropriate file:

ssh robot212@10.31.167.202
Type password: admin
cd Desktop/final_project/jetson/
MR_1_jetson_main.py   