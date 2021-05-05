# Overview
This is an implementation for controlling a robot with MS-12A motors from Makeblock. 

# Features
* PTP Movement 
* PTP Movement with synchronized servos (movement starts and ends at same time)
* PTP Movement where the speed is slightly increased and decreased
* Linear Movement where points in between can be buffered
  * So the movement is smoother because of less latency
  * The points have to be calculated separately
* Several other helpful functions listed in chapter API

# Prerequisites

* Arduino: https://www.arduino.cc/en/software
* Makeblock-Libraries: https://github.com/Makeblock-official/Makeblock-Libraries
* (optional) vscode: https://code.visualstudio.com/
  * with extension: vsciot-vscode.vscode-arduino

# Installation

* With git: git clone https://github.com/martin-kritzl/MMR2-MECH-Arduino.git
* Or download zip from: https://github.com/martin-kritzl/MMR2-MECH-Arduino
* In Arduino:
  * Tools
    * Board: Arduino/Genuino Mega 2560
    * Programmer: AVRISP mkII
    * Port: ...choose
  * Sketch
    * Upload

# Preperation

* Make sure the setup function and the defines are correct for your usage
  * But should work out of the box
* At default the first robot is connected at 16 (TX)and 17 (RX), and the second robot at 14 (TX) and 15 (RX). Additionally the servos must be powered by 12V.
  * It is possible to add one more robot.
* At default only three servos per robot are supported
  * A define must be changed an the cmd parse must be extended

# API

## Endpoints
Initialize Position:
* **Input**: \<id>;init;\<theta1>;\<theta2>;\<theta3>
* **Note**: set angles to zero when tcp already in coordinate origin

Simple Movement:
* **Input**: \<id>;move;\<theta1>;\<speed1>;\<theta2>;\<speed2>;\<theta3>;\<speed3>
* **Return**: \<id>;move;true;false;false;\<theta1>;\<speed1>;\<theta2>;\<speed2>;\<theta3>;\<speed3>
* **Note**: same as moveAdv (exact=true;speed_smooth=false;synchronize=false)

Advanced Movement:
* **Input**: \<id>;moveAdv;\<exact>;\<speed_smooth>;\<synchronize>;\<theta1>;\<speed1>;\<theta2>;\<speed2>;\<theta3>;\<speed3>
* **Return**: \<id>;move;\<exact>;\<speed_smooth>;\<synchronize>;\<theta1>;\<speed1>;\<theta2>;\<speed2>;\<theta3>;\<speed3>
* **Note**: when synchronize=true then only the \<speed1> is used for all servos

Move to home position:
* **Input**: \<id>;home
* **Return**: \<id>;move;\<exact>;\<speed_smooth>;\<synchronize>;\<theta1>;\<speed1>;\<theta2>;\<speed2>;\<theta3>;\<speed3>

Get Angles of servos:
* **Input**: \<id>;angles
* **Return**: \<id>;\<theta1>;\<theta2>;\<theta3>

Stop all servos:
* **Input**: \<id>;stop

Status of the robot:
* **Input**: \<id>;status
* **Return**: \<id>;\<idle/moving>

## Parameters
* **id**: Id of the robot starting by 1
* **thetaX**: Absolut angle depending on the init
* **speedX**: The speed in rpm the servo should move
* **exact**: Defines if the point should be reached exactly
* **speed_smooth**: Defines if the speed should be increased and decreased smoothly
* **synchronize**: Defines if all servos should be synchronized. That means that they are starting and stopping the movement at the same time