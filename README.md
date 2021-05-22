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
* There is always a collision detection enabled

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
* At default the first robot is connected at 16 (TX) and 17 (RX), and the second robot at 14 (TX) and 15 (RX). Additionally the servos must be powered by 12V.
  * It is possible to add one more robot.
* At default only three servos per robot are supported
  * A define must be changed an the cmd parse must be extended

# API

## Endpoints
All the following endpoints can be reached with a serial connection. For example when developing python use the "pyserial" package.

Initialize Position:
* **Input**: rob;\<id>;init;\<theta1>;\<theta2>;\<theta3>
* **Return**: rob;\<id>;init
* **Note**: set angles to zero when tcp already in coordinate origin

Simple Movement:
* **Input**: rob;\<id>;move;\<theta1>;\<speed1>;\<theta2>;\<speed2>;\<theta3>;\<speed3>
* **Return**: rob;\<id>;move
* **Asynchron Return**: rob;\<id>;async;true;false;false;0;\<theta1>;\<speed1>;\<theta2>;\<speed2>;\<theta3>;\<speed3>
* **Note**: same as moveAdv (exact=true;speed_smooth=false;synchronize=false;delay=0). Asychron return when move is finished

Advanced Movement:
* **Input**: rob;\<id>;moveAdv;\<exact>;\<speed_smooth>;\<synchronize>;\<delay_ms>;\<theta1>;\<speed1>;\<theta2>;\<speed2>;\<theta3>;\<speed3>
* **Return**: rob;\<id>;moveAdv
* **Asynchron Return**: rob;\<id>;async;\<exact>;\<speed_smooth>;\<synchronize>;\<delay_ms>;\<theta1>;\<speed1>;\<theta2>;\<speed2>;\<theta3>;\<speed3>
* **Note**: when synchronize=true then only the \<speed1> is used for all servos. Asychron return when move is finished

Move to home position:
* **Input**: rob;\<id>;home
* **Return**: rob;\<id>;home
* **Asynchron Return**: rob;\<id>;async;true;false;false;0;\<theta1>;5;\<theta2>;5;\<theta3>;5

Get Angles of servos:
* **Input**: rob;\<id>;angles
* **Return**: rob;\<id>;\<theta1>;\<theta2>;\<theta3>

Start all servos:
* **Input**: rob;\<id>;start
* **Return**: rob;\<id>;start
* **Note**: When starting the programm this command is triggert automatically

Stop all servos:
* **Input**: rob;\<id>;stop
* **Return**: rob;\<id>;stop

Clear all buffered commands:
* **Input**: rob;\<id>;clear
* **Return**: rob;\<id>;clear

Status of the robot:
* **Input**: rob;\<id>;status
* **Return**: rob;\<id>;\<disabled/uninitialized/idle/moving>
* **Note**: disabled (Collision was detected, start robot with start command and maybe clear before); uninitialized (the init command has to be send before); idle (there is nothing in the buffer and robot is not moving); moving (robot moves at the moment)

## Parameters
* **id**: Id of the robot starting by 1
* **thetaX**: Absolut angle depending on the init
* **speedX**: The speed in rpm the servo should move
* **exact**: Defines if the point should be reached exactly (e.g.: when using linear movement use false for points in between)
* **speed_smooth**: Defines if the speed should be increased and decreased smoothly
* **synchronize**: Defines if all servos should be synchronized. That means that they are starting and stopping the movement at the same time
* **delay**: Defines how many milliseconds the robot should wait after the movement