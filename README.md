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

* Connecting robots
  * First robot (id 1) is connected to 14 (TX, yellow) and 15 (RX, white)
  * Second robot (id 2) is connected to 16 (TX, yellow) and 17 (RX, white)
  * Third robot (id 3) is connected to 18 (TX, yellow) and 19 (RX, white)
  * Additionally the servos must be powered by 12V.
  * The ground (GND) of Arduino and the ground of the power supply must be connected
  * The program can run with one, two or three robots
* At default only three servos per robot are supported
  * To allow more, a define must be changed an the cmd parse must be extended

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

Set breaks:
* **Input**: rob;\<id>;break;\<break_status>
* **Return**: rob;\<id>;break;
* **Note**: Enable or disable breaks (disable for teaching; enable for movement)

Status of the robot:
* **Input**: rob;\<id>;status
* **Return**: rob;\<id>;\<disabled/uninitialized/disconnected/idle/moving>
* **Note**: disabled (Collision was detected, start robot with start command and maybe clear before); uninitialized (the init command has to be send before); disconnected (one or more servos are not connected); idle (there is nothing in the buffer and robot is not moving); moving (robot moves at the moment)

## Parameters
* **id**: Id of the robot starting by 1
* **thetaX**: Absolut angle depending on the init
* **speedX**: The speed in rpm the servo should move
* **exact**: Defines if the point should be reached exactly (e.g.: when using linear movement use false for points in between)
* **speed_smooth**: Defines if the speed should be increased and decreased smoothly
* **synchronize**: Defines if all servos should be synchronized. That means that they are starting and stopping the movement at the same time
* **delay**: Defines how many milliseconds the robot should wait after the movement