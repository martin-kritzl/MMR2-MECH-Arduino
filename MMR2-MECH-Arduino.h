/*
Copyright (C) 2021 Martin Kritzl

This file is part of MMR2-MECH-Arduino
(https://github.com/martin-kritzl/MMR2-MECH-Arduino)


MMR2-MECH-Arduino is free software: you can redistribute it and/or modify 
it under the terms of the GNU General Public License as published by 
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful, 
but WITHOUT ANY WARRANTY; without even the implied warranty of 
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
GNU General Public License for more details.


You should have received a copy of the GNU General Public License 
along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef MMR2_MECH_ARDUINO_H
#define MMR2_MECH_ARDUINO_H


#include <MeSmartServo.h>
#include <SoftwareSerial.h>
#include "Robot.h"

// How many robots should be controlled
#define ROBOTS_NUM 2
// How many caracters can be read from the serial
#define INPUT_SIZE 256

// Defines the pins of the serial interfaces
#define UART1_TX 18
#define UART1_RX 19
#define UART2_TX 16
#define UART2_RX 17
#define UART3_TX 14
#define UART3_RX 15

/**
 * @brief Parses the init command
 * 
 * @param token The input token
 * @param output The array where the init angles should be stored
 * @return int The number of servos
 */
int rob_parse_init(const char* token, float output[]);

/**
 * @brief Parses the break command
 * 
 * @param token The input token
 * @return true The robot should break
 * @return false The robot should release
 */
bool rob_parse_break(const char* token);

/**
 * @brief Parses the move command
 * 
 * @param token The input token
 * @return RobotInstruction The read move command
 */
RobotInstruction rob_parse_move(const char* token);

/**
 * @brief Parses the moveAdv command
 * 
 * @param token The input token
 * @return RobotInstruction The read move command
 */
RobotInstruction rob_parse_moveAdv(const char* token);

/**
 * @brief Prints the move command
 * 
 * @param id of the robot
 * @param cmd The move command
 * @param num_servos The number of servos the robot has
 */
void rob_print_move(int id, RobotInstruction cmd, int num_servos);

/**
 * @brief Prints the current angles
 * 
 * @param id of the robot
 * @param angles the current angles
 * @param num the number of servos the robot has
 */
void rob_print_angles(int id, float angles[], int num);

/**
 * @brief Prints the status of the robot
 * 
 * @param id of the robot
 * @param running at least one servo is moving
 * @param disabled the robot is disabled
 * @param connected the servos are connected
 * @param num_servos the number of servos the robot has
 */
void rob_print_status(int id, bool running, bool disabled, bool connected, int num_servos);

/**
 * @brief Wrapper function for all other parsing functions. Parses all robot related commands.
 * 
 * @param token The input token
 */
void rob_parse(const char* token);

/**
 * @brief Setup function that initialises the robots
 * 
 */
void setup();

/**
 * @brief Loop function that reads from serial, prints to serial and controls robots.
 * 
 */
void loop();

#endif //MMR2_MECH_ARDUINO_H