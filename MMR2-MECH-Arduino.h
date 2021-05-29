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

#define ROBOTS_NUM 3
#define SERVO_NUM_1 2
#define SERVO_NUM_2 1
#define INPUT_SIZE 50

#define UART1_TX 14
#define UART1_RX 15
#define UART2_TX 16
#define UART2_RX 17
#define UART3_TX 18
#define UART3_RX 19

int rob_parse_init(const char* token, float output[]);
bool rob_parse_break(const char* token);
RobotInstruction rob_parse_move(const char* token);
RobotInstruction rob_parse_moveAdv(const char* token);
void rob_print_move(int id, RobotInstruction cmd, int num_servos);
void rob_print_angles(int id, float angles[], int num);
void rob_print_status(int id, bool running, bool disabled, bool connected, int num_servos);
void rob_parse(const char* token);
void setup();
void loop();

#endif //MMR2_MECH_ARDUINO_H