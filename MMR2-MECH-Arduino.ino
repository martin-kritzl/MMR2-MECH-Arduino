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

#include "MMR2-MECH-Arduino.h"

/**
 *  //Important commands:
 *  //mysmartservo.setInitAngle(<id>); //This function used to get the smart servo's angle.
 *  //mysmartservo.setZero(<id>); //set smart servo current angle zero postion.
 *  //mysmartservo.move(<id>,<angle [degrees]>,<speed [rpm]>); //smart servo moves relativ angle.
 *  //mysmartservo.moveTo(<id>,<angle [degrees]>,<speed [rpm]>); //smart servo moves to the absolute angle.
 *  //mysmartservo.setBreak(<id>;<release [true,false]>); //set smart servo break status.
 *  //mysmartservo.getAngleRequest(<id>); //This function used to get the smart servo's angle.
 */

unsigned long cur_time;

MePort_Sig mePort[17] =
{
    { NC, NC }, {   5,   4 }, {   3,   2 }, {   7,   6 }, {   9,   8 }, 
    { UART2_TX, UART2_RX }, { UART1_TX, UART1_RX }, {  A9, A14 }, {  A8, A13 }, {  A7, A12 }, 
    //             LIGHT2        LIGHT1        TEMP          SOUND
    { A6,A11 }, {  NC,  A2 }, {  NC,  A3 }, {  NC,  A0 }, {  NC,  A1 },
    { UART3_TX, UART3_RX }, { NC, NC },
};

Robot* robots[ROBOTS_NUM];

int rob_parse_init(const char* token, float output[]) {
    int i = 0;
    while (token != NULL)
    {
        switch (i) {
            case 0: break;
            case 1:
                output[0] = atof(token);
                break;
            case 2:
                output[1] = atof(token);
                break;
            case 3:
                output[2] = atof(token);
                break;
        }
        ++i;
        token = strtok(NULL, ";");
    }
    return i-1;
}

bool rob_parse_break(const char* token) {
    int i = 0;
    bool status = true;
    while (token != NULL)
    {
        switch (i) {
            case 0: break;
            case 1:
                status = (!strncasecmp(token, "true", 4) ? true : false);
                break;
        }
        ++i;
        token = strtok(NULL, ";");
    }
    return status;
}

RobotInstruction rob_parse_move(const char* token) {
    RobotInstruction result;
    result.enabled = true;

    result.exact = true;
    result.speed_smooth = false;
    result.synchronize = false;
    
    int i = 0;                        // counter for number of tokens
    while (token != NULL)             // stop if the tokenizer returns NULL, then the string is over
    {
        switch (i) {
            case 0: break;
            case 1:
                result.servo[0].angle = atoi(token);
                break;
            case 2:
                result.servo[0].speed = atoi(token);
                break;
            case 3:
                result.servo[1].angle = atoi(token);
                break;
            case 4:
                result.servo[1].speed = atoi(token);
                break;
            case 5:
                result.servo[2].angle = atoi(token);
                break;
            case 6:
                result.servo[2].speed = atoi(token);
                break;
            default:
                result.enabled = false; // if the number of is different (larger) then the string could not be parsed correctly
                break;
        }

        ++i;

        if (result.enabled == false) {
            return result;
        }

        token = strtok(NULL, ";");
    }
    return result;
}

RobotInstruction rob_parse_moveAdv(const char* token) {
    RobotInstruction result;
    result.enabled = true;
    
    int i = 0;                        // counter for number of tokens
    while (token != NULL)             // stop if the tokenizer returns NULL, then the string is over
    {
        switch (i) {
            case 0: break;
            case 1:
                result.exact = (!strncasecmp(token, "true", 4) ? true : false);
                break;
            case 2:
                result.speed_smooth = (!strncasecmp(token, "true", 4) ? true : false);
                break;
            case 3:
                result.synchronize = (!strncasecmp(token, "true", 4) ? true : false);
                break;
            case 4:
                result.delay = atoi(token);
                break;
            case 5:
                result.servo[0].angle = atoi(token);
                break;
            case 6:
                result.servo[0].speed = atoi(token);
                break;
            case 7:
                result.servo[1].angle = atoi(token);
                break;
            case 8:
                result.servo[1].speed = atoi(token);
                break;
            case 9:
                result.servo[2].angle = atoi(token);
                break;
            case 10:
                result.servo[2].speed = atoi(token);
                break;
            default:
                result.enabled = false; // if the number of is different (larger) then the string could not be parsed correctly
                break;
        }

        ++i;

        if (result.enabled == false) {
            return result;
        }

        token = strtok(NULL, ";");
    }
    return result;
}

void rob_print_move(int id, RobotInstruction cmd, int num_servos) {
    Serial.print(F("rob;"));Serial.print(id);Serial.print(F(";async;"));
    Serial.print((cmd.exact) ? F("true;") : F("false;"));
    Serial.print((cmd.speed_smooth) ? F("true;") : F("false;"));
    Serial.print((cmd.synchronize) ? F("true;") : F("false;"));
    Serial.print(cmd.delay);Serial.print(";");
    for (int i = 0; i < num_servos;i++) {
        Serial.print(cmd.servo[i].angle);Serial.print(";");Serial.print(cmd.servo[i].speed);Serial.print(";");
    }
    Serial.println("");
}

void rob_print_angles(int id, float angles[], int num) {
    Serial.print(F("rob;"));Serial.print(id);Serial.print(F(";angles;"));
    for (int i = 0; i < num; i++) {
        Serial.print(angles[i]);Serial.print(";");
    }
    Serial.println("");
}

void rob_print_status(int id, bool running, bool disabled, bool connected, int num_servos) {
    Serial.print(F("rob;"));Serial.print(id);Serial.print(F(";status;"));
    if (num_servos == 0) {
        Serial.println(F("uninitialized;"));
    } else if (!connected) {
        Serial.println(F("disconnected;"));
    } else if (disabled) {
        Serial.println(F("disabled;"));
    } else if (running) {
        Serial.println(F("moving;"));
    } else {
        Serial.println(F("idle;"));
    }
}

void rob_parse(const char* token) {
    int i = 0;                        // counter for number of tokens
    int robot_index;
    while (token != NULL)             // stop if the tokenizer returns NULL, then the string is over
    {
        if (i==0) {
            robot_index = atoi(token)-1;
            if (robot_index == -1 || robot_index >= ROBOTS_NUM) {
                Serial.println(F("ERROR: Robot id is not correct"));
                break;
            }
        } else if (i==1) {
            if (!strncasecmp(token, "moveAdv", 7)) {
                RobotInstruction cmd = rob_parse_moveAdv(token); // parse the incoming command
                if (cmd.enabled == true) {
                    robots[robot_index]->newCmd(cmd);
                    Serial.print(F("rob;"));Serial.print(robot_index+1);Serial.println(F(";moveAdv"));
                } else {
                    Serial.print(F("DEBUG: Wrong input"));
                }
            }
            else if (!strncasecmp(token, "move", 4)) {
                RobotInstruction cmd = rob_parse_move(token); // parse the incoming command
                if (cmd.enabled == true) {
                    robots[robot_index]->newCmd(cmd);
                    Serial.print(F("rob;"));Serial.print(robot_index+1);Serial.println(F(";move"));
                } else {
                    Serial.print(F("DEBUG: Wrong input"));
                }
            }
            else if (!strncasecmp(token, "init", 4)) {
                float init_angles[MAX_NUM_SERVOS];
                int num_servos = rob_parse_init(token, init_angles);
                robots[robot_index]->setNumServos(num_servos);
                robots[robot_index]->setInitAngles(init_angles);
                Serial.print(F("rob;"));Serial.print(robot_index+1);Serial.println(F(";init"));
            }
            else if (!strncasecmp(token, "stop", 4)) {
                robots[robot_index]->disableServos();
                Serial.print(F("rob;"));Serial.print(robot_index+1);Serial.println(F(";stop"));
            }
            else if (!strncasecmp(token, "start", 4)) {
                robots[robot_index]->enableServos();
                Serial.print(F("rob;"));Serial.print(robot_index+1);Serial.println(F(";start"));
            }
            else if (!strncasecmp(token, "angles", 6)) {
                float angles[MAX_NUM_SERVOS];
                robots[robot_index]->getAngles(angles);
                rob_print_angles(robot_index+1, angles, robots[robot_index]->getNumServos());
            }
            else if (!strncasecmp(token, "status", 6)) {
                rob_print_status(robot_index+1, robots[robot_index]->isMoving(), robots[robot_index]->isDisabled(), 
                robots[robot_index]->isConnected(), robots[robot_index]->getNumServos());
            }
            else if (!strncasecmp(token, "home", 4)) {
                robots[robot_index]->home();
                Serial.print(F("rob;"));Serial.print(robot_index+1);Serial.println(F(";home"));
            }
            else if (!strncasecmp(token, "clear", 5)) {
                robots[robot_index]->clearCmds();
                Serial.print(F("rob;"));Serial.print(robot_index+1);Serial.println(F(";clear"));
            }
            else if (!strncasecmp(token, "calibrate", 9)) {
                robots[robot_index]->calibrate();
                Serial.print(F("rob;"));Serial.print(robot_index+1);Serial.println(F(";calibrate"));
            }
            else if (!strncasecmp(token, "break", 5)) {
                bool break_status = rob_parse_break(token);
                robots[robot_index]->setBreaks(break_status,true);
                Serial.print(F("rob;"));Serial.print(robot_index+1);Serial.println(F(";break"));
            } else {
                Serial.println(F("ERROR: Command not found"));
            }
        } else {
            break;
        }
        ++i;
        token = strtok(NULL, ";");
    }
}

void setup() {
    Serial.begin(115200);               // set the data rate for the SoftwareSerial port
    delay(50);                          // must delay over 50ms

    robots[0] = new Robot(1, 5);
    robots[1] = new Robot(2, 15);
    
    // Ansonsten stoppen die Servos zu beginn und fahren dann erst an
    delay(1000);

    Serial.print(F("INFO: "));Serial.print(ROBOTS_NUM);Serial.println(F(" robots are ready. Make sure to init first."));
}

void print_time() {
    unsigned long time = millis();
    Serial.println(time-cur_time);
    if (time-cur_time > 10) {
        Serial.println(F("TIME: long"));
    }
    cur_time = time;
}

void loop() { 
    while (Serial.available()) {
        int robot_index = -1;
        char serial_in[INPUT_SIZE];
        byte serial_size = Serial.readBytesUntil('\n',serial_in, INPUT_SIZE);
        serial_in[serial_size-1] = 0; // add 0-terminator to end of string
        char *token = strtok(serial_in, ";");
        if (token != NULL)
        {
            if (!strncasecmp(token, "rob", 3)) {
                token = strtok(NULL, ";");
                rob_parse(token);
            } else {
                Serial.println(F("ERROR: Type not found"));
            }
        }
    }

    for (int i = 0; i < ROBOTS_NUM; i++) {
        RobotInstruction actCmd = robots[i]->cmdFinished();
        if (actCmd.enabled == true) {
            rob_print_move(i+1, actCmd, robots[i]->getNumServos());
        }
        if (actCmd.collision) {
            Serial.print(F("err;rob;"));Serial.print(i);Serial.println(F(";collision"));
        }
    }
}