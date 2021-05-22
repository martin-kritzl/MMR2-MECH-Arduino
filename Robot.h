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

#ifndef ROBOT_H
#define ROBOT_H

#include "MeSmartServo.h"

#define MAX_NUM_SERVOS 3
#define BUFFER_LEN 20
#define COLLISION_MAX_COUNT 10

#define ANGLE_TOLERANCE 7
#define ANGLE_TOLERANCE_EXACT 2
#define SPEED_MIN 5
#define SPEED_INCREMENT 0.8

/** 
 * @struct DriveInstruction
 * @brief Struct which contains the drive instruction (absolute angle and speed) of a motor.
 */
struct DriveInstruction
{
    int angle; ///< Absolute angle of the motor [degrees].
    float speed=-1.0; ///< Speed of the motor [rpm].
};

/** 
 * @struct RobotInstruction
 * @brief Struct which contains the move commands for all three axes.
 */
struct RobotInstruction
{
    DriveInstruction servo[MAX_NUM_SERVOS];
    bool exact = false;
    bool speed_smooth = false;
    bool synchronize = false;
    bool enabled = false;
    unsigned int delay = 0;
};

class Robot{
    private:
        int id;
        int num_servos;
        MeSmartServo *servos;
        RobotInstruction move_buffer[BUFFER_LEN];
        float last_speed[MAX_NUM_SERVOS];
        int write_buffer;
        int read_buffer;
        bool started;
        bool moving_servos[MAX_NUM_SERVOS];
        float init_angle[MAX_NUM_SERVOS];
        float last_angles[MAX_NUM_SERVOS];
        int count_same_angles;
    public:
        Robot(int id, int port);
        ~Robot();
        void increase_write_buffer();
        void increase_read_buffer();
        bool cmdAvailable();
        bool newCmd(RobotInstruction cmd);
        bool home();
        bool checkCmd();
        bool checkServo(int id, int angle, bool exact);
        bool checkAllServo(RobotInstruction cmd);
        bool driveServo(int id, DriveInstruction cmd);
        bool driveAllServo(RobotInstruction cmd);
        RobotInstruction finishCurrentRobotInstruction();
        RobotInstruction cmdFinished();
        void resetSpeeds();
        void stopServos();
        void enableServos();
        void disableServos();
        void clearCmds();
        void setInitAngles(float init_angles[]);
        float getAngle(int id);
        void getAngles(float angles[]);
        bool isMoving();
        bool isDisabled();
        bool checkCollision();
        void setAllServosMoving();
        DriveInstruction smoothCmd(DriveInstruction cmd, float cur_speed);
        RobotInstruction synchronizeServos(RobotInstruction cmd);
        // bool start();
        DriveInstruction getDriveInstruction(int id, RobotInstruction cmd);
        DriveInstruction getCurrentDriveInstruction(int id);
        RobotInstruction getCurrentRobotInstruction();

        void setNumServos(int num_servos);
        int getNumServos();

        MeSmartServo* getServos();
};

#endif // ROBOT_H