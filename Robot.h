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

// The Auriga.h file is not neccessary when defining the MePort_Sig array
#include "MeSmartServo.h"

// The maximum servos per robot. When increasing value ram consumption increases
#define MAX_NUM_SERVOS 3
// The buffer length for move commands. Neccessary for smooth linear movements
#define BUFFER_LEN 20
// How often the same angle can be meassured before a collision is detected
#define COLLISION_MAX_COUNT 300

// The angle tolerance for points in between (for linear movement)
#define ANGLE_TOLERANCE 7
// The angle tolerance for normal endpoints
#define ANGLE_TOLERANCE_EXACT 2
// The minimum speed the robot should move
#define SPEED_MIN 1
// The speed that should be incremented each loop, when speed_smooth is activated
#define SPEED_INCREMENT 0.8

// Defines the normal operating voltage of the servo, to check if the servos is operating
#define MIN_VOLTAGE 6.0
#define MAX_VOLTAGE 12.6

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
 * @brief Struct which contains the move commands for all axes and additional information.
 */
struct RobotInstruction
{
    DriveInstruction servo[MAX_NUM_SERVOS]; ///< The move commands of all axis
    bool exact = false; ///< If the point should be ecactly reached
    bool speed_smooth = false; ///< If the speed should be slowly increased at acceleration
    bool synchronize = false; ///< If the axis should be synchronised (the speed should be calculated)
    bool enabled = false; ///< If the command is valid (used for internal reasons)
    bool collision = false; ///< If the command caused a collision
    unsigned int delay = 0; ///< If a delay should be applied after the movement
};

class Robot{
    private:
        int id; ///< The id of the robot
        int num_servos; ///< The number of axis the robot has
        MeSmartServo *servos; ///< The smart-servo object (can access all servos)
        RobotInstruction move_buffer[BUFFER_LEN]; ///< The buffer where move commands are saved
        float last_speed[MAX_NUM_SERVOS]; ///< The last speed per servo
        int write_buffer; ///< The index in the move_buffer where the next command should be stored
        int read_buffer; ///< The index in the move_buffer where the next command should be executed
        bool started; ///< If the robot is started (Automatically at startup, not started when collision)
        bool moving_servos[MAX_NUM_SERVOS]; ///< What servo is at the moment running
        float init_angle[MAX_NUM_SERVOS]; ///< The initial angle of each servo
        float last_angles[MAX_NUM_SERVOS]; ///< The last angles of each servo
        int count_same_angles; ///< Counter that increases when angle of servo is the same as before (used for collision detection)
        unsigned long sleep_until; ///< Time that specifies when the delay of move command is over (ms)
        bool cur_break_status; ///< If the breaks are active at the moment
        DriveInstruction last_send_inst[MAX_NUM_SERVOS]; ///< The last send command directly to the servos. Used to decrease communication to servo.
    public:
        /**
         * @brief Construct a new Robot object and initializes the servos
         * 
         * @param id of the robot
         * @param port of the robot
         */
        Robot(int id, int port);

        /**
         * @brief Destroy the Robot object. Free the servos
         * 
         */
        ~Robot();

        /**
         * @brief Increases the write index of the move buffer for new commands
         * 
         */
        void increase_write_buffer();

        /**
         * @brief Increases the read index of the move buffer for next executed commands
         * 
         */
        void increase_read_buffer();

        /**
         * @brief Checks if the move buffer has commands left.
         * 
         * @return true If there are still commands
         * @return false If the buffer is empty
         */
        bool cmdAvailable();

        /**
         * @brief Put a new move command in the buffer
         * 
         * @param cmd The move instruction
         * @return true Successful operation
         * @return false When the buffer is already full
         */
        bool newCmd(RobotInstruction cmd);

        /**
         * @brief Move the robot in home position
         * 
         * @return true Successful operation
         * @return false When the buffer is already full
         */
        bool home();

        /**
         * @brief Wrapper of cmdAvailable that sets the breaks when neccessary
         * 
         * @return true If there are still commands
         * @return false If the buffer is empty
         */
        bool checkCmd();

        /**
         * @brief Checks if the servo reached it's endposition
         * 
         * @param id of the servo
         * @param angle the absolut angle
         * @param exact if the point should be exactly reached
         * @return true If the point is reached
         * @return false If the point is not reached
         */
        bool checkServo(int id, int angle, bool exact);

        /**
         * @brief Wrapper for checkServo that checks all servos
         * 
         * @param cmd The move instruction
         * @return true If all servos reached their end positions
         * @return false When at least one servo is still moving
         */
        bool checkAllServo(RobotInstruction cmd);

        /**
         * @brief Sends the move command directly to the servo
         * 
         * @param id of the servo
         * @param cmd that should be send
         * @return true Successful operation
         * @return false Servo returns error
         */
        bool driveServo(int id, DriveInstruction cmd);

        /**
         * @brief Wrapper for driveServo that sends commands to all servos
         * 
         * @param cmd The move command that should be send
         * @return true Successful operation
         * @return false At least one servo returns error
         */
        bool driveAllServo(RobotInstruction cmd);

        /**
         * @brief Is exectued when a movement is finished and prepares for next movement
         * 
         * @return RobotInstruction the finished move command
         */
        RobotInstruction finishCurrentRobotInstruction();

        /**
         * @brief The main method of the class that should be called in a loop.
         * 
         * @return RobotInstruction The current movement instruction. 
         * 
         * @note RobotInstruction.enabled is true when point is reached.
         *       RobotInstruction.collision is true when a collision is detected.
         */
        RobotInstruction cmdFinished();

        /**
         * @brief Resets last speeds. Used for speed_smooth
         * 
         */
        void resetSpeeds();

        /**
         * @brief Enables or disables the breaks of the servo
         * 
         * @param break_status True for breaks and false for release
         * @param force True for forcing the command false for checking the actual state before
         * 
         * @note Setting the breaks only means to set the movement to false, because last command 
         * already breaks the servo. Releasing means to disable the force.
         */
        void setBreaks(bool break_status, bool force);

        /**
         * @brief Checks if the servos are connected with the Arduino
         * 
         * @return true Servos are connected
         * @return false Servos not connected
         */
        bool isConnected();

        /**
         * @brief Stop all servos
         * 
         */
        void stopServos();

        /**
         * @brief Enables the servos after disabling them
         * 
         */
        void enableServos();

        /**
         * @brief Disables the servos
         * 
         */
        void disableServos();

        /**
         * @brief Clears all commands stored in the move buffer
         * 
         */
        void clearCmds();

        /**
         * @brief Set the init angles of the robot. When using only teaching and no inverse kinematic, 
         * 0 degrees for all servos is okay.
         * 
         * @param init_angles 
         */
        void setInitAngles(float init_angles[]);

        /**
         * @brief Sets the zero position of the servos again. The robots should be position very exaclty.
         * 
         */
        void calibrate();

        /**
         * @brief Connects the servos with the Arduino. Already done on startup.
         * But can be used after power down.
         * 
         */
        void connectServos();

        /**
         * @brief Returns the current angle of a servo plus the init angle
         * 
         * @param id of the servo
         * @return float current angle of a servo plus the init angle
         */
        float getAngle(int id);

        /**
         * @brief Wrapper for getAngle that returns angles of all servos
         * 
         * @param angles The array that should be filled with the angles
         */
        void getAngles(float angles[]);

        /**
         * @brief Checks if at least one servo is moving
         * 
         * @return true The robot is moving
         * @return false The robot is idle
         */
        bool isMoving();

        /**
         * @brief Checks if the robot is disabled
         * 
         * @return true robot is disabled
         * @return false robot is started
         */
        bool isDisabled();

        /**
         * @brief Checks if the robot has a collision
         * 
         * @return true no collision detected
         * @return false collision detected
         */
        bool checkCollision();

        /**
         * @brief Sets all servos moving
         * 
         */
        void setAllServosMoving();

        /**
         * @brief The speed is slowly increased for smoother movement
         * 
         * @param cmd The move command that should be improved
         * @param cur_speed The speed that is currently executed
         * @return DriveInstruction The move command with modified speed
         */
        DriveInstruction smoothCmd(DriveInstruction cmd, float cur_speed);

        /**
         * @brief The speed of servos is calculated automatically to synchronise axis.
         * The fastest speed is provided by the speed of the first servo
         * 
         * @param cmd The move command
         * @return RobotInstruction The modified move command (only speed modified)
         */
        RobotInstruction synchronizeServos(RobotInstruction cmd);

        /**
         * @brief Returns the move command for only one servo
         * 
         * @param id of the servo
         * @param cmd The whole move command
         * @return DriveInstruction The move command for one servo
         */
        DriveInstruction getDriveInstruction(int id, RobotInstruction cmd);

        /**
         * @brief Returns the current move command for only one servo
         * 
         * @param id of the servo
         * @return DriveInstruction The current move command for one servo
         */
        DriveInstruction getCurrentDriveInstruction(int id);

        /**
         * @brief Returns the current move command for all servos
         * 
         * @return RobotInstruction Move command for all servos
         */
        RobotInstruction getCurrentRobotInstruction();

        /**
         * @brief Sets the number of servos the roboter has
         * 
         * @param num_servos number of servos
         */
        void setNumServos(int num_servos);

        /**
         * @brief Returns the number of servos the roboter has
         * 
         * @return int number of servos
         */
        int getNumServos();

        /**
         * @brief Get the Servos object
         * 
         * @return MeSmartServo* servo object
         */
        MeSmartServo* getServos();
};

#endif // ROBOT_H