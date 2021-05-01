#ifndef ROBOT_H
#define ROBOT_H

#include "MeSmartServo.h"

#define BUFFER_LEN 20
#define ANGLE_TOLLERANCE 3

/** 
 * @struct DriveInstruction
 * @brief Struct which contains the drive instruction (absolute angle and speed) of a motor.
 */
struct DriveInstruction
{
    int angle; ///< Absolute angle of the motor [degrees].
    int speed=-1; ///< Speed of the motor [rpm].
};

/** 
 * @struct RobotInstruction
 * @brief Struct which contains the move commands for all three axes.
 */
struct RobotInstruction
{
    DriveInstruction motor1; ///< Drive command of the first motor
    DriveInstruction motor2; ///< Drive command of the second motor
    DriveInstruction motor3; ///< Drive command of the third motor
    bool enabled = false;
};

class Robot{
    private:
        int id;
        int num_servos;
        MeSmartServo *servos;
        RobotInstruction move_buffer[BUFFER_LEN];
        int write_buffer;
        int read_buffer;
        bool moving;
    public:
        Robot(int id, int port, int num_servos);
        ~Robot();
        void increase_write_buffer();
        void increase_read_buffer();
        bool cmdAvailable();
        bool newCmd(RobotInstruction cmd);
        bool checkCmd();
        bool checkServo(int id, int angle);
        bool checkAllServo(RobotInstruction cmd);
        bool driveServo(int id, DriveInstruction cmd);
        bool driveAllServo(RobotInstruction cmd);
        void finishCurrentRobotInstruction();
        // bool start();
        DriveInstruction getDriveInstruction(int id, RobotInstruction cmd);
        DriveInstruction getCurrentDriveInstruction(int id);
        RobotInstruction getCurrentRobotInstruction();

        MeSmartServo* getServos();
};

#endif // ROBOT_H