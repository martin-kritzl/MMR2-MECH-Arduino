#ifndef ROBOT_H
#define ROBOT_H

#include "MeSmartServo.h"

#define BUFFER_LEN 20
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
    DriveInstruction servo[3];
    bool exact = false;
    bool enabled = false;
};

class Robot{
    private:
        int id;
        int num_servos;
        MeSmartServo *servos;
        RobotInstruction move_buffer[BUFFER_LEN];
        float last_speed[3];
        int write_buffer;
        int read_buffer;
        bool moving;
        float init_angle[3];
    public:
        Robot(int id, int port, int num_servos);
        ~Robot();
        void increase_write_buffer();
        void increase_read_buffer();
        bool cmdAvailable();
        bool newCmd(RobotInstruction cmd);
        bool checkCmd();
        bool checkServo(int id, int angle, bool exact);
        bool checkAllServo(RobotInstruction cmd);
        bool driveServo(int id, DriveInstruction cmd);
        bool driveAllServo(RobotInstruction cmd);
        RobotInstruction finishCurrentRobotInstruction();
        RobotInstruction cmdFinished();
        void resetSpeeds();
        void stopServos();
        void setInitAngles(float init_angles[3]);
        float getAngle(int id);
        bool isRunning();
        DriveInstruction smoothCmd(DriveInstruction cmd, float cur_speed);
        // bool start();
        DriveInstruction getDriveInstruction(int id, RobotInstruction cmd);
        DriveInstruction getCurrentDriveInstruction(int id);
        RobotInstruction getCurrentRobotInstruction();

        MeSmartServo* getServos();
};

#endif // ROBOT_H