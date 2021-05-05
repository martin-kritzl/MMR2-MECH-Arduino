#ifndef ROBOT_H
#define ROBOT_H

#include "MeSmartServo.h"

#define MAX_NUM_SERVOS 3
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
    DriveInstruction servo[MAX_NUM_SERVOS];
    bool exact = false;
    bool speed_smooth = false;
    bool synchronize = false;
    bool enabled = false;
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
        bool moving;
        float init_angle[MAX_NUM_SERVOS];
    public:
        Robot(int id, int port);
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
        void setInitAngles(float init_angles[]);
        float getAngle(int id);
        void getAngles(float angles[]);
        bool isRunning();
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