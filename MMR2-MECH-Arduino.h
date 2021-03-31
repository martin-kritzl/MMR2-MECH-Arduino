#include <MSmartServo.h>
#include <SoftwareSerial.h>

#define BUFFER_LEN 20
#define RX_PIN 10
#define TX_PIN 11

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
};