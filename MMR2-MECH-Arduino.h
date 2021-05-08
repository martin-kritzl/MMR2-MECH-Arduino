#include <MeSmartServo.h>
#include <SoftwareSerial.h>
#include "Robot.h"

#define ROBOTS_NUM 2
#define SERVO_NUM_1 2
#define SERVO_NUM_2 1
#define INPUT_SIZE 50

#define UART1_TX 18
#define UART1_RX 19
#define UART2_TX 16
#define UART2_RX 17
#define UART3_TX 14
#define UART3_RX 15

int rob_parse_init(const char* token, float output[]);
RobotInstruction rob_parse_move(const char* token);
RobotInstruction rob_parse_moveAdv(const char* token);
void rob_print_move(int id, RobotInstruction cmd, int num_servos);
void rob_print_angles(int id, float angles[], int num);
void rob_print_status(int id, bool running);
void rob_parse(const char* token);
void setup();
void loop();