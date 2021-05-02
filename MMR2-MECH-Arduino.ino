#include "MMR2-MECH-Arduino.h"
// #include "MeAuriga.h"

 MePort_Sig mePort[17] =
 {
   { NC, NC }, {   5,   4 }, {   3,   2 }, {   7,   6 }, {   9,   8 }, 
   { UART2_TX, UART2_RX }, { A10, A15 }, {  A9, A14 }, {  A8, A13 }, {  A7, A12 }, 
   //             LIGHT2        LIGHT1        TEMP          SOUND
   { A6,A11 }, {  NC,  A2 }, {  NC,  A3 }, {  NC,  A0 }, {  NC,  A1 },
   { NC, NC }, { NC, NC },
 };

//  MePort_Sig mePort[17] =
//  {
//    {UART1_TX, UART1_RX}, {UART2_TX, UART2_RX}, {UART3_TX, UART3_RX}, 
//    {NC, NC}, {NC, NC}, {NC, NC}, {NC, NC}, {NC, NC}, {NC, NC}, {NC, NC}, 
//    {NC, NC}, {NC, NC}, {NC, NC}, {NC, NC}, {NC, NC}, {NC, NC}, {NC, NC}
//  };

// Robot robot1(1, 5, 1);
Robot* robot1;

/**
 * Interface:
 *      incoming: move;<exact>;<theta1>;<speed1>;<theta2>;<speed2>;<theta3>;<speed3>
 *      result: done;move;<exact>;<theta1>;<speed1>;<theta2>;<speed2>;<theta3>;<speed3>
 * 
 *      incoming: move;home
 *      result: done
 * 
 *      incoming: move;stop
 * 
 *      incoming: read;angles
 *      result: <theta1>;<theta2>;<theta3>
 * 
 *      incoming: read;status
 *      result: [idle,moving]
 * 
 *      incoming: set;init
 *      result: done
 */

void setup() {
    Serial.begin(115200);               // set the data rate for the SoftwareSerial port
    delay(50);                          // must delay over 50ms

    float init_angles[3];
    init_angles[0] = 180;
    init_angles[1] = 0;
    init_angles[2] = 0;
    robot1 = new Robot(1, 5, 1, init_angles);
    Serial.print("Setup");

    // RobotInstruction cmd1;
    // cmd1.enabled=true;
    // cmd1.motor1.angle = 20;
    // cmd1.motor1.speed = 10;
    // cmd1.exact = true;
    
    // RobotInstruction cmd2;
    // cmd2.enabled=true;
    // cmd2.motor1.angle = -100;
    // cmd2.motor1.speed = 30;

    // RobotInstruction cmd3;
    // cmd3.enabled=true;
    // cmd3.motor1.angle = -150;
    // cmd3.motor1.speed = 10;
    // cmd3.exact = true;

    // robot1->newCmd(cmd1);
    // robot1->newCmd(cmd2);

    // robot1->newCmd(cmd1);
    // robot1->newCmd(cmd2);
    // robot1->newCmd(cmd3);

    // robot1->newCmd(cmd1);
    // robot1->newCmd(cmd2);
    // robot1->newCmd(cmd3);

    RobotInstruction cmd1;
    cmd1.enabled=true;
    cmd1.motor1.angle = 181;
    cmd1.motor1.speed = 9;
    cmd1.motor2.angle = -3;
    cmd1.motor2.speed = 7;
    cmd1.exact = false;
    robot1->newCmd(cmd1);

    RobotInstruction cmd2;
    cmd2.enabled=true;
    cmd2.motor1.angle = 181;
    cmd2.motor1.speed = 50;
    cmd2.motor2.angle = -4;
    cmd2.motor2.speed = 1;
    cmd2.exact = false;
    robot1->newCmd(cmd2);

    RobotInstruction cmd3;
    cmd3.enabled=true;
    cmd3.motor1.angle = 180;
    cmd3.motor1.speed = 50;
    cmd3.motor2.angle = -4;
    cmd3.motor2.speed = 1;
    cmd3.exact = false;
    robot1->newCmd(cmd3);

    RobotInstruction cmd4;
    cmd4.enabled=true;
    cmd4.motor1.angle = 179;
    cmd4.motor1.speed = 50;
    cmd4.motor2.angle = -5;
    cmd4.motor2.speed = 5;
    cmd4.exact = false;
    robot1->newCmd(cmd4);

    RobotInstruction cmd5;
    cmd5.enabled=true;
    cmd5.motor1.angle = 178;
    cmd5.motor1.speed = 50;
    cmd5.motor2.angle = -6;
    cmd5.motor2.speed = 1;
    cmd5.exact = false;
    robot1->newCmd(cmd5);

    RobotInstruction cmd6;
    cmd6.enabled=true;
    cmd6.motor1.angle = 177;
    cmd6.motor1.speed = 50;
    cmd6.motor2.angle = -6;
    cmd6.motor2.speed = 5;
    cmd6.exact = false;
    robot1->newCmd(cmd6);

    RobotInstruction cmd7;
    cmd7.enabled=true;
    cmd7.motor1.angle = 177;
    cmd7.motor1.speed = 50;
    cmd7.motor2.angle = -7;
    cmd7.motor2.speed = 8;
    cmd7.exact = false;
    robot1->newCmd(cmd7);

    RobotInstruction cmd8;
    cmd8.enabled=true;
    cmd8.motor1.angle = 176;
    cmd8.motor1.speed = 50;
    cmd8.motor2.angle = -8;
    cmd8.motor2.speed = 37;
    cmd8.exact = false;
    robot1->newCmd(cmd8);

    RobotInstruction cmd9;
    cmd9.enabled=true;
    cmd9.motor1.angle = 175;
    cmd9.motor1.speed = 50;
    cmd9.motor2.angle = -8;
    cmd9.motor2.speed = 2;
    cmd9.exact = false;
    robot1->newCmd(cmd9);
}

void loop() {
    //Important commands:
    //mysmartservo.setInitAngle(<id>); //This function used to get the smart servo's angle.
    //mysmartservo.setZero(<id>); //set smart servo current angle zero postion.
    //mysmartservo.move(<id>,<angle [degrees]>,<speed [rpm]>); //smart servo moves relativ angle.
    //mysmartservo.moveTo(<id>,<angle [degrees]>,<speed [rpm]>); //smart servo moves to the absolute angle.
    //mysmartservo.setBreak(<id>;<release [true,false]>); //set smart servo break status.
    //mysmartservo.getAngleRequest(<id>); //This function used to get the smart servo's angle.

    robot1->checkCmd();
    Serial.print("Voltage: ");
    Serial.println(robot1->getServos()->getVoltageRequest(1));
    Serial.print("Target angle: ");
    Serial.println(robot1->getCurrentDriveInstruction(1).angle);
}