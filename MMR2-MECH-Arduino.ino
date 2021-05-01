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
 *      incoming: move;<theta1>;<speed1>;<theta2>;<speed2>;<theta3>;<speed3>
 *      result: done;move;<theta1>;<speed1>;<theta2>;<speed2>;<theta3>;<speed3>
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

    robot1 = new Robot(1, 5, 1);
    Serial.print("Setup");

    RobotInstruction cmd1;
    cmd1.enabled=true;
    cmd1.motor1.angle = 20;
    cmd1.motor1.speed = 10;
    robot1->newCmd(cmd1);

    RobotInstruction cmd2;
    cmd2.enabled=true;
    cmd2.motor1.angle = -30;
    cmd2.motor1.speed = 30;
    robot1->newCmd(cmd2);

    robot1->newCmd(cmd1);
    robot1->newCmd(cmd2);

    robot1->newCmd(cmd1);
    robot1->newCmd(cmd2);
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