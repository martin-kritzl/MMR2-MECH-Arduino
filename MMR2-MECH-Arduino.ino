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

MeSmartServo mysmartservo(5);
DriveInstruction move_buffer[BUFFER_LEN];

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
    mysmartservo.begin(115200);         // set the data rate for the Smart Servos
    delay(5);
    Serial.begin(115200);               // set the data rate for the SoftwareSerial port
    mysmartservo.assignDevIdRequest();  // distribution device ID number to the smart servo link.
    delay(50);                          // must delay over 50ms
}

void loop() {
    //Important commands:
    //mysmartservo.setInitAngle(<id>); //This function used to get the smart servo's angle.
    //mysmartservo.setZero(<id>); //set smart servo current angle zero postion.
    //mysmartservo.move(<id>,<angle [degrees]>,<speed [rpm]>); //smart servo moves relativ angle.
    //mysmartservo.moveTo(<id>,<angle [degrees]>,<speed [rpm]>); //smart servo moves to the absolute angle.
    //mysmartservo.setBreak(<id>;<release [true,false]>); //set smart servo break status.
    //mysmartservo.getAngleRequest(<id>); //This function used to get the smart servo's angle.

    mysmartservo.move(1, 20, 20);
    Serial.print("Voltage: ");
    Serial.println(mysmartservo.getVoltageRequest(1));
}