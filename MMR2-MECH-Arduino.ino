#include "MMR2-MECH-Arduino.h"

MSmartServo mysmartservo(RX_PIN,TX_PIN);
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
    //mysmartservo.moveTo(<id>,<angle [degrees]>,<speed [rpm]>); //smart servo moves to the absolute angle.
    //mysmartservo.setBreak(<id>;<release [true,false]>); //set smart servo break status.
    //mysmartservo.getAngleRequest(<id>); //This function used to get the smart servo's angle.
}