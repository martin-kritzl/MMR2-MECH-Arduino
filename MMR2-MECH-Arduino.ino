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
 *      incoming: <id>;move;<exact>;<theta1>;<speed1>;<theta2>;<speed2>;<theta3>;<speed3>
 *      result: <id>;done;move;<exact>;<theta1>;<speed1>;<theta2>;<speed2>;<theta3>;<speed3>
 * 
 *      1;move;true;142;10;-29;10
 *      1;move;true;180;10;0;10
 * 
 *      incoming: <id>;move;home;speed
 *      result: <id>;done;move;<exact>;<theta1>;<speed1>;<theta2>;<speed2>;<theta3>;<speed3>
 * 
 *      incoming: <id>;move;stop
 * 
 *      incoming: <id>;read;angles
 *      result: <id>;<theta1>;<theta2>;<theta3>
 * 
 *      incoming: <id>;read;status
 *      result: <id>;[idle,moving]
 * 
 *      incoming: <id>;set;init
 *      result: <id>;done
 */

void setup() {
    Serial.begin(115200);               // set the data rate for the SoftwareSerial port
    delay(50);                          // must delay over 50ms

    float init_angles[3];
    init_angles[0] = 180;
    init_angles[1] = 0;
    init_angles[2] = 0;
    robot1 = new Robot(1, 5, ROBOTS_NUM, init_angles);
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
    cmd1.motor1.angle = 180;
    cmd1.motor1.speed = 3;
    cmd1.motor2.angle = -4;
    cmd1.motor2.speed = 3;
    cmd1.exact = false;
    robot1->newCmd(cmd1);

    RobotInstruction cmd2;
    cmd2.enabled=true;
    cmd2.motor1.angle = 177;
    cmd2.motor1.speed = 20;
    cmd2.motor2.angle = -6;
    cmd2.motor2.speed = 0;
    cmd2.exact = false;
    robot1->newCmd(cmd2);

    RobotInstruction cmd3;
    cmd3.enabled=true;
    cmd3.motor1.angle = 175;
    cmd3.motor1.speed = 20;
    cmd3.motor2.angle = -8;
    cmd3.motor2.speed = 0;
    cmd3.exact = false;
    robot1->newCmd(cmd3);

    RobotInstruction cmd4;
    cmd4.enabled=true;
    cmd4.motor1.angle = 173;
    cmd4.motor1.speed = 20;
    cmd4.motor2.angle = -10;
    cmd4.motor2.speed = 5;
    cmd4.exact = false;
    robot1->newCmd(cmd4);

    RobotInstruction cmd5;
    cmd5.enabled=true;
    cmd5.motor1.angle = 170;
    cmd5.motor1.speed = 20;
    cmd5.motor2.angle = -12;
    cmd5.motor2.speed = 1;
    cmd5.exact = false;
    robot1->newCmd(cmd5);

    RobotInstruction cmd6;
    cmd6.enabled=true;
    cmd6.motor1.angle = 168;
    cmd6.motor1.speed = 20;
    cmd6.motor2.angle = -14;
    cmd6.motor2.speed = 6;
    cmd6.exact = false;
    robot1->newCmd(cmd6);

    RobotInstruction cmd7;
    cmd7.enabled=true;
    cmd7.motor1.angle = 166;
    cmd7.motor1.speed = 20;
    cmd7.motor2.angle = -15;
    cmd7.motor2.speed = 10;
    cmd7.exact = false;
    robot1->newCmd(cmd7);

    RobotInstruction cmd8;
    cmd8.enabled=true;
    cmd8.motor1.angle = 163;
    cmd8.motor1.speed = 20;
    cmd8.motor2.angle = -17;
    cmd8.motor2.speed = 0;
    cmd8.exact = false;
    robot1->newCmd(cmd8);

    RobotInstruction cmd9;
    cmd9.enabled=true;
    cmd9.motor1.angle = 161;
    cmd9.motor1.speed = 20;
    cmd9.motor2.angle = -19;
    cmd9.motor2.speed = 2;
    cmd9.exact = false;
    robot1->newCmd(cmd9);

    RobotInstruction cmd10;
    cmd10.enabled=true;
    cmd10.motor1.angle = 159;
    cmd10.motor1.speed = 20;
    cmd10.motor2.angle = -20;
    cmd10.motor2.speed = 13;
    cmd10.exact = false;
    robot1->newCmd(cmd10);

    RobotInstruction cmd11;
    cmd11.enabled=true;
    cmd11.motor1.angle = 157;
    cmd11.motor1.speed = 12;
    cmd11.motor2.angle = -21;
    cmd11.motor2.speed = 14;
    cmd11.exact = false;
    robot1->newCmd(cmd11);

    RobotInstruction cmd12;
    cmd12.enabled=true;
    cmd12.motor1.angle = 154;
    cmd12.motor1.speed = 9;
    cmd12.motor2.angle = -23;
    cmd12.motor2.speed = 20;
    cmd12.exact = false;
    robot1->newCmd(cmd12);

    RobotInstruction cmd13;
    cmd13.enabled=true;
    cmd13.motor1.angle = 152;
    cmd13.motor1.speed = 0;
    cmd13.motor2.angle = -24;
    cmd13.motor2.speed = 0;
    cmd13.exact = false;
    robot1->newCmd(cmd13);

    RobotInstruction cmd14;
    cmd14.enabled=true;
    cmd14.motor1.angle = 150;
    cmd14.motor1.speed = 16;
    cmd14.motor2.angle = -25;
    cmd14.motor2.speed = 2;
    cmd14.exact = false;
    robot1->newCmd(cmd14);

    RobotInstruction cmd15;
    cmd15.enabled=true;
    cmd15.motor1.angle = 148;
    cmd15.motor1.speed = 20;
    cmd15.motor2.angle = -26;
    cmd15.motor2.speed = 0;
    cmd15.exact = false;
    robot1->newCmd(cmd15);

    RobotInstruction cmd16;
    cmd16.enabled=true;
    cmd16.motor1.angle = 146;
    cmd16.motor1.speed = 14;
    cmd16.motor2.angle = -28;
    cmd16.motor2.speed = 2;
    cmd16.exact = false;
    robot1->newCmd(cmd16);

    RobotInstruction cmd17;
    cmd17.enabled=true;
    cmd17.motor1.angle = 144;
    cmd17.motor1.speed = 20;
    cmd17.motor2.angle = -28;
    cmd17.motor2.speed = 0;
    cmd17.exact = false;
    robot1->newCmd(cmd17);

    RobotInstruction cmd18;
    cmd18.enabled=true;
    cmd18.motor1.angle = 142;
    cmd18.motor1.speed = 12;
    cmd18.motor2.angle = -29;
    cmd18.motor2.speed = 2;
    cmd18.exact = true;
    robot1->newCmd(cmd18);

    RobotInstruction cmd19;
    cmd19.enabled=true;
    cmd19.motor1.angle = 180;
    cmd19.motor1.speed = 10;
    cmd19.motor2.angle = 0;
    cmd19.motor2.speed = 10;
    cmd19.exact = true;
    robot1->newCmd(cmd19);
}

RobotInstruction parse_move(char input[])
{
    RobotInstruction result;
    result.enabled = true;

    int i = 0;                        // counter for number of tokens
    char *token = strtok(input, ";"); // split the string into tokens
    while (token != NULL)             // stop if the tokenizer returns NULL, then the string is over
    {
        switch (i) {
            case 0: break;
            case 1: break;
            case 2:
                result.exact = (!strncasecmp(token, "true", 5) ? true : false);
                break;
            case 3:
                result.motor1.angle = atoi(token);
                break;
            case 4:
                result.motor1.speed = atoi(token);
                break;
            case 5:
                result.motor2.angle = atoi(token);
                break;
            case 6:
                result.motor2.speed = atoi(token);
                break;
            case 7:
                result.motor3.angle = atoi(token);
                break;
            case 8:
                result.motor3.speed = atoi(token);
                break;
            default:
                result.enabled = false; // if the number of is different (larger) then the string could not be parsed correctly
                break;
        }

        ++i;

        if (result.enabled == false) {
            return result;
        }

        token = strtok(NULL, ";");
    }
    return result;
}

void loop() {
    //Important commands:
    //mysmartservo.setInitAngle(<id>); //This function used to get the smart servo's angle.
    //mysmartservo.setZero(<id>); //set smart servo current angle zero postion.
    //mysmartservo.move(<id>,<angle [degrees]>,<speed [rpm]>); //smart servo moves relativ angle.
    //mysmartservo.moveTo(<id>,<angle [degrees]>,<speed [rpm]>); //smart servo moves to the absolute angle.
    //mysmartservo.setBreak(<id>;<release [true,false]>); //set smart servo break status.
    //mysmartservo.getAngleRequest(<id>); //This function used to get the smart servo's angle.

    if (Serial.available()) {
        char serial_in[INPUT_SIZE];
        byte serial_size = Serial.readBytes(serial_in, INPUT_SIZE);
        serial_in[serial_size] = 0; // add 0-terminator to end of string
        Serial.print("Command recevied: ");
        Serial.println(serial_in);
        Serial.println("Parsing...");
        RobotInstruction cmd = parse_move(serial_in); // parse the incoming command
        if (cmd.enabled == true) {
            robot1->newCmd(cmd);
        } else {
            Serial.print("Wrong input");
        }
        
    }

    robot1->checkCmd();
    Serial.print("Voltage: ");
    Serial.println(robot1->getServos()->getVoltageRequest(1));
    Serial.print("Target angle: ");
    Serial.println(robot1->getCurrentDriveInstruction(1).angle);
}