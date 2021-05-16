#include "MMR2-MECH-Arduino.h"

/**
 *  //Important commands:
 *  //mysmartservo.setInitAngle(<id>); //This function used to get the smart servo's angle.
 *  //mysmartservo.setZero(<id>); //set smart servo current angle zero postion.
 *  //mysmartservo.move(<id>,<angle [degrees]>,<speed [rpm]>); //smart servo moves relativ angle.
 *  //mysmartservo.moveTo(<id>,<angle [degrees]>,<speed [rpm]>); //smart servo moves to the absolute angle.
 *  //mysmartservo.setBreak(<id>;<release [true,false]>); //set smart servo break status.
 *  //mysmartservo.getAngleRequest(<id>); //This function used to get the smart servo's angle.
 */

unsigned long cur_time;

MePort_Sig mePort[17] =
{
    { NC, NC }, {   5,   4 }, {   3,   2 }, {   7,   6 }, {   9,   8 }, 
    { UART2_TX, UART2_RX }, { A10, A15 }, {  A9, A14 }, {  A8, A13 }, {  A7, A12 }, 
    //             LIGHT2        LIGHT1        TEMP          SOUND
    { A6,A11 }, {  NC,  A2 }, {  NC,  A3 }, {  NC,  A0 }, {  NC,  A1 },
    { UART3_TX, UART3_RX }, { NC, NC },
};

Robot* robots[ROBOTS_NUM];

int rob_parse_init(const char* token, float output[]) {
    int i = 0;
    while (token != NULL)
    {
        switch (i) {
            case 0: break;
            case 1:
                output[0] = atof(token);
                break;
            case 2:
                output[1] = atof(token);
                break;
            case 3:
                output[2] = atof(token);
                break;
        }
        ++i;
        token = strtok(NULL, ";");
    }
    return i-1;
}

RobotInstruction rob_parse_move(const char* token) {
    RobotInstruction result;
    result.enabled = true;

    result.exact = true;
    result.speed_smooth = false;
    result.synchronize = false;
    
    int i = 0;                        // counter for number of tokens
    while (token != NULL)             // stop if the tokenizer returns NULL, then the string is over
    {
        switch (i) {
            case 0: break;
            case 1:
                result.servo[0].angle = atoi(token);
                break;
            case 2:
                result.servo[0].speed = atoi(token);
                break;
            case 3:
                result.servo[1].angle = atoi(token);
                break;
            case 4:
                result.servo[1].speed = atoi(token);
                break;
            case 5:
                result.servo[2].angle = atoi(token);
                break;
            case 6:
                result.servo[2].speed = atoi(token);
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

RobotInstruction rob_parse_moveAdv(const char* token) {
    RobotInstruction result;
    result.enabled = true;
    
    int i = 0;                        // counter for number of tokens
    while (token != NULL)             // stop if the tokenizer returns NULL, then the string is over
    {
        switch (i) {
            case 0: break;
            case 1:
                result.exact = (!strncmp(token, "true", 4) ? true : false);
                break;
            case 2:
                result.speed_smooth = (!strncmp(token, "true", 4) ? true : false);
                break;
            case 3:
                result.synchronize = (!strncmp(token, "true", 4) ? true : false);
                break;
            case 4:
                result.delay = atoi(token);
                break;
            case 5:
                result.servo[0].angle = atoi(token);
                break;
            case 6:
                result.servo[0].speed = atoi(token);
                break;
            case 7:
                result.servo[1].angle = atoi(token);
                break;
            case 8:
                result.servo[1].speed = atoi(token);
                break;
            case 9:
                result.servo[2].angle = atoi(token);
                break;
            case 10:
                result.servo[2].speed = atoi(token);
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

void rob_print_move(int id, RobotInstruction cmd, int num_servos) {
    Serial.print("rob;");Serial.print(id);Serial.print(";async;");
    Serial.print((cmd.exact) ? "true;" : "false;");
    Serial.print((cmd.speed_smooth) ? "true;" : "false;");
    Serial.print((cmd.synchronize) ? "true;" : "false;");
    Serial.print(cmd.delay);Serial.print(";");
    for (int i = 0; i < num_servos;i++) {
        Serial.print(cmd.servo[i].angle);Serial.print(";");Serial.print(cmd.servo[i].speed);Serial.print(";");
    }
    Serial.println("");
}

void rob_print_angles(int id, float angles[], int num) {
    Serial.print("rob;");Serial.print(id);Serial.print(";angles;");
    for (int i = 0; i < num; i++) {
        Serial.print(angles[i]);Serial.print(";");
    }
    Serial.println("");
}

void rob_print_status(int id, bool running) {
    Serial.print("rob;");Serial.print(id);Serial.print(";status;");Serial.println((running) ? "running;" : "idle;");
}

void rob_parse(const char* token) {
    int i = 0;                        // counter for number of tokens
    int robot_index;
    while (token != NULL)             // stop if the tokenizer returns NULL, then the string is over
    {
        if (i==0) {
            robot_index = atoi(token)-1;
            if (robot_index == -1 || robot_index >= ROBOTS_NUM) {
                Serial.println("ERROR: Robot id is not correct");
                break;
            }
        } else if (i==1) {
            if (!strncmp(token, "moveAdv", 7)) {
                RobotInstruction cmd = rob_parse_moveAdv(token); // parse the incoming command
                if (cmd.enabled == true) {
                    robots[robot_index]->newCmd(cmd);
                    Serial.print("rob;");Serial.print(robot_index+1);Serial.println(";moveAdv");
                } else {
                    Serial.print("DEBUG: Wrong input");
                }
            }
            else if (!strncmp(token, "move", 4)) {
                RobotInstruction cmd = rob_parse_move(token); // parse the incoming command
                if (cmd.enabled == true) {
                    robots[robot_index]->newCmd(cmd);
                    Serial.print("rob;");Serial.print(robot_index+1);Serial.println(";move");
                } else {
                    Serial.print("DEBUG: Wrong input");
                }
            }
            else if (!strncmp(token, "init", 4)) {
                float init_angles[MAX_NUM_SERVOS];
                int num_servos = rob_parse_init(token, init_angles);
                robots[robot_index]->setNumServos(num_servos);
                robots[robot_index]->setInitAngles(init_angles);
                Serial.print("rob;");Serial.print(robot_index+1);Serial.println(";init");
            }
            else if (!strncmp(token, "stop", 4)) {
                robots[robot_index]->disableServos();
                Serial.print("rob;");Serial.print(robot_index+1);Serial.println(";stop");
            }
            else if (!strncmp(token, "start", 4)) {
                robots[robot_index]->enableServos();
                Serial.print("rob;");Serial.print(robot_index+1);Serial.println(";start");
            }
            else if (!strncmp(token, "angles", 6)) {
                float angles[MAX_NUM_SERVOS];
                robots[robot_index]->getAngles(angles);
                rob_print_angles(robot_index+1, angles, robots[robot_index]->getNumServos());
            }
            else if (!strncmp(token, "status", 6)) {
                rob_print_status(robot_index+1, robots[robot_index]->isRunning());
            }
            else if (!strncmp(token, "home", 4)) {
                robots[robot_index]->home();
            }
            else if (!strncmp(token, "clear", 5)) {
                robots[robot_index]->clearCmds();
                Serial.print("rob;");Serial.print(robot_index+1);Serial.println(";clear");
            }
            else {
                Serial.println("ERROR: Command not found");
            }
        } else {
            break;
        }
        ++i;
        token = strtok(NULL, ";");
    }
}

void setup() {
    Serial.begin(115200);               // set the data rate for the SoftwareSerial port
    delay(50);                          // must delay over 50ms

    robots[0] = new Robot(1, 5);
    robots[1] = new Robot(2, 15);
    
    // Ansonsten stoppen die Servos zu beginn und fahren dann erst an
    delay(1000);

    Serial.print("INFO: ");Serial.print(ROBOTS_NUM);Serial.println(" robots are ready. Make sure to init first.");


    /**
     * 
     * Von hier an kann alles aus der setup Funktion geloescht werden
     * 
     */

    // ##### Testsetup:

    // robots[0]->setNumServos(SERVO_NUM_1);
    // robots[1]->setNumServos(SERVO_NUM_2);

    // float init_angles[MAX_NUM_SERVOS];
    // init_angles[0] = 180;
    // init_angles[1] = 0;
    // init_angles[2] = 0;

    // robots[0]->setInitAngles(init_angles);
    // robots[1]->setInitAngles(init_angles);

    // Serial.println("DEBUG: Setup");

    // // ##### Erster Test: Pick and Place

    // RobotInstruction cmd1;
    // cmd1.enabled=true;
    // cmd1.servo[0].angle = 132;
    // cmd1.servo[0].speed = 20;
    // cmd1.servo[1].angle = 0;
    // cmd1.servo[1].speed = 10;
    // cmd1.exact = true;
    // cmd1.speed_smooth = false;
    // cmd1.synchronize = true;
    // robots[0]->newCmd(cmd1);

    // RobotInstruction cmd2;
    // cmd2.enabled=true;
    // cmd2.servo[0].angle = 138;
    // cmd2.servo[0].speed = 20;
    // cmd2.servo[1].angle = -35;
    // cmd2.servo[1].speed = 10;
    // cmd2.exact = true;
    // cmd2.speed_smooth = false;
    // cmd2.synchronize = true;
    // robots[0]->newCmd(cmd2);

    // RobotInstruction cmd3;
    // cmd3.enabled=true;
    // cmd3.servo[0].angle = 216;
    // cmd3.servo[0].speed = 20;
    // cmd3.servo[1].angle = 35;
    // cmd3.servo[1].speed = 10;
    // cmd3.exact = true;
    // cmd3.speed_smooth = false;
    // cmd3.synchronize = true;
    // robots[0]->newCmd(cmd3);

    // RobotInstruction cmd4;
    // cmd4.enabled=true;
    // cmd4.servo[0].angle = 180;
    // cmd4.servo[0].speed = 20;
    // cmd4.servo[1].angle = 43;
    // cmd4.servo[1].speed = 10;
    // cmd4.exact = true;
    // cmd4.speed_smooth = false;
    // cmd4.synchronize = true;
    // robots[0]->newCmd(cmd4);

    // RobotInstruction cmd5;
    // cmd5.enabled=true;
    // cmd5.servo[0].angle = 180;
    // cmd5.servo[0].speed = 20;
    // cmd5.servo[1].angle = 0;
    // cmd5.servo[1].speed = 10;
    // cmd5.exact = true;
    // cmd5.speed_smooth = false;
    // cmd5.synchronize = true;
    // robots[0]->newCmd(cmd5);

    // ##### Zweiter Test: Linearbewegung

    // RobotInstruction cmd1;
    // cmd1.enabled=true;
    // cmd1.servo[0].angle = 180;
    // cmd1.servo[0].speed = 3;
    // cmd1.servo[1].angle = -4;
    // cmd1.servo[1].speed = 3;
    // cmd1.exact = false;
    // cmd1.speed_smooth = true;
    // robots[0]->newCmd(cmd1);

    // RobotInstruction cmd2;
    // cmd2.enabled=true;
    // cmd2.servo[0].angle = 177;
    // cmd2.servo[0].speed = 20;
    // cmd2.servo[1].angle = -6;
    // cmd2.servo[1].speed = 0;
    // cmd2.exact = false;
    // cmd2.speed_smooth = true;
    // robots[0]->newCmd(cmd2);

    // RobotInstruction cmd3;
    // cmd3.enabled=true;
    // cmd3.servo[0].angle = 175;
    // cmd3.servo[0].speed = 20;
    // cmd3.servo[1].angle = -8;
    // cmd3.servo[1].speed = 0;
    // cmd3.exact = false;
    // cmd3.speed_smooth = true;
    // robots[0]->newCmd(cmd3);

    // RobotInstruction cmd4;
    // cmd4.enabled=true;
    // cmd4.servo[0].angle = 173;
    // cmd4.servo[0].speed = 20;
    // cmd4.servo[1].angle = -10;
    // cmd4.servo[1].speed = 5;
    // cmd4.exact = false;
    // cmd4.speed_smooth = true;
    // robots[0]->newCmd(cmd4);

    // RobotInstruction cmd5;
    // cmd5.enabled=true;
    // cmd5.servo[0].angle = 170;
    // cmd5.servo[0].speed = 20;
    // cmd5.servo[1].angle = -12;
    // cmd5.servo[1].speed = 1;
    // cmd5.exact = false;
    // cmd5.speed_smooth = true;
    // robots[0]->newCmd(cmd5);

    // RobotInstruction cmd6;
    // cmd6.enabled=true;
    // cmd6.servo[0].angle = 168;
    // cmd6.servo[0].speed = 20;
    // cmd6.servo[1].angle = -14;
    // cmd6.servo[1].speed = 6;
    // cmd6.exact = false;
    // cmd6.speed_smooth = true;
    // robots[0]->newCmd(cmd6);

    // RobotInstruction cmd7;
    // cmd7.enabled=true;
    // cmd7.servo[0].angle = 166;
    // cmd7.servo[0].speed = 20;
    // cmd7.servo[1].angle = -15;
    // cmd7.servo[1].speed = 10;
    // cmd7.exact = false;
    // cmd7.speed_smooth = true;
    // robots[0]->newCmd(cmd7);

    // RobotInstruction cmd8;
    // cmd8.enabled=true;
    // cmd8.servo[0].angle = 163;
    // cmd8.servo[0].speed = 20;
    // cmd8.servo[1].angle = -17;
    // cmd8.servo[1].speed = 0;
    // cmd8.exact = false;
    // cmd8.speed_smooth = true;
    // robots[0]->newCmd(cmd8);

    // RobotInstruction cmd9;
    // cmd9.enabled=true;
    // cmd9.servo[0].angle = 161;
    // cmd9.servo[0].speed = 20;
    // cmd9.servo[1].angle = -19;
    // cmd9.servo[1].speed = 2;
    // cmd9.exact = false;
    // cmd9.speed_smooth = true;
    // robots[0]->newCmd(cmd9);

    // RobotInstruction cmd10;
    // cmd10.enabled=true;
    // cmd10.servo[0].angle = 159;
    // cmd10.servo[0].speed = 20;
    // cmd10.servo[1].angle = -20;
    // cmd10.servo[1].speed = 13;
    // cmd10.exact = false;
    // cmd10.speed_smooth = true;
    // robots[0]->newCmd(cmd10);

    // RobotInstruction cmd11;
    // cmd11.enabled=true;
    // cmd11.servo[0].angle = 157;
    // cmd11.servo[0].speed = 12;
    // cmd11.servo[1].angle = -21;
    // cmd11.servo[1].speed = 14;
    // cmd11.exact = false;
    // cmd11.speed_smooth = true;
    // robots[0]->newCmd(cmd11);

    // RobotInstruction cmd12;
    // cmd12.enabled=true;
    // cmd12.servo[0].angle = 154;
    // cmd12.servo[0].speed = 9;
    // cmd12.servo[1].angle = -23;
    // cmd12.servo[1].speed = 20;
    // cmd12.exact = false;
    // cmd12.speed_smooth = true;
    // robots[0]->newCmd(cmd12);

    // RobotInstruction cmd13;
    // cmd13.enabled=true;
    // cmd13.servo[0].angle = 152;
    // cmd13.servo[0].speed = 0;
    // cmd13.servo[1].angle = -24;
    // cmd13.servo[1].speed = 0;
    // cmd13.exact = false;
    // cmd13.speed_smooth = true;
    // robots[0]->newCmd(cmd13);

    // RobotInstruction cmd14;
    // cmd14.enabled=true;
    // cmd14.servo[0].angle = 150;
    // cmd14.servo[0].speed = 16;
    // cmd14.servo[1].angle = -25;
    // cmd14.servo[1].speed = 2;
    // cmd14.exact = false;
    // cmd14.speed_smooth = true;
    // robots[0]->newCmd(cmd14);

    // RobotInstruction cmd15;
    // cmd15.enabled=true;
    // cmd15.servo[0].angle = 148;
    // cmd15.servo[0].speed = 20;
    // cmd15.servo[1].angle = -26;
    // cmd15.servo[1].speed = 0;
    // cmd15.exact = false;
    // cmd15.speed_smooth = true;
    // robots[0]->newCmd(cmd15);

    // RobotInstruction cmd16;
    // cmd16.enabled=true;
    // cmd16.servo[0].angle = 146;
    // cmd16.servo[0].speed = 14;
    // cmd16.servo[1].angle = -28;
    // cmd16.servo[1].speed = 2;
    // cmd16.exact = false;
    // cmd16.speed_smooth = true;
    // robots[0]->newCmd(cmd16);

    // RobotInstruction cmd17;
    // cmd17.enabled=true;
    // cmd17.servo[0].angle = 144;
    // cmd17.servo[0].speed = 20;
    // cmd17.servo[1].angle = -28;
    // cmd17.servo[1].speed = 0;
    // cmd17.exact = false;
    // cmd17.speed_smooth = true;
    // robots[0]->newCmd(cmd17);

    // RobotInstruction cmd18;
    // cmd18.enabled=true;
    // cmd18.servo[0].angle = 142;
    // cmd18.servo[0].speed = 12;
    // cmd18.servo[1].angle = -29;
    // cmd18.servo[1].speed = 2;
    // cmd18.exact = true;
    // cmd18.speed_smooth = true;
    // robots[0]->newCmd(cmd18);

    // RobotInstruction cmd19;
    // cmd19.enabled=true;
    // cmd19.servo[0].angle = 180;
    // cmd19.servo[0].speed = 10;
    // cmd19.servo[1].angle = 0;
    // cmd19.servo[1].speed = 10;
    // cmd19.exact = true;
    // cmd19.speed_smooth = true;
    // cmd19.synchronize = true;
    // robots[0]->newCmd(cmd19);
}

void print_time() {
    unsigned long time = millis();
    Serial.println(time-cur_time);
    if (time-cur_time > 10) {
        Serial.println("TIME: long");
    }
    cur_time = time;
}

void loop() {
    while (Serial.available()) {
        int robot_index = -1;
        char serial_in[INPUT_SIZE];
        byte serial_size = Serial.readBytesUntil('\n',serial_in, INPUT_SIZE);
        serial_in[serial_size-1] = 0; // add 0-terminator to end of string
        char *token = strtok(serial_in, ";");
        if (token != NULL)
        {
            if (!strncmp(token, "rob", 3)) {
                token = strtok(NULL, ";");
                rob_parse(token);
            } else {
                Serial.println("ERROR: Type not found");
            }
        }
    }

    for (int i = 0; i < ROBOTS_NUM; i++) {
        RobotInstruction actCmd = robots[i]->cmdFinished();
        if (actCmd.enabled == true) {
            rob_print_move(i+1, actCmd, robots[i]->getNumServos());
        }
        if (robots[i]->checkCollision()) {
            robots[i]->disableServos();
            Serial.println("ERROR: Collision detected");
        }
    }
}