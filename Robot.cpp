#include "Robot.h"

Robot::Robot(int id, int port, int num_servos) {
    this->id = id;
    this->num_servos = num_servos;
    this->write_buffer = 0;
    this->read_buffer = 0;
    this->moving = false;

    this->servos = new MeSmartServo(port);
    this->servos->begin(115200);
    delay(5);
    this->servos->assignDevIdRequest();
}

Robot::~Robot() {
    free(this->servos);
}

void Robot::increase_write_buffer() {
    this->write_buffer = (this->write_buffer+1) % BUFFER_LEN;
}

void Robot::increase_read_buffer() {
    this->read_buffer = (this->read_buffer+1) % BUFFER_LEN;
}

bool Robot::cmdAvailable() {
    return this->getCurrentRobotInstruction().enabled;
}

bool Robot::newCmd(RobotInstruction cmd) {
    if (this->move_buffer[write_buffer].enabled == true) {
        return false;
    }

    this->move_buffer[write_buffer] = cmd;
    this->increase_write_buffer();
    return true;
    
}

RobotInstruction Robot::getCurrentRobotInstruction() {
    return this->move_buffer[this->read_buffer];
}

void Robot::finishCurrentRobotInstruction() {
    this->move_buffer[this->read_buffer].enabled = false;
    this->increase_read_buffer();
}

DriveInstruction Robot::getDriveInstruction(int id, RobotInstruction cmd) {
    switch (id) {
        case 1: return cmd.motor1;
            break;
        case 2: return cmd.motor2;
            break;
        case 3: return cmd.motor3;
            break;
    }
}

DriveInstruction Robot::getCurrentDriveInstruction(int id) {
    RobotInstruction cur_cmd = this->getCurrentRobotInstruction();
    return this->getDriveInstruction(id, cur_cmd);
}

// bool Robot::start() {
//     if (this->cmdAvailable() == false) {
//         return false;
//     }
//     this->driveAllServo(this->getCurrentRobotInstruction());
// }

bool Robot::checkCmd() {
    Serial.println("Check Cmd");
    if (this->cmdAvailable() == false) {
        return true;
    }
    Serial.println(this->moving);
    if (this->moving == false) {
        
        this->driveAllServo(this->getCurrentRobotInstruction());
    }
    else if (this->checkAllServo(this->getCurrentRobotInstruction())) {
        this->finishCurrentRobotInstruction();
        if (this->cmdAvailable() == true) {
            this->driveAllServo(this->getCurrentRobotInstruction());
        } else {
            this->moving = false;
        }
    }
}

bool Robot::checkAllServo(RobotInstruction cmd) {
    if (this->cmdAvailable() == false) return false;
    for (int i = 1; i <= this->num_servos; i++) {
        DriveInstruction cmd = this->getCurrentDriveInstruction(i);
        if (this->checkServo(i, cmd.angle) == false) {
            return false;
        }
    }
    return true;
}

bool Robot::checkServo(int id, int angle) {
    long cur_angle = this->servos->getAngleRequest(id);
    return ((cur_angle <= angle + ANGLE_TOLLERANCE) && (cur_angle >= angle - ANGLE_TOLLERANCE));
}

bool Robot::driveServo(int id, DriveInstruction cmd) {
    Serial.println("Drive Servo");
    this->moving = true;
    return this->servos->moveTo(id, cmd.angle, cmd.speed);
}

bool Robot::driveAllServo(RobotInstruction cmd) {
    Serial.println("Drive All Servo");
    bool success = true;
    for (int i = 1; i <= this->num_servos; i++) {
        if (this->driveServo(id, this->getDriveInstruction(i, cmd)) == false) {
            success = false;
        }
    }
    return success;
}


MeSmartServo* Robot::getServos() {
    return this->servos;
}