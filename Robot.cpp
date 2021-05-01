#include "Robot.h"

Robot::Robot(int id, int port, int num_servos) {
    this->id = id;
    this->num_servos = num_servos;
    this->write_buffer = 0;
    this->read_buffer = 0;
    this->moving = false;

    this->last_speed[0] = 0.0;
    this->last_speed[1] = 0.0;
    this->last_speed[2] = 0.0;

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

void Robot::resetSpeeds() {
    for (int i = 0; i < this->num_servos;i++) {
        this->last_speed[i] = 0;
    }
}

void Robot::finishCurrentRobotInstruction() {
    this->move_buffer[this->read_buffer].enabled = false;
    this->resetSpeeds();
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
    if (this->cmdAvailable() == false) {
        this->moving = false;
        return true;
    }

    this->driveAllServo(this->getCurrentRobotInstruction());

    if (this->checkAllServo(this->getCurrentRobotInstruction())) {
        this->finishCurrentRobotInstruction();
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
    this->moving = true;
    return this->servos->moveTo(id, cmd.angle, cmd.speed);
}

DriveInstruction Robot::smoothCmd(DriveInstruction cmd, float last_speed) {
    DriveInstruction new_drive;
    if (abs(last_speed) < SPEED_MIN) {
        last_speed = SPEED_MIN;
    } else if (abs(last_speed) >= cmd.speed) {
        last_speed -= SPEED_INCREMENT;
    } else {
        last_speed += SPEED_INCREMENT;
    }
    
    new_drive.angle = cmd.angle;
    new_drive.speed = last_speed;

    return new_drive;
}

bool Robot::driveAllServo(RobotInstruction cmd) {
    Serial.println("Drive All Servo");
    bool success = true;
    for (int i = 1; i <= this->num_servos; i++) {
        DriveInstruction new_cmd = this->smoothCmd(this->getDriveInstruction(i, cmd), this->last_speed[i-1]);
        this->last_speed[i-1] = new_cmd.speed;
        Serial.print("Speed after : ");
        Serial.println(new_cmd.speed);
        if (this->driveServo(id, new_cmd) == false) {
            success = false;
        }
    }
    return success;
}


MeSmartServo* Robot::getServos() {
    return this->servos;
}