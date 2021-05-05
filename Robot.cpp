#include "Robot.h"

Robot::Robot(int id, int port) {
    this->id = id;
    this->num_servos = 0;
    this->write_buffer = 0;
    this->read_buffer = 0;
    this->moving = false;

    this->servos = new MeSmartServo(port);
    this->servos->begin(115200);
    delay(5);
    this->servos->assignDevIdRequest();

    this->stopServos();

    for (int i = 0; i < this->num_servos;i++) {
        this->last_speed[i] = 0.0;
        this->servos->setZero(i+1);
    }
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

RobotInstruction Robot::finishCurrentRobotInstruction() {
    RobotInstruction finished;
    if (this->getCurrentRobotInstruction().exact == true) {
        this->resetSpeeds();
    }
    finished = this->getCurrentRobotInstruction();
    this->move_buffer[this->read_buffer].enabled = false;
    this->increase_read_buffer();
    return finished;
}

DriveInstruction Robot::getDriveInstruction(int id, RobotInstruction cmd) {
    return cmd.servo[id-1];
    // switch (id) {
    //     case 1: return cmd.servo[0];
    //         break;
    //     case 2: return cmd.servo[1];
    //         break;
    //     case 3: return cmd.servo[2];
    //         break;
    // }
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
        return false;
    }
    return true;
}

void Robot::setInitAngles(float init_angles[3]) {
    for (int i = 0; i < this->num_servos;i++) {
        Serial.println(init_angle[i]);
        this->init_angle[i] = init_angles[i];
        this->servos->setZero(i+1);
    }
}

RobotInstruction Robot::cmdFinished() {
    if (this->checkCmd() == true) {
        this->driveAllServo(this->getCurrentRobotInstruction());

        if (this->checkAllServo(this->getCurrentRobotInstruction())) {
            return this->finishCurrentRobotInstruction();
        }
    }
    RobotInstruction tmp;
    tmp.enabled = false;
    return tmp;
}

bool Robot::checkAllServo(RobotInstruction cmd) {
    if (this->cmdAvailable() == false) return false;
    for (int i = 1; i <= this->num_servos; i++) {
        DriveInstruction drive = this->getCurrentDriveInstruction(i);
        if (this->checkServo(i, drive.angle - this->init_angle[i-1], cmd.exact) == false) {
            return false;
        }
    }
    if (cmd.exact) {
        this->stopServos();
    }
    return true;
}

void Robot::stopServos() {
    for (int i = 1; i<= this->num_servos;i++) {
        this->servos->setPwmMove(i,0);
        // this->servos->setBreak(i, true);
    }
}

float Robot::getAngle(int id) {
    if (id > this->num_servos) return 0;
    return this->servos->getAngleRequest(id) + this->init_angle[id-1];
}

bool Robot::isRunning() {
    return this->moving;
}

bool Robot::checkServo(int id, int angle, bool exact) {
    long cur_angle = this->servos->getAngleRequest(id);
    if (exact == true) {
        return ((cur_angle <= angle + ANGLE_TOLERANCE_EXACT) && (cur_angle >= angle - ANGLE_TOLERANCE_EXACT));
    }
    return ((cur_angle <= angle + ANGLE_TOLERANCE) && (cur_angle >= angle - ANGLE_TOLERANCE));
}

bool Robot::driveServo(int id, DriveInstruction cmd) {
    this->moving = true;
    // if (id == 1 && this->id == 1) {
    //     Serial.print("Angle: ");Serial.print(cmd.angle - this->init_angle[id-1]);
    //     Serial.print("; Speed: ");Serial.println(cmd.speed);
    // }
    // this->servos->setBreak(id, false);
    // Serial.print("DEBUG: Drive Servo: ");Serial.println(id);
    // Serial.print("DEBUG: Angle:  ");Serial.print(cmd.angle);
    // Serial.print("; Cur Ang:");Serial.print(this->servos->getAngleRequest(id)+this->init_angle[id-1]);
    // Serial.print("; Speed:  "); Serial.println(cmd.speed);
    return this->servos->moveTo(id, cmd.angle - this->init_angle[id-1], cmd.speed);
}

DriveInstruction Robot::smoothCmd(DriveInstruction cmd, float last_speed) {
    DriveInstruction new_drive;
    new_drive.angle = cmd.angle;

    if (last_speed == cmd.speed) {
        new_drive.speed = last_speed;
    } else if (last_speed+SPEED_INCREMENT <= cmd.speed) {
        new_drive.speed = last_speed+SPEED_INCREMENT;
    } else if (last_speed < cmd.speed) {
        new_drive.speed = cmd.speed;
    } else if (last_speed-SPEED_INCREMENT >= cmd.speed) {
        new_drive.speed = last_speed-SPEED_INCREMENT;
    } else if (last_speed > cmd.speed) {
        new_drive.speed = cmd.speed;
    }

    if (new_drive.speed < SPEED_MIN) {
        new_drive.speed = SPEED_MIN;
    }

    return new_drive;
}

bool Robot::driveAllServo(RobotInstruction cmd) {
    // Serial.println("DEBUG: Drive All Servo");
    bool success = true;
    for (int i = 1; i <= this->num_servos; i++) {
        DriveInstruction new_cmd = this->smoothCmd(this->getDriveInstruction(i, cmd), this->last_speed[i-1]);

        // if (i == 1 && this->id == 1) {
        //     Serial.print("DEBUG: Speed target  : ");
        //     Serial.println(cmd.servo[i-1].speed);
        //     Serial.print("DEBUG: Speed last  : ");
        //     Serial.println(this->last_speed[i-1]);
        //     Serial.print("DEBUG: Speed after : ");
        //     Serial.println(new_cmd.speed);
        // }

        this->last_speed[i-1] = new_cmd.speed;
        
        if (this->driveServo(i, new_cmd) == false) {
            success = false;
        }
    }
    return success;
}

void Robot::setNumServos(int num_servos) {
    this->num_servos = num_servos;
}

int Robot::getNumServos() {
    return this->num_servos;
}

MeSmartServo* Robot::getServos() {
    return this->servos;
}