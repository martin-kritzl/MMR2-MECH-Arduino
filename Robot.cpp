/*
Copyright (C) 2021 Martin Kritzl

This file is part of MMR2-MECH-Arduino
(https://github.com/martin-kritzl/MMR2-MECH-Arduino)


MMR2-MECH-Arduino is free software: you can redistribute it and/or modify 
it under the terms of the GNU General Public License as published by 
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful, 
but WITHOUT ANY WARRANTY; without even the implied warranty of 
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
GNU General Public License for more details.


You should have received a copy of the GNU General Public License 
along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#include "Robot.h"

Robot::Robot(int id, int port) {
    this->id = id;
    this->num_servos = 0;
    this->write_buffer = 0;
    this->read_buffer = 0;
    this->started = true;
    this->count_same_angles = 0;
    this->sleep_until = 0;
    this->cur_break_status = false;

    this->servos = new MeSmartServo(port);
    this->servos->begin(115200);
    delay(5);
    this->connectServos();
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
    if (this->move_buffer[this->write_buffer].enabled == true || cmd.enabled == false) {
        return false;
    }

    //If no command is present, then the robot is set to running state.
    if (cmdAvailable() == false) {
        this->setAllServosMoving();
    }

    //For the commands which are alone in the buffer
    //Otherwise the synchronization would not be applied, 
    //because in method cmdFinished the current command would not be synchronized.
    if (cmd.synchronize && this->read_buffer == this->write_buffer) {
        cmd = this->synchronizeServos(cmd);
    }

    this->move_buffer[write_buffer] = cmd;
    this->increase_write_buffer();

    return true;
    
}

bool Robot::home() {
    RobotInstruction cmd;
    cmd.enabled = true;
    cmd.servo[0].speed = SPEED_MIN;
    cmd.synchronize = true;
    cmd.exact = true;
    for (int i = 0; i < this->num_servos; i++) {
        cmd.servo[i].angle = this->init_angle[i];
    }
    return this->newCmd(cmd);
}

RobotInstruction Robot::getCurrentRobotInstruction() {
    return this->move_buffer[this->read_buffer];
}

void Robot::resetSpeeds() {
    for (int i = 0; i < this->num_servos;i++) {
        this->last_speed[i] = 0;
    }
}

void Robot::setBreaks(bool break_status, bool force) {
    if ((this->cur_break_status != break_status) || force) {
        for (int i = 0; i < this->num_servos;i++) {
            if (break_status == true) {
                this->moving_servos[i] = false;
            } else {
                this->servos->setPwmMove(i+1, 0);
            }
        }
        this->cur_break_status = break_status;
    }
}

bool Robot::isConnected() {
    if (this->num_servos == 0) return false;
    for (int i = 1; i <= this->num_servos;i++) {
        float voltage = this->servos->getVoltageRequest(i);
        if (voltage < MIN_VOLTAGE || voltage > MAX_VOLTAGE)
            return false;
    }
    return true;
}

void Robot::setAllServosMoving() {
    for (int i = 0; i < this->num_servos; i++) {
        this->moving_servos[i] = true;
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

    this->sleep_until = millis() + finished.delay;

    // At least one motor must be set to moving again if there is still a
    // command in the buffer, because otherwise the status idle is displayed in between.
    if (this->cmdAvailable()) {
        this->moving_servos[0] = true;
    }

    return finished;
}

DriveInstruction Robot::getDriveInstruction(int id, RobotInstruction cmd) {
    return cmd.servo[id-1];
}

DriveInstruction Robot::getCurrentDriveInstruction(int id) {
    RobotInstruction cur_cmd = this->getCurrentRobotInstruction();
    return this->getDriveInstruction(id, cur_cmd);
}

bool Robot::checkCmd() {
    if (this->cmdAvailable() == false) {
        //If there is no more command and the state is still running
        //Stop all motors and set running to false
        if (this->isMoving()) {
            this->setBreaks(true,true);
        }
        
        return false;
    }
    return true;
}

void Robot::setInitAngles(float init_angles[]) {
    this->setBreaks(true,false);
    this->clearCmds();
    for (int i = 0; i < this->num_servos;i++) {
        this->init_angle[i] = init_angles[i];
        this->last_angles[i] = init_angles[i];
    }
}

void Robot::calibrate() {
    for (int i = 1; i <= this->num_servos;i++) {
        this->servos->setZero(i+1);
    }
}

void Robot::connectServos() {
    this->servos->assignDevIdRequest();
    delay(1000);
    this->setBreaks(true,true);
}
    

RobotInstruction Robot::cmdFinished() {
    RobotInstruction result;
    if (millis() > this->sleep_until) {
        // Reset the time, otherwise the overflow of millis() will result in a
        // wrong waiting time
        this->sleep_until = 0;
        if (this->checkCmd() == true && this->started == true) {
            this->driveAllServo(this->getCurrentRobotInstruction());

            if (this->checkAllServo(this->getCurrentRobotInstruction())) {
                result = this->finishCurrentRobotInstruction();
                if (this->getCurrentRobotInstruction().synchronize) {
                    RobotInstruction tmp = this->synchronizeServos(this->getCurrentRobotInstruction());
                    this->move_buffer[read_buffer] = tmp;
                }
                
                return result;
            }
            if (this->checkCollision()) {
                this->disableServos();
                result.collision = true;
            }
        }
    }
    result.enabled = false;
    return result;
}

bool Robot::checkAllServo(RobotInstruction cmd) {
    if (this->cmdAvailable() == false) return false;
    for (int i = 1; i <= this->num_servos; i++) {
        DriveInstruction drive = this->getCurrentDriveInstruction(i);
        if (this->checkServo(i, drive.angle - this->init_angle[i-1], cmd.exact) == true) {
            this->moving_servos[i-1] = false;
        }
    }
    if (this->isMoving()) {
        return false;
    }
    if (cmd.exact) {
        this->setBreaks(true,true);
    }
    return true;
}

void Robot::stopServos() {
    this->setBreaks(true,false);
}

float Robot::getAngle(int id) {
    if (id > this->num_servos) return 0;
    return this->servos->getAngleRequest(id) + this->init_angle[id-1];
}

void Robot::getAngles(float angles[]) {
    for (int i = 0; i < this->num_servos; i++) {
        angles[i] = this->getAngle(i+1);
    }
}

RobotInstruction Robot::synchronizeServos(RobotInstruction cmd) {
    float diff_angles[this->num_servos];
    float act_angles[this->num_servos];
    this->getAngles(act_angles);
    double delta_max = 0;

    for (int i = 0; i < this->num_servos; i++) {
        diff_angles[i] = fabs(cmd.servo[i].angle - act_angles[i]);
        if (diff_angles[i] > delta_max) {
            delta_max = diff_angles[i];
        }
    }
    if (delta_max>0) {
        //Necessary time is calculated
        double time = delta_max/cmd.servo[0].speed;

        for (int i = 0; i < this->num_servos; i++) {
            cmd.servo[i].speed = diff_angles[i]/time;
            if (cmd.servo[i].speed < SPEED_MIN)
                cmd.servo[i].speed = SPEED_MIN;
        }
    }
    return cmd;
}

bool Robot::isMoving() {
    bool moving = false;
    for (int i = 0; i < this->num_servos; i++) {
        if (this->moving_servos[i] == true)
            moving = true;
    }
    return moving;
}

bool Robot::isDisabled() {
    return !this->started;
}

bool Robot::checkServo(int id, int angle, bool exact) {
    long cur_angle = this->servos->getAngleRequest(id);
    if (exact == true) {
        return ((cur_angle <= angle + ANGLE_TOLERANCE_EXACT) && (cur_angle >= angle - ANGLE_TOLERANCE_EXACT));
    }
    return ((cur_angle <= angle + ANGLE_TOLERANCE) && (cur_angle >= angle - ANGLE_TOLERANCE));
}

bool Robot::checkCollision() {
    if (this->cmdAvailable() == false || this->started == false) {
        return false;
    }
    bool same = false;
    float current_angle;
    for (int i = 0; i < this->num_servos; i++) {
        current_angle = this->getAngle(i+1);

        if (this->last_angles[i] == current_angle && this->moving_servos[i] == true) {
            same = true;
        }
        last_angles[i] = current_angle;
    }
    if (same == true) {
        this->count_same_angles++;
    } else {
        this->count_same_angles = 0;
    }

    if (this->count_same_angles > COLLISION_MAX_COUNT) {
        //Reset the counter for the next error case
        this->count_same_angles = 0;
        return true;
    }
    return false;
}

bool Robot::driveServo(int id, DriveInstruction cmd) {
    if (this->last_send_inst[id-1].angle == cmd.angle && this->last_send_inst[id-1].speed == cmd.speed) {
        return true;
    } else {
        this->last_send_inst[id-1] = cmd;
    }
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
    bool success = true;
    for (int i = 1; i <= this->num_servos; i++) {
        DriveInstruction drive = this->getDriveInstruction(i, cmd);

        if (cmd.speed_smooth) {
            drive = this->smoothCmd(drive, this->last_speed[i-1]);
        }

        this->last_speed[i-1] = drive.speed;
        
        if (this->driveServo(i, drive) == false) {
            success = false;
        }
    }
    return success;
}

void Robot::enableServos() {
    this->started = true;
}

void Robot::disableServos() {
    this->setBreaks(false,true);
    this->started = false;
}

void Robot::clearCmds() {
    this->setBreaks(true,true);
    RobotInstruction tmp;
    tmp.enabled = false;
    for (int i = 0; i < BUFFER_LEN; i++) {
        this->move_buffer[i] = tmp;
    }
    this->write_buffer = this->read_buffer;
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