
#include "Arduino.h"
#include "ODriveArduino.h"

#define READ_TIMEOUT 200 // ms <- works fine at 115200 baud


// Print with stream operator
template<class T> inline Print& operator <<(Print& obj, T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print& obj, float arg) { obj.print(arg, 4); return obj; }

ODriveArduino::ODriveArduino(Stream& serial) : serial_(serial) {
    this->serial_ = serial;
}

void ODriveArduino::setPosition(int motor_number, float position, float velocity_feedforward, float torque_feedforward) {
    this->serial_ << "p " << motor_number << " " << position << " " << velocity_feedforward << " " << torque_feedforward << "\n";
}


void ODriveArduino::setVelocity(int motor_number, float velocity, float torque_feedforward) {
    this->serial_ << "v " << motor_number << " " << velocity << " " << torque_feedforward << "\n";
}

void ODriveArduino::setTorque(int motor_number, float torque) {
    this->serial_ << "c " << motor_number << " " << torque << "\n";
}

void ODriveArduino::setSimplePosition(int motor_number, float position) {
    this->serial_ << "t " << motor_number << " " << position << "\n";
}

float ODriveArduino::readFloat() {
    return readString().toFloat();
}

float ODriveArduino::getVelocity(int motor_number) {
    this->serial_ << "r axis" << motor_number << ".encoder.vel_estimate\n";
    return ODriveArduino::readFloat();
}

float ODriveArduino::getPosition(int motor_number) {
    this->serial_ << "r axis" << motor_number << ".encoder.pos_estimate\n";
    return ODriveArduino::readFloat();
}

float ODriveArduino::getPositionF(int motor_number) {
    this->serial_ << "f " << motor_number << "\n";
    String pos_vel = ODriveArduino::readString();
    String pos = pos_vel.substring(0, pos_vel.indexOf(' '));
    return pos.toFloat();
}

int32_t ODriveArduino::readInt() {
    return readString().toInt();
}

bool ODriveArduino::runState(int motor_number, int requested_state, bool wait_for_idle, float timeout) {
    int timeout_ctr = (int)(timeout * 10.0F);
    this->serial_ << "w axis" << motor_number << ".requested_state " << requested_state << '\n';
    if (wait_for_idle) {
        do {
            delay(100);
            this->serial_ << "r axis" << motor_number << ".current_state\n";
        } while (readInt() != AXIS_STATE_IDLE && --timeout_ctr > 0);
    }
    return timeout_ctr > 0;
}

void ODriveArduino::saveConfig() {
    this->serial_ << "ss\n";
}

void ODriveArduino::clearErrors() {
    this->serial_ << "sc\n";
}

void ODriveArduino::enableWatchdog(int motor_number, bool enable) {
    this->serial_ << "w axis" << motor_number << ".config.enable_watchdog " << enable << '\n';
}

void ODriveArduino::updateWatchdog(int motor_number) {
    this->serial_ << "u" << motor_number << "\n";
}



String ODriveArduino::readString() {
    String str = "";
    uint32_t timeout_start = millis();
    for (;;) {
        while (!this->serial_.available()) {
            if (millis() - timeout_start >= READ_TIMEOUT) {
                return str;
            }
        }
        char c = this->serial_.read();
        if (c == '\n') {
            break;
        }
        str += c;
    }
    return str;
}
