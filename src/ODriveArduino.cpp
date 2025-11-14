
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

uint32_t ODriveArduino::getErrors(int motor_number) {
    uint32_t errCode;
    this->serial_ << "r axis" << motor_number << ".active_errors\n";
    errCode = (uint32_t)readInt();
    this->serial_ << "r axis" << motor_number << ".disarm_reason\n";
    errCode = errCode | (uint32_t)readInt();    
    return errCode;
}

String ODriveArduino::decodeErrors(int32_t errorCode) {
    String errDescr;
    errDescr.clear();
    if ((errorCode & ODRIVE_ERROR_INITIALIZING)              > 0) errDescr.concat("Initializing; ");
    if ((errorCode & ODRIVE_ERROR_SYSTEM_LEVEL)              > 0) errDescr.concat("System level error; ");
    if ((errorCode & ODRIVE_ERROR_TIMING_ERROR)              > 0) errDescr.concat("Timing error; ");
    if ((errorCode & ODRIVE_ERROR_MISSING_ESTIMATE)          > 0) errDescr.concat("Missing estimate; ");
    if ((errorCode & ODRIVE_ERROR_BAD_CONFIG)                > 0) errDescr.concat("Bad config; ");
    if ((errorCode & ODRIVE_ERROR_DRV_FAULT)                 > 0) errDescr.concat("Driver fault; ");
    if ((errorCode & ODRIVE_ERROR_MISSING_INPUT)             > 0) errDescr.concat("Missing input; ");
    if ((errorCode & ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE)       > 0) errDescr.concat("DC bus overvoltage; ");
    if ((errorCode & ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE)      > 0) errDescr.concat("DC bus undervoltage; ");
    if ((errorCode & ODRIVE_ERROR_DC_BUS_OVER_CURRENT)       > 0) errDescr.concat("DC bus overcurrent; ");
    if ((errorCode & ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT) > 0) errDescr.concat("DC bus over regeneration; ");
    if ((errorCode & ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION)   > 0) errDescr.concat("Brake deadtime violation; ");
    if ((errorCode & ODRIVE_ERROR_MOTOR_OVER_TEMP)           > 0) errDescr.concat("Brake duty cycle NaN; ");
    if ((errorCode & ODRIVE_ERROR_INVERTER_OVER_TEMP)        > 0) errDescr.concat("Invalid brake resistance; ");
    if ((errorCode & ODRIVE_ERROR_VELOCITY_LIMIT_VIOLATION)  > 0) errDescr.concat("Velocity limit; ");
    if ((errorCode & ODRIVE_ERROR_POSITION_LIMIT_VIOLATION)  > 0) errDescr.concat("Position limit; ");
    if ((errorCode & ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED)    > 0) errDescr.concat("Watchdog expired; ");
    if ((errorCode & ODRIVE_ERROR_ESTOP_REQUESTED)           > 0) errDescr.concat("ESTOP requested; ");
    if ((errorCode & ODRIVE_ERROR_SPINOUT_DETECTED)          > 0) errDescr.concat("Spinout detected; ");
    if ((errorCode & ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED)   > 0) errDescr.concat("Brake resistor disarmed; ");
    if ((errorCode & ODRIVE_ERROR_THERMISTOR_DISCONNECTED)   > 0) errDescr.concat("Thermistor disconnected; ");
    if ((errorCode & ODRIVE_ERROR_CALIBRATION_ERROR)         > 0) errDescr.concat("Calibration error; ");
    return errDescr;
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
