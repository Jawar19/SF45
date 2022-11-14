#include <iomanip>

#include "SF45.h"

using namespace LIDAR;

SF45::SF45(const char* pPortName, const int32_t &baudRate) {
    platformInit();
    serial = platformCreateSerialPort();
    if( !this->serial->connect(pPortName, baudRate)){
        throw new std::runtime_error("Connection Error, could not connect to unit on init");
    }


}

SF45::~SF45(){
    if (this->serial) {
        serial->disconnect();
        delete serial;
    }
}

bool SF45::updateData(){
    if (!lwnxCmdReadString(this->serial, 0, this->unitData.modelName)) { return false; }
    if (!lwnxCmdReadUInt32(this->serial, 1, &this->unitData.hardwareVer)) { return false; }
    if (!lwnxCmdReadUInt32(this->serial, 2, &this->unitData.firmwareVer)) { return false; }
    if (!lwnxCmdReadString(this->serial, 3, this->unitData.serialNumber)) { return false; }

    lwnxConvertFirmwareVersionToStr(this->unitData.firmwareVer, this->unitData.firmwareVersionStr);

    return true;
}

void SF45::printUnitHeader(){
    std::cout << "SF45 platform" << std::endl;
    std::cout << std::setw(15) << std::left << "Model: " << std::setw(10) << std::right << this->unitData.modelName << '\n';
    std::cout << std::setw(15) << std::left << "Serial#: " << std::setw(10) << std::right << this->unitData.serialNumber << '\n';
    std::cout << std::setw(15) << std::left << "HW Version: " << std::setw(10) << std::right << this->unitData.hardwareVer << '\n';
    std::cout << std::setw(15) << std::left << "FW Version: " << std::setw(10) << std::right << this->unitData.firmwareVersionStr << '\n';
}

uint16_t SF45::getScanSpeed(){

    uint16_t scanSpeed = -1;
    lwnxCmdReadUInt16(serial, 85, &scanSpeed);

    return scanSpeed;
}

bool SF45::setScanSpeed(const uint16_t scanSpeed){
    if (scanSpeed >= 5 && scanSpeed <= 2000) {
        return lwnxCmdWriteUInt16(this->serial, 85, scanSpeed);
    }
}

uint8_t SF45::getSampleRate(){
    uint8_t sampleRate = -1;
    lwnxCmdReadUInt8(this->serial, 66, &sampleRate);
    return sampleRate;
}

bool SF45::setSampleRate(const SampleRates sampleRate){
    return lwnxCmdWriteUInt8(serial, 66, sampleRate);
}


float SF45::getLowAngle() {
    float lowAngle = -1;
    lwnxCmdReadFloat32(this->serial, 98, &lowAngle);
    return lowAngle;
}

float SF45::getHighAngle() {
    float highAngle = -1;
    lwnxCmdReadFloat32(this->serial, 99, &highAngle);
    return highAngle;
}

bool SF45::setLowAngle(const float angle) {
    if (angle < -5 && angle > -170){
	    return lwnxCmdWriteFloat32(this->serial, 98, angle);
    }
    return false;
}

bool SF45::setHighAngle(const float angle) {
	if (angle < 170 && angle > 5){
	    return lwnxCmdWriteFloat32(this->serial, 99, angle);
    }
    return false;
}

float SF45::getAngle() {
    float angle = -1;
    lwnxCmdReadFloat32(this->serial, 97, &angle);
    return angle;
}

bool SF45::setAngle(const float angle) {
    if (angle > -170 && angle < 170) {
        return lwnxCmdWriteFloat32(this->serial, 97, angle);
    }
    return false;
}

float SF45::getFoV() {
    float low = this->getLowAngle();
    float high = this->getHighAngle();
    return high-low;
}

bool SF45::setFoV(const float fov) {
    if (fov >= 10 && fov <= 340) {
        this->setHighAngle((fov/2));
        this->setLowAngle((fov/2)*-1);
    }
}