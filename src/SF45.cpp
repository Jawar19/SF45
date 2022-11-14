#include "SF45.h"

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

uint16_t SF45::getScanSpeed(){

    uint16_t scanSpeed = -1;
    lwnxCmdReadUInt16(serial, 85, &scanSpeed);

    return scanSpeed;
}

bool SF45::setScanSpeed(const uint16_t scanSpeed){
    return lwnxCmdWriteUInt16(this->serial, 85, scanSpeed);
}