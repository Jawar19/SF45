#include <iomanip>
#include <thread>

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

        this->stopReadStream();

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

uint16_t SF45::readInt16(uint8_t* Buffer, uint32_t Offset) {
	uint16_t result;
	result = (Buffer[Offset + 0] << 0) | (Buffer[Offset + 1] << 8);
	return result;
}


uint32_t SF45::readInt32(uint8_t* Buffer, uint32_t Offset) {
	uint32_t result;
	result = (Buffer[Offset + 0] << 0) | (Buffer[Offset + 1] << 8) | (Buffer[Offset + 2] << 16) | (Buffer[Offset + 3] << 24);
	return result;
}

PointInfo_t SF45::interpretResponse(lwResponsePacket &response) {
    PointInfo_t point;
    point.firstDistRaw = readInt16(response.data, 4);
    point.firstDistFilter = readInt16(response.data, 6);
    point.firstStrength = readInt16(response.data, 8);

    point.lastDistRaw = readInt16(response.data, 10);
    point.lastDistFilter = readInt16(response.data, 12);
    point.lastStrength = readInt16(response.data, 14);

    point.noise = readInt16(response.data, 16);
    point.temp = readInt16(response.data, 18) / 100;
    point.angle = (int16_t)(readInt16(response.data, 20)) / 100.;

    return point;
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

uint32_t SF45::getPackageConfig() {
    uint32_t config = -1;
    lwnxCmdReadUInt32(this->serial, 27, &config);
    return config;
}

bool SF45::setPackageConfig(uint32_t config) {
    if (config > 0x1FF) {
        return false;
    }
    return lwnxCmdWriteUInt32(this->serial, 27, config);
}

bool SF45::enableScanning(bool enable) {
    lwnxCmdWriteUInt8(this->serial, 96, uint8_t(enable));
}

bool SF45::enableStream(bool enable) {
    uint32_t setValue = 0;
    if (enable) {
        setValue = 5;
    }
    return lwnxCmdWriteUInt32(this->serial, 30, setValue);
}

PointInfo_t SF45::pollLidar(){
    lwResponsePacket response;

    if (!lwnxRecvPacket(this->serial, 44, &response, 1000)) {
        return;
    }

    return interpretResponse(response);
}

void SF45::readStreamWorker() {
    std::cout << "Worker thread called" << std::endl;
    while (isReadingStream) {
        std::cout << "reading stream, in thread: " << std::this_thread::get_id << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    std::cout << "Ending stream reader thread" << std::endl;
}

bool SF45::startReadStream() {
    std::cout << "Starting worker thread" << std::endl;
    this->streamReaderThread = std::thread(&SF45::readStreamWorker, this);
}

bool SF45::stopReadStream() {
    if (this->streamReaderThread.joinable()) {
        this->isReadingStream = false;
        this->streamReaderThread.join();
    }
}