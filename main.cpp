#include <iostream>
#include <iomanip>

// Lightware includes
#include "common.h"
#include "lwNx.h"


#define BIT(n)     (1U << (n))

//----------------------------------------------------------------------------------------------------------------------------------
// Helper utilities.
//----------------------------------------------------------------------------------------------------------------------------------
void printHexDebug(uint8_t* Data, uint32_t Size) {
	printf("Buffer: ");

	for (uint32_t i = 0; i < Size; ++i) {
		printf("0x%02X ", Data[i]);
	}

	printf("\n");
}

void exitWithMessage(const char* Msg) {
	printf("%s\nPress any key to Exit...\n", Msg);
	std::cin.ignore();
	exit(1);
}

void exitCommandFailure() {
	exitWithMessage("No response to command, terminating sample.\n");
}

uint16_t readInt16(uint8_t* Buffer, uint32_t Offset) {
	uint16_t result;
	result = (Buffer[Offset + 0] << 0) | (Buffer[Offset + 1] << 8);
	return result;
}


uint32_t readInt32(uint8_t* Buffer, uint32_t Offset) {
	uint32_t result;
	result = (Buffer[Offset + 0] << 0) | (Buffer[Offset + 1] << 8) | (Buffer[Offset + 2] << 16) | (Buffer[Offset + 3] << 24);
	return result;
}


typedef struct
{
	uint16_t	dist;
	int			angle;
	uint16_t	firstreturnstrength;
	float		temperature;
}pointInfo_t;


int main() {
    std::cout << "Welcome to the SF45 datalogger" << std::endl;

    platformInit();

    #ifdef __linux__
	    const char* portName = "/dev/ttyUSB0";
    #else
        const char* portName = "COM4";
    #endif

    int32_t     baudRate    = 921600;
	const int   FOV         = 50;	// measures in degrees on either side of 0

    uint8_t     updateRate  = 5;
    uint16_t    scanSpeed   = 15;

    uint32_t dist_info = 0;
	dist_info |= BIT(0);  // First return raw
	dist_info |= BIT(1);  // First return filter
	dist_info |= BIT(2);  // First return strength
	dist_info |= BIT(3);  // Last return raw
	dist_info |= BIT(4);  // Last return filter
	dist_info |= BIT(5);  // Last return strength
	dist_info |= BIT(6);  // Background noise
	dist_info |= BIT(7);  // Temperature
	dist_info |= BIT(8);  // Yaw angle

    lwSerialPort* serial = platformCreateSerialPort();
    if (!serial->connect(portName, baudRate))
    {
        exitWithMessage("Could not establish serial connection\n");
    }

    // Read the product name. (Command 0: Product name)
	char modelName[16];
	if (!lwnxCmdReadString(serial, 0, modelName)) { exitCommandFailure(); }

	// Read the hardware version. (Command 1: Hardware version)
	uint32_t hardwareVersion;
	if (!lwnxCmdReadUInt32(serial, 1, &hardwareVersion)) { exitCommandFailure(); }

	// Read the firmware version. (Command 2: Firmware version)
	uint32_t firmwareVersion;
	if (!lwnxCmdReadUInt32(serial, 2, &firmwareVersion)) { exitCommandFailure(); }
	char firmwareVersionStr[16];
	lwnxConvertFirmwareVersionToStr(firmwareVersion, firmwareVersionStr);

	// Read the serial number. (Command 3: Serial number)
	char serialNumber[16];
	if (!lwnxCmdReadString(serial, 3, serialNumber)) { exitCommandFailure(); }

    std::cout << std::setw(15) << std::left << "Model: " << std::setw(10) << std::right << modelName << '\n';
    std::cout << std::setw(15) << std::left << "HW Version: " << std::setw(10) << std::right << hardwareVersion << '\n';
    std::cout << std::setw(15) << std::left << "FW Version: " << std::setw(10) << std::right << firmwareVersionStr << '\n';

    // setup of the lidar
    if (!lwnxCmdWriteInt8(serial, 66, updateRate))
    {
        exitCommandFailure();
    }
    uint8_t readPSec;
	if (!lwnxCmdReadUInt8(serial, 66, &readPSec)) { exitCommandFailure(); }
    std::cout << std::setw(15) << std::left << "Sample rate: " << int(readPSec) << " Micro second delay" << std::endl;

    uint16_t ScanSpeed;
	if (!lwnxCmdReadUInt16(serial, 85, &ScanSpeed)) { exitCommandFailure(); }
    std::cout << std::setw(15) << std::left << "scan speed: " << int(ScanSpeed) << " speed" << std::endl;

    printf("Stop scanning..");
	if (!lwnxCmdWriteUInt8(serial, 96, 1)) { exitCommandFailure(); }

    return 0;
}