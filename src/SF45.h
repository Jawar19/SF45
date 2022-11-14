/**
 * @file SF45.h
 * @author Jacob Warrer (jrw@phaseone.com)
 * @brief SF45 serial interface base on the Lightware SF45/B and the accompanying serial interface which this is dependent on.
 *  -   https://lightwarelidar.com/collections/frontpage/products/copy-of-sf45-b-50m
 *  -   https://support.lightware.co.za/sf45b/#/introduction
 *  -   https://github.com/LightWare-Optoelectronics/SampleLibrary/tree/master/binary%20protocol/sf45_lwnx_c
 * @version 0.1
 * @date 2022-11-14
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef SF45_H
#define SF45_H

#include <algorithm>
#include <list>

#include "common.h"
#include "lwNx.h"

namespace LIDAR{

    /**
     * @brief Unitdata struct contains basic information of the LiDAR connected to the system.
     *
     */
    struct UnitData
    {
        char modelName[16];
        uint32_t hardwareVer;
        uint32_t firmwareVer;
        char firmwareVersionStr[16];
        char serialNumber[16];
    };

    enum SampleRates {
        sr50, 
        sr100, 
        sr200, 
        sr400, 
        sr500, 
        sr625, 
        sr1000, 
        sr1250, 
        sr1538, 
        sr2000, 
        sr2500, 
        sr5000
    };

    class SF45
    {

    private:
        lwSerialPort *serial = nullptr;
        UnitData unitData;

        /**
         * @brief Updates the unit data within this object to reflect the current state of the connected device.
         * 
         * @return true All read command executed successfully
         * @return false any of the read commands failed during extraction
         */
        bool updateData();

    public:
        SF45(const char *pPortName, const int32_t &baudRate);
        ~SF45();

        /**
         * @brief Print a header text of the unit, describing the current HW/FW versions and serial numbers 
         * 
         */
        void printUnitHeader();

        /**
         * @brief Get the Scan Speed of the LiDAR.
         * polls the SF45 to get the current scan speed, return -1 if read from sensor fails.
         * 
         * @return uint16_t unsigned integer value describing the delay between each scan position. A higher value will cause the scan to take longer.
         */
        uint16_t getScanSpeed();
        
        /**
         * @brief Set the Scan Speed of the LiDAR.
         * Send package to the LiDAR to set the scanSpeed
         * 
         * @param scanSpeed The delay between each scan position. A higher value will cause the scan to take longer.
         * @return true If the write were successful
         * @return false if the write were not successful
         */
        bool setScanSpeed(const uint16_t scanSpeed);

        uint8_t getSampleRate();
        bool setSampleRate(const SampleRates sampleRate);

        float getLowAngle();
        bool setLowAngle(const float angle);

        float getHighAngle();
        bool setHighAngle(const float angle);

        float getAngle();
        bool setAngle(const float angle);

        float getFoV();
        bool setFoV(const float FoV);


    };
}
#endif /*SF45_h*/