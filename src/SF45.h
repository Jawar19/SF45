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
        char        modelName[16];
        uint32_t    hardwareVer;
        uint32_t    firmwareVer;
        char        firmwareVersionStr[16];
        char        serialNumber[16];
        uint32_t    info_bitmap = 0x1FF;
    };

    /**
     * @brief PointInfo struct containing all data for a single point of the lidar
     * 
     */
    struct PointInfo_t
	{
		uint16_t	firstDistRaw;
		uint16_t	firstDistFilter;
		uint16_t	firstStrength;

		uint16_t	lastDistRaw;
		uint16_t	lastDistFilter;
		uint16_t	lastStrength;
		
		int			noise;
		float		temp;
		float		angle;
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

        static uint16_t readInt16(uint8_t* Buffer, uint32_t Offset);
        static uint32_t readInt32(uint8_t* Buffer, uint32_t Offset);

        /**
         * @brief interprets the response packet and returns a struct of data of the point.
         * 
         * @param packet response from the serial connection, to be interpreted, must not be null
         * @return PointInfo_t struct containing the interpreted data of the response package
         */
        PointInfo_t interpretResponse(lwResponsePacket &response);

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

        /**
         * @brief Get the Package Configuration from the LiDAR. This configuration is not persistent and must be configured at each setup.
         * first eight bit are used for the bitmap and configured as following:
         * |---------------------------------|
         * | BIT |   Output                  |
         * |---------------------------------|
         * | 0   |   First return RAW        |
         * | 1   |   First return filtered   |
         * | 2   |   First return strength   |
         * | 3   |   Last return RAW         |
         * | 4   |   Last return filtered    |
         * | 5   |   Last return strength    |
         * | 6   |   Background noise        |
         * | 7   |   Temperature             |
         * | 8   |   Yaw angle (degrees)     |
         * |---------------------------------|
         *  
         * more info can be read at: https://support.lightware.co.za/sf45b/#/command_detail/command%20descriptions/27.%20distance%20output
         *
         *  @return uint32_t containing the bitmap.
         */
        uint32_t getPackageConfig();

        /**
         * @brief Set the Package Config setting. Configures the return data of the distance data, both on request and data streaming.
         * bitmap contains the following
         * |---------------------------------|
         * | BIT |   Output                  |
         * |---------------------------------|
         * | 0   |   First return RAW        |
         * | 1   |   First return filtered   |
         * | 2   |   First return strength   |
         * | 3   |   Last return RAW         |
         * | 4   |   Last return filtered    |
         * | 5   |   Last return strength    |
         * | 6   |   Background noise        |
         * | 7   |   Temperature             |
         * | 8   |   Yaw angle (degrees)     |
         * |---------------------------------|
         * 
         * more info can be read at: https://support.lightware.co.za/sf45b/#/command_detail/command%20descriptions/27.%20distance%20output
         * 
         * @param config uint32_t where the first 8 bits are used as a bitmap structure
         * @return true if setting is successfully sent to the LiDAR
         * @return false if setting is unsuccessfully sent to the LiDAR
         */
        bool setPackageConfig(uint32_t config);

        /**
         * @brief Enable or disable scanning mode in the LiDAR by setting the boolean parameter
         * 
         * @param enable true to enable scanning mode, false to disable scanning mode.
         * @return true if scanning mode were successfully sent to the device.
         * @return false if the scanning mode were unsuccessfully sent to the device.
         */
        bool enableScanning(bool enable);

        /**
         * @brief Enable or disable streaming of data from the LiDAR according to the package config and sample rate.
         * 
         * @param enable true to enable streaming, false to disable streaming
         * @return true if streaming mode were successfully sent to the device.
         * @return false if streaming mode were unsuccessfully sent to the device.
         */
        bool enableStream(bool enable);

        /**
         * @brief Polls LiDAR one time and returns the data as a PointInfo struct object
         * 
         * @return PointInfo_t struct containing data of the given point 
         */
        PointInfo_t pollLidar();

    };
}
#endif /*SF45_h*/