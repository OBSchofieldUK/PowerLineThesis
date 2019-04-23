/// ****************************************************************************
///
/// \file      comm\Modbus\LtComLeddarM16Modbus.h
///
/// \brief     Structure definition for M16 sensor using Modbus communication
///
/// \author    David Levy
///
/// \since     September 2017
///
/// \copyright (c) 2017 LeddarTech Inc. All rights reserved.
///
/// ***************************************************************************

#pragma once

#include "comm/Modbus/LtComModbus.h"

namespace LtComLeddarM16Modbus
{
#define M16_MAX_SERIAL_DETECTIONS   48
#define M16_DISTANCE_SCALE          1
#define M16_AMPLITUDE_SCALE         64
#define M16_TEMPERATURE_SCALE       256
#define M16_SENSITIVITY_SCALE       256
#define M16_WAIT_AFTER_REQUEST      5    //Time to wait in ms after a request to be sure the next request is properly transmitted. 2ms should be enough in most cases

    enum eDeviceId
    {
        DID_ACCUMULATION_EXP = 0,
        DID_OVERSAMPLING_EXP = 1,
        DID_BASE_POINT_COUNT = 2,
        DID_THRESHOLD_OFFSET = 4,
        DID_LED_INTENSITY = 5,
        DID_ACQ_OPTIONS = 6,
        DID_CHANGE_DELAY = 7,
        DID_COM_SERIAL_PORT_MAX_ECHOES = 8,
        DID_PRECISION = 11,
        DID_COM_SERIAL_PORT_ECHOES_RES = 14,
        DID_SEGMENT_ENABLE = 15,
        DID_COM_SERIAL_PORT_STOP_BITS = 27,
        DID_COM_SERIAL_PORT_PARITY = 28,
        DID_COM_SERIAL_PORT_BAUDRATE = 29,
        DID_COM_SERIAL_PORT_ADDRESS = 30
    };

#pragma pack(push,1)
    typedef struct
    {
        uint8_t     mSize;                      /// Number of bytes of information (excluding this one). Currently 0x95 since the size of information returned is fixed.
        char        mSerialNumber[32];          /// Serial number as an ASCII string
        uint8_t     mRunStatus;                 /// Run status 0: OFF, 0xFF:ON. Should always return 0xFF, otherwise the sensor is defective
        char        mDeviceName[64];            /// The device name as a Unicode string
        char        mSoftwarePartNumber[16];    /// The software part number as an ASCII string
        char        mHardwarePartNumber[16];    /// The hardware part number as an ASCII string
        uint16_t    mFirmwareVersion[4];        /// The full firmware version as 4 16 - bit values
        uint32_t    mFirmwareCRC;               /// The firmware 32 - bit CRC
        uint16_t    mFirmwareType;              /// The firmware type(LeddarTech internal use)
        uint16_t    mFPGAVersion;               /// The FPGA version
        uint32_t    mDeviceOptions;             /// Device option flags(LeddarTech internal use)
        uint16_t    mDeviceId;                  /// Device identification code (9 for sensor module)
    } sLeddarM16ServerId;

    typedef struct
    {
        uint16_t    mDistance;
        uint16_t    mAmplitude;
        uint8_t     mFlags;
        /// Low 4 bits are flags describing the measurement:
        /// Bit 0 - Detection is valid(will always be set)
        /// Bit 1 - Detection was the result of object demerging
        /// Bit 2 - Reserved
        /// Bit 3 - Detection is saturated
        /// High 4 bits are the segment number.
    } sLeddarM16Detections;
#pragma pack(pop)
}
