// *****************************************************************************
// Module..: Leddar
//
/// \file    LdCarrierEnhancedModbus.h
///
/// \brief   Definition of the LeddarVu8 carrier board connecting by Modbus.
///
/// \author  Patrick Boulay
///
/// \since   September 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************


#pragma once

#include "LdSensor.h"
#include "LdConnectionUniversal.h"
#include "LdConnectionUniversalModbus.h"

namespace LeddarDevice
{

    class LdCarrierEnhancedModbus : virtual public LdSensor
    {
    public:
        LdCarrierEnhancedModbus( LeddarConnection::LdConnection *aConnection, LeddarCore::LdPropertiesContainer *aProperties );

        virtual void GetConfig( void ) override {}
        virtual void SetConfig( void ) override  {}
        virtual void GetConstants( void ) override;
        virtual bool GetEchoes( void ) override { return false; }
        virtual void GetStates( void ) override {}
        virtual void Reset( LeddarDefines::eResetType, LeddarDefines::eResetOptions = LeddarDefines::RO_NO_OPTION ) override {}

    protected:
        struct sCarrierDeviceInformation
        {
            uint8_t  mModbusAddress;
            uint8_t  mFunctionCode;
            uint8_t  mSubFunctionCode;
            char     mHardwarePartNumber[ 32 ];
            char     mHardwareSerialNumber[ 32 ];
            uint32_t mOptions;
            uint16_t mCrc;
        };
        struct sCarrierFirmwareInformation
        {
            uint8_t  mModbusAddress;
            uint8_t  mFunctionCode;
            uint8_t  mSubFunctionCode;
            char     mFirmwarePartNumber[ 32 ];
            char     mFirmwareVersion[ 4 ];
        };


        LeddarConnection::LdConnectionUniversalModbus *mModbusConnection;
    };
}
