/// *****************************************************************************
/// Module..: Leddar
///
/// \file    LdSensorM16Modbus.h
///
/// \brief   Class of M16 Sensor communucating in modbus protocol.
///
/// \author  David Levy
///
/// \since   September 2017
///
/// Copyright (c) 2017 LeddarTech Inc. All rights reserved.
/// *****************************************************************************

#pragma once

#include "LdSensor.h"
#include "LdConnectionInfoModbus.h"

#include "LdLibModbusSerial.h"

namespace LeddarDevice
{
    class LdSensorM16Modbus : public LdSensor
    {
    public:
        explicit LdSensorM16Modbus( LeddarConnection::LdConnection *aConnection );
        ~LdSensorM16Modbus( void );

        virtual void      Connect( void ) override;
        virtual void      GetConfig( void ) override;
        virtual void      SetConfig( void ) override;
        virtual void      GetConstants( void ) override;
        virtual bool      GetEchoes( void ) override;
        virtual void      GetStates( void ) override;
        virtual void      Reset( LeddarDefines::eResetType /*aType*/, LeddarDefines::eResetOptions = LeddarDefines::RO_NO_OPTION ) override {};

    protected:

        const LeddarConnection::LdConnectionInfoModbus  *mConnectionInfoModbus;
        LeddarConnection::LdLibModbusSerial *mInterface;
    };
}

