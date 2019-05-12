// *****************************************************************************
// Module..: Leddar
//
/// \file    LdConnectionInfoModbus.h
///
/// \brief   Connection information on Modbus devices.
///
/// \author  Patrick Boulay
///
/// \since   July 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#pragma once
#include "LdConnectionInfo.h"
#include "LtDefines.h"

namespace LeddarConnection
{
    class LdConnectionInfoModbus : public LdConnectionInfo
    {
    public:
        enum eParity
        {
            MB_PARITY_NONE,
            MB_PARITY_EVEN,
            MB_PARITY_ODD
        };

        LdConnectionInfoModbus( const std::string& aSerialPort, const std::string& aDescription, uint32_t aBaud, eParity aParity, uint8_t aDataBits, uint8_t aStopBits, uint8_t aModbusAddr );
        virtual ~LdConnectionInfoModbus();

        virtual std::string GetSerialPort( void ) const
        {
            return mSerialPort;
        }
        virtual std::string GetDescription (void) const
        {
            return mDescription;
        }

        virtual uint32_t GetBaud( void ) const
        {
            return mBaud;
        }
        virtual void SetBaud( uint32_t aBaud )
        {
            mBaud = aBaud;
        }
        virtual eParity GetParity( void ) const
        {
            return mParity;
        }
        virtual void SetParity( eParity aParity )
        {
            mParity = aParity;
        }
        virtual uint8_t GetDataBits( void ) const
        {
            return mDataBits;
        }
        virtual void SetDataBits( uint8_t aDataBits )
        {
            mDataBits = aDataBits;
        }
        virtual uint8_t GetStopBits( void ) const
        {
            return mStopBits;
        }
        virtual void SetStopBits( uint8_t aStopBits )
        {
            mStopBits = aStopBits;
        }
        virtual uint8_t GetModbusAddr( void ) const
        {
            return mModbusAddr;
        }
        virtual void SetModbusAddr( uint8_t aModbusAddr )
        {
            mModbusAddr = aModbusAddr;
        }

    protected:
        std::string mSerialPort;
        std::string mDescription;
        uint32_t    mBaud;
        eParity     mParity;
        uint8_t     mDataBits;
        uint8_t     mStopBits;
        uint8_t     mModbusAddr;
    };
}
