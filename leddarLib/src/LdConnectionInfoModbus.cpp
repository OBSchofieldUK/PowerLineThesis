// *****************************************************************************
// Module..: Leddar
//
/// \file    LdConnectionInfoModbus.cpp
///
/// \brief   Connection information on Modbus devices.
///
/// \author  Patrick Boulay
///
/// \since   July 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdConnectionInfoModbus.h"


// *****************************************************************************
// Function: LdConnectionInfoModbus::LdConnectionInfoModbus
//
/// \brief   Constructor.
///
/// \param  aSerialPort     Serial port of the device.
/// \param  aDescription    .
/// \param  aBaud           Baud rate.
/// \param  aParity         Parity ( see LdConnectionInfoModbus::eParity ).
/// \param  aDataBits       Data bits ( 5, 6, 7 or 8 ).
/// \param  aStopBits       Stop bits ( 1 or 2 ).
/// \param  aModbusAddr     .
///
/// \author  Patrick Boulay
///
/// \since   July 2016
// *****************************************************************************

LeddarConnection::LdConnectionInfoModbus::LdConnectionInfoModbus( const std::string &aSerialPort, const std::string &aDescription, uint32_t aBaud, eParity aParity, uint8_t aDataBits,
        uint8_t aStopBits, uint8_t aModbusAddr ) :
    LdConnectionInfo( CT_LIB_MODBUS, aSerialPort ),
    mSerialPort( aSerialPort ),
    mDescription( aDescription ),
    mBaud( aBaud ),
    mParity( aParity ),
    mDataBits( aDataBits ),
    mStopBits( aStopBits ),
    mModbusAddr( aModbusAddr )
{

}

// *****************************************************************************
// Function: LdConnectionInfoModbus::~LdConnectionInfoModbus
//
/// \brief   Destructor.
///
/// \author  Patrick Boulay
///
/// \since   July 2016
// *****************************************************************************

LeddarConnection::LdConnectionInfoModbus::~LdConnectionInfoModbus()
{
}
