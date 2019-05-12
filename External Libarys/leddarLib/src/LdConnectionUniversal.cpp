// *****************************************************************************
// Module..: Leddar
//
/// \file    LdConnectionUniversal.cpp
///
/// \brief   Base class of LdConnectionUniversal
///
/// \author  Patrick Boulay
///
/// \since   February 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************


#include "LdConnectionUniversalDefines.h"
#include "LdConnectionUniversal.h"
#include "LtCRCUtils.h"
#include "LtExceptions.h"
#include "LtIntUtilities.h"
#include "LtStringUtils.h"
#include "LtTimeUtils.h"
#include "comm/LtComLeddarTechPublic.h"


#include <cstddef>
#include <cstring>
#include <iomanip>
#include <sstream>

using namespace LeddarConnection;

// *****************************************************************************
// Function: LdConnectionUniversal::LdConnectionUniversal
//
/// \brief   Constructor
///
/// \param  aConnectionInfo Connection information
/// \param  aInterface      Interface
///
/// \author  Patrick Boulay
///
/// \since   March 2016
// *****************************************************************************

LdConnectionUniversal::LdConnectionUniversal( const LdConnectionInfo *aConnectionInfo,
        LdConnection            *aInterface ) :
    LdConnection( aConnectionInfo, aInterface ),
    mAlwaysReadyCheck( false ),
    mDeviceReadyTimeout( 10 )
{
    mIsBigEndian = LeddarUtils::LtIntUtilities::IsBigEndian();
}

// *****************************************************************************
// Function: LdConnectionUniversal::~LdConnectionUniversal
//
/// \brief   Destructor
///
/// \author  Patrick Boulay
///
/// \since   March 2016
// *****************************************************************************

LdConnectionUniversal::~LdConnectionUniversal()
{
}

// *****************************************************************************
// Function: LdConnectionUniversal::Init
//
/// \brief  Read information needed on connection in Universal protocol.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
// *****************************************************************************

void
LdConnectionUniversal::Init()
{
    try
    {
        // Get the sensor type
        uint32_t lDeviceTypeRegister;

        if( mConnectionInfo->GetType() == LeddarConnection::LdConnectionInfo::CT_ETHERNET_UNIVERSAL )
        {
            lDeviceTypeRegister = LtComLeddarTechPublic::LT_COMM_DEVICE_TYPE_ADDRESS_NEW;
        }
        else
        {
            lDeviceTypeRegister = LtComLeddarTechPublic::LT_COMM_DEVICE_TYPE_ADDRESS_OLD;
        }

        ReadRegister( lDeviceTypeRegister, ( ( uint8_t * )&mDeviceType ), sizeof( uint16_t ), 5 );

        // Set the mSecureTransferEnableFlag for CRC check
        uint8_t lSecureFlag = 1;
        WriteRegister( 0x00FFFB00 + offsetof( sTransactionCfg, mSecureTransferEnableFlag ), &lSecureFlag, sizeof( lSecureFlag ), 0 );

        // Disable ready
        uint8_t lReadyDeassertingData = 3;
        this->WriteRegister( 0x00FFFB00 + offsetof( sTransactionCfg, mReadyDeassertingData ), &lReadyDeassertingData, 1 );

        // Enable the partial blocking mode
        uint8_t lMode = 0;
        this->WriteRegister( 0x00FFFB00 + offsetof( sTransactionCfg, mTransferMode ), &lMode, 1 );
    }
    catch( std::exception )
    {
        throw;
    }

}

// *****************************************************************************
// Function: LdConnectionUniversal::IsDeviceReady
//
/// \brief   Check if the device is ready for read or write.
///
/// \return  Return true if the device is ready.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
// *****************************************************************************
bool
LdConnectionUniversal::IsDeviceReady( int32_t aTimeout,
                                      int16_t  aCRCTry )
{
    while( aTimeout > 0 )
    {
        try
        {
            if( ( GetStatusRegister( aCRCTry ) & 0x01 ) == 0 )
            {
                return true;
            }
        }
        catch( std::exception & )
        {}

        LeddarUtils::LtTimeUtils::Wait( mDeviceReadyTimeout );
        aTimeout -= mDeviceReadyTimeout;
    }

    return false;
}



// *****************************************************************************
// Function: LdConnectionUniversal::IsWriteEnable
//
/// \brief   Get the write enable flag.
///
/// \return  Return true if the device is write enable.
///
/// \author  Patrick Boulay
///
/// \since   April 2016
// *****************************************************************************

bool
LdConnectionUniversal::IsWriteEnable( int16_t aCrcTry )
{
    uint8_t lStatusReg = GetStatusRegister( aCrcTry );
    return ( ( lStatusReg & 0x02 ) == 0x02 ) && ( lStatusReg != 0xFF );
}

// *****************************************************************************
// Function: LdConnectionUniversal::GetStatusRegister
//
/// \brief   Get the status register of the sensor.
///
/// \return  Return the register value.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
// *****************************************************************************

uint8_t
LdConnectionUniversal::GetStatusRegister( int16_t aCRCTry )
{
    uint8_t *lInputBuffer, *lOutputBuffer;
    InternalBuffers( lInputBuffer, lOutputBuffer );
    Read( REGMAP_RDSR, 0, 1, aCRCTry );
    return lOutputBuffer[0];
}

// *****************************************************************************
// Function: LdConnectionUniversal::ReadRegister
//
/// \brief   Read register on the device
///
/// \param  aAddress Address of the register.
/// \param  aBuffer Buffer to get the register.
/// \param  aSize Size of the buffer to receive.
/// \param  aCRCTry Number of retry if CRC check fail. (0 mean no CRC check).
///
/// \author  Patrick Boulay
///
/// \since   April 2016
// *****************************************************************************

void LdConnectionUniversal::ReadRegister( const uint32_t &aAddress,
        uint8_t        *aBuffer,
        const uint16_t &aSize,
        int16_t         aCRCTry )
{
    try
    {
        uint8_t *lInputBuffer, *lOutputBuffer;
        InternalBuffers( lInputBuffer, lOutputBuffer );
        Read( REGMAP_READ, aAddress, aSize, aCRCTry );
        memcpy( aBuffer, lOutputBuffer, aSize );
    }
    catch( std::exception )
    {
        throw;
    }
}

// *****************************************************************************
// Function: LdConnectionUniversal::ReadRegister
//
/// \brief   Read register on the device without CRC check.
///          This version exist because of the pure virtual version in LdConnection.
///
/// \param  aAddress Address of the register.
/// \param  aBuffer Buffer to get the register.
/// \param  aSize Size of the buffer to receive.
///
/// \author  Patrick Boulay
///
/// \since   May 2016
// *****************************************************************************

void LdConnectionUniversal::ReadRegister( const uint32_t &aAddress,
        uint8_t        *aBuffer,
        const uint16_t &aSize )
{
    ReadRegister( aAddress, aBuffer, aSize, 0 );
}

// *****************************************************************************
// Function: LdConnectionUniversal::SetAlwaysReadyCheck
//
/// \brief   Set the mAlwaysReadyCheck flag.
///
/// param    aValue Always ready check value.
///
/// \author  Vincent Simard Bilodeau
///
/// \since   August 2016
// *****************************************************************************

void
LdConnectionUniversal::SetAlwaysReadyCheck( bool aValue )
{
    mAlwaysReadyCheck = aValue;
}

// *****************************************************************************
// Function: LdConnectionUniversal::WriteRegister
//
/// \brief   Write register on the device
///
/// \param  aAddress Address of the register.
/// \param  aBuffer Buffer to get the register.
/// \param  aSize Size of the buffer to receive.
/// \param  aCRCTry     Number of retry if CRC check fail. (0 mean no CRC check).
///
/// \exception LtNotConnectedException when device is not connected.
///
/// \author  Patrick Boulay
///
/// \since   April 2016
// *****************************************************************************

void
LdConnectionUniversal::WriteRegister( const uint32_t &aAddress,
                                      const uint8_t  *aBuffer,
                                      const uint16_t &aSize,
                                      int16_t         aCRCTry )
{
    uint8_t *lInputBuffer, *lOutputBuffer;
    InternalBuffers( lInputBuffer, lOutputBuffer );

    try
    {
        SetWriteEnable( true );
        memcpy( lInputBuffer, aBuffer, aSize );
        Write( REGMAP_WRITE, aAddress, aSize, aCRCTry );
    }
    catch( std::exception &/*e*/ )
    {
        SetWriteEnable( false );
        throw;
    }

    SetWriteEnable( false );
}

// *****************************************************************************
// Function: LdConnectionUniversal::WriteRegister
//
/// \brief   Write register on the device
///
/// \param  aAddress Address of the register.
/// \param  aBuffer Buffer to get the register.
/// \param  aSize Size of the buffer to receive.
///
/// \exception runtime_error when device is not connected.
///
/// \author  Patrick Boulay
///
/// \since   April 2016
// *****************************************************************************

void
LdConnectionUniversal::WriteRegister( const uint32_t &aAddress,
                                      const uint8_t  *aBuffer,
                                      const uint16_t &aSize )
{
    WriteRegister( aAddress, aBuffer, aSize, 5 );
}

// *****************************************************************************
// Function: LdConnectionUniversal::Read
//
/// \brief   Read data from the device using internal buffer.
///          Do not use in embded implementation.
///
/// \param  aOpCode             Hexa corresponding to the function (read, write, read status, ...) sent to the device
/// \param  aAddress  Address of the data to read.
/// \param  aData     Memory space to read from device.
/// \param  aDataSize Size of memory to read.
/// \param  aCRCTry   Number of retry if CRC check fail. (0 mean no CRC check).
/// \param  aIsReadyTimeout     Timeout in milliseconds of the timer that wait the device
///                             to be ready after the write operation. (Default 1000)
///                             The value must be to - 1 if no waiting is required.
///
/// \author  Patrick Boulay
///
/// \since   April 2016
// *****************************************************************************

void
LdConnectionUniversal::Read( uint8_t          aOpCode,
                             uint32_t         aAddress,
                             uint8_t         *aData,
                             const uint32_t  &aDataSize,
                             int16_t          aCRCTry,
                             const int16_t   &aIsReadyTimeout )
{

    uint8_t *lInputBuffer, *lOutputBuffer;
    InternalBuffers( lInputBuffer, lOutputBuffer );

    try
    {
        Read( aOpCode, aAddress, aDataSize, aCRCTry, aIsReadyTimeout );
        memcpy( aData, lOutputBuffer, aDataSize );
    }
    catch( std::exception )
    {
        throw;
    }
}

// *****************************************************************************
// Function: LdConnectionUniversal::Write
//
/// \brief   Write data from the device copying data to internal buffer.
///          Do not use in embded implementation.
///
/// \param  aOpCode             Hexa corresponding to the function (read, write, read status, ...) sent to the device
/// \param  aAddress  Address of the data to read.
/// \param  aData     Memory space to send to device.
/// \param  aDataSize Size of memory to read.
/// \param  aCRCTry             Number of retry if CRC check fail. (0 mean no CRC check).
/// \param  aPostIsReadyTimeout Timeout  Timeout in milliseconds of the timer that wait the device
///                   to be ready after the write operation. (Default 1000)
///                   The value must be to -1 if no waiting is required.
/// \param  aPreIsReadyTimeout  Timeout  Timeout in milliseconds of the timer that wait the device
///                         to be ready before the write operation. (Default 1000)
///                         The value must be to -1 if no waiting is required.
///
/// \exception std::runtime_error If the timeout is expired and the device is not ready for other instruction.
///
/// \author  Patrick Boulay
///
/// \since   April 2016
// *****************************************************************************

void
LdConnectionUniversal::Write( uint8_t          aOpCode,
                              uint32_t         aAddress,
                              uint8_t         *aData,
                              const uint32_t  &aDataSize,
                              int16_t          aCRCTry,
                              const int16_t   &aPostIsReadyTimeout,
                              const int16_t   &aPreIsReadyTimeout,
                              const uint16_t  &aWaitAfterOpCode )
{
    uint8_t *lInputBuffer, *lOutputBuffer;
    InternalBuffers( lInputBuffer, lOutputBuffer );

    try
    {
        memcpy( lInputBuffer, aData, aDataSize );
        Write( aOpCode, aAddress, aDataSize,
               aCRCTry, aPostIsReadyTimeout, aPreIsReadyTimeout, aWaitAfterOpCode );
    }
    catch( std::exception )
    {
        throw;
    }
}

// *****************************************************************************
// Function: LdConnectionUniversal::SetWriteEnable
//
/// \brief   Send set write enable or disable.
///
/// \param  aStatus   Write enable = true, disable = false
/// \param  aCrcTry   Number of retry if CRC check fail. (0 mean no CRC check).
///
/// \author  Patrick Boulay
///
/// \since   May 2016
// *****************************************************************************

void
LdConnectionUniversal::SetWriteEnable( bool    aStatus,
                                       int16_t aCrcTry )
{
    uint8_t *lInputBuffer, *lOutputBuffer;
    InternalBuffers( lInputBuffer, lOutputBuffer );
    const uint8_t lOpCode = ( aStatus ? REGMAP_WREN : REGMAP_WRDIS );
    Write( lOpCode, 0x0, 0, aCrcTry );

    if( IsWriteEnable() != aStatus )
    {
        throw std::runtime_error( "Error to set write enable status." );
    }
}

// *****************************************************************************
// Function: LdConnectionUniversal::GetErrorInfo
//
/// \brief   Return the string associated to the error code
///
/// \param   aErrorCode Error code to get the information
///
/// \return  Information string about the error code.
///
/// \author  Patrick Boulay
///
/// \since   July 2016
// *****************************************************************************

std::string
LdConnectionUniversal::GetErrorInfo( uint32_t aErrorCode )
{
    switch( aErrorCode )
    {
        case REGMAP_NO_ERR:
            return "No error";

        case REGMAP_ACCESS_RIGHT_VIOLATION:
            return "Access right violation";

        case REGMAP_INVALID_ADDR:
            return "Invalid address";

        case REGMAP_CMD_NOT_FOUND:
            return "Command not found";

        case REGMAP_WRITE_DISABLE:
            return "Write disable";

        case REGMAP_CRC_FAILED:
            return "CRC failed";

        case REGMAP_CMD_EXEC_ERROR:
            return "Command execution error";

        default:
            return "Invalid error code";
    }
}
