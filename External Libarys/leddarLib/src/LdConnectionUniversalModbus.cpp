// *****************************************************************************
// Module..: Leddar
//
/// \file    LdConnectionUniversalModbus.cpp
///
/// \brief   Base class of LdConnectionModbus
///
/// \author  Patrick Boulay
///
/// \since   July 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdConnectionUniversalModbus.h"
#include "LdConnectionUniversalDefines.h"

#include "LdConnection.h"
#include "LdConnectionInfoModbus.h"
#include "LdConnectionModbusStructures.h"

#include "LtExceptions.h"
#include "LtIntUtilities.h"
#include "LtStringUtils.h"
#include "LtTimeUtils.h"

#include <cstring>

using namespace LeddarConnection;

// *****************************************************************************
// Function: LdConnectionUniversalModbus::LdConnectionUniversalModbus
//
/// \brief   Constructor
///
/// \param  aConnectionInfo Connection information
/// \param  aInterface      Interface information
///
/// \author  Patrick Boulay
///
/// \since   July 2016
// *****************************************************************************

LdConnectionUniversalModbus::LdConnectionUniversalModbus( const LdConnectionInfo *aConnectionInfo, LdConnection *aInterface ) :
    LdConnectionUniversal( aConnectionInfo, aInterface )
{
    mTransferBufferSize = MODBUS_MAX_SIZE + 768; //1024
    mConnectionInfoModbus = dynamic_cast< const LdConnectionInfoModbus * >( mConnectionInfo );
    mInterfaceModbus = dynamic_cast< LdInterfaceModbus * >( aInterface );
    mTransferInputBuffer = new uint8_t[mTransferBufferSize];
    mTransferOutputBuffer = new uint8_t[mTransferBufferSize];

    SetDeviceReadyTimeout(100);
}

// *****************************************************************************
// Function: LdConnectionUniversalModbus::~LdConnectionUniversalModbus
//
/// \brief   Destructor
///
/// \author  Patrick Boulay
///
/// \since   July 2016
// *****************************************************************************

LdConnectionUniversalModbus::~LdConnectionUniversalModbus()
{
    if( this->IsConnected() )
    {
        delete[] mTransferInputBuffer;
        mTransferInputBuffer = nullptr;
        delete[] mTransferOutputBuffer;
        mTransferOutputBuffer = nullptr;
        this->Disconnect();
    }
}

// *****************************************************************************
// Function: LdConnectionUniversalModbus::IsEngineStop
//
/// \brief   Check if the carrier engine is disabled.
///
/// \author  Vincent Simard Bilodeau
///
/// \since   July 2016
// *****************************************************************************

bool
LdConnectionUniversalModbus::IsEngineStop( int32_t aTimeout )
{
    do
    {
        mInterfaceModbus->ReadRegisters( 0x0A, 1, ( uint16_t * )mTransferOutputBuffer );

        if( ( ( uint16_t * )mTransferOutputBuffer )[0] == 10 )
            break;

        LeddarUtils::LtTimeUtils::Wait( 10 );
        aTimeout -= 10;

        if( aTimeout <= 0 )
        {
            throw LeddarException::LtTimeoutException( "Carrier engine never stopped" );
            return false;
        }

    }
    while( 1 );

    return true;
}

// *****************************************************************************
// Function: LdConnectionUniversalModbus::StopEngine
//
/// \brief   Stop carrier engine.
///
/// \author  Vincent Simard Bilodeau
///
/// \since   July 2016
// *****************************************************************************

void
LdConnectionUniversalModbus::StopEngine()
{
    mInterfaceModbus->WriteRegister( 0x0A, 0 );
}

// *****************************************************************************
// Function: LdConnectionUniversalModbus::StartEngine
//
/// \brief   Start carrier engine.
///
/// \author  David Levy
///
/// \since   March 2017
// *****************************************************************************

void
LdConnectionUniversalModbus::StartEngine()
{
    try
    {
        mInterfaceModbus->WriteRegister( 0x0A, 1 );
    }
    catch( LeddarException::LtComException )
    {}
}

// *****************************************************************************
// Function: LdConnectionUniversalModbus::Connect
//
/// \brief   Establish a connection with de device.
///
/// \author  Vincent Simard Bilodeau
///
/// \since   July 2016
// *****************************************************************************

void
LdConnectionUniversalModbus::Connect( void )
{
    try
    {
        // Connect interface
        mInterface->Connect();

        Init();
    }
    catch( std::exception )
    {
        throw;
    }
}

// *****************************************************************************
// Function: LdConnectionUniversalModbus::Init
//
/// \brief   Initialize the device
///
/// \author  Patrick Boulay
///
/// \since   April 2017
// *****************************************************************************

void
LdConnectionUniversalModbus::Init( void )
{
    try
    {
        // Disable carrier acquistion engine.
        StopEngine();

        while( !IsEngineStop( 10000 ) );

        // Get device type and set secure transaction mode
        LdConnectionUniversal::Init();

    }
    catch( std::exception )
    {
        throw;
    }
}

// *****************************************************************************
// Function: LdConnectionUniversalModbus::RawConnect
//
/// \brief   Establish a raw connection with de device.
///
/// \author  Patrick Boulay
///
/// \since   July 2016
// *****************************************************************************

void
LdConnectionUniversalModbus::RawConnect( void )
{
    mInterface->Connect();

    // Disable carrier acquistion engine.
    StopEngine();

    while (!IsEngineStop(10000));
}

// *****************************************************************************
// Function: LdConnectionUniversalModbus::Disconnect
//
/// \brief   Disconnect the device.
///
/// \author  Patrick Boulay
///
/// \since   July 2016
// *****************************************************************************

void
LdConnectionUniversalModbus::Disconnect( void )
{

    if( mInterface->IsConnected() )
    {
        StartEngine();
        mInterface->Disconnect();
    }
}

// *****************************************************************************
// Function: LdConnectionUniversalModbus::IsConnected
//
/// \brief   Return true is the device is connected.
///
/// \author  Patrick Boulay
///
/// \since   July 2016
// *****************************************************************************

bool
LdConnectionUniversalModbus::IsConnected( void ) const
{
    if( mInterface != nullptr )
    {
        return mInterface->IsConnected();
    }

    return false;
}

// *****************************************************************************
// Function: LdConnectionUniversalModbus::Read
//
/// \brief   Read data from device to mTransferOutputBuffer.
///
/// \param  aOpCode         Hexa corresponding to the function (read, write, read status, ...) sent to the device
/// \param  aAddress Address of the register.
/// \param  aDataSize       Size of the buffer to receive.
/// \param  aCRCTry         Number of retry if CRC check fail. (0 mean no CRC check).
/// \param  aIsReadyTimeout Timeout in milliseconds of the timer that wait the device
///                         to be ready after the read operation.
///                         The value must be to - 1 if no waiting is required.
///
/// \exception LtNotConnectedException when device is not connected.
///
/// \author  Patrick Boulay
/// \author  Frédéric Parent
///
/// \since   September 2016
// *****************************************************************************

void
LdConnectionUniversalModbus::Read( uint8_t aOpCode, uint32_t aAddress,
                                   const uint32_t &aDataSize, int16_t aCRCTry,
                                   const int16_t &aIsReadyTimeout )
{

    // Check if device is ready (only for 0xb opcode)
    int16_t lIsReadyTimeout = this->mAlwaysReadyCheck ? 5000 : 0;

    if( ( aIsReadyTimeout > 0 || lIsReadyTimeout ) && aOpCode == REGMAP_READ )
    {
        const uint16_t lTimeout = std::max(aIsReadyTimeout, lIsReadyTimeout);
        if (!IsDeviceReady(lTimeout))
            throw LeddarException::LtTimeoutException("(LdConnectionUniversalModbus::Read) Timeout expired. Device not ready for other operation ( timeout: " + LeddarUtils::LtStringUtils::IntToString(lTimeout) + " ).", true);

    }

    if( aDataSize > mTransferBufferSize )
        throw std::overflow_error("ModBus transfer buffer is too small, resize it with ResizeInternalBuffers.");

    LdConnectionModbuStructures::sModbusPacket lReceiveBuffer;
    LdConnectionModbuStructures::sModbusPacket lAnswerBuffer;

    if( aOpCode == REGMAP_READ )
    {
        uint32_t lBytesToRead = aDataSize;
        uint32_t lBytesRead = 0;
        size_t lRet = 0;
        uint8_t lDataSize = static_cast<uint8_t>( sizeof( ( ( LdConnectionModbuStructures::sModbusReadDataAnswer * ) 0 )->mData ) );

        lReceiveBuffer.mHeader.mModbusAddress   = mConnectionInfoModbus->GetModbusAddr();
        lReceiveBuffer.mHeader.mFunctionCode    = 0x42;

        while( lBytesToRead != 0 )
        {
            lReceiveBuffer.uRequest.mReadData.mNumberOfBytesToRead  = ( lBytesToRead > lDataSize ? lDataSize : lBytesToRead );
            lReceiveBuffer.uRequest.mReadData.mBaseAddress          = aAddress + lBytesRead;

            while( aCRCTry >= 0 )
            {
                try
                {
                    uint32_t lOutSize = static_cast<uint32_t>( sizeof( LdConnectionModbuStructures::sModbusHeader ) +
                                        sizeof( LdConnectionModbuStructures::sModbusReadDataReq ) );
                    uint32_t lInSize = static_cast<uint32_t>( sizeof( LdConnectionModbuStructures::sModbusHeader ) +
                                       offsetof( LdConnectionModbuStructures::sModbusReadDataAnswer, mData ) +
                                       MODBUS_CRC_SIZE );

                    mInterfaceModbus->SendRawRequest( ( uint8_t * )&lReceiveBuffer, lOutSize );
                    lRet = mInterfaceModbus->ReceiveRawConfirmation( ( uint8_t * )&lAnswerBuffer, lInSize + lReceiveBuffer.uRequest.mReadData.mNumberOfBytesToRead );

                    if( lRet < lReceiveBuffer.uRequest.mReadData.mNumberOfBytesToRead )
                        throw LeddarException::LtComException( "Missing bytes in modbus packet." );

                    break;

                }
                catch( ... )
                {
                    aCRCTry--;

                    if( aCRCTry < 0 )
                        throw;
                }
            }

            memcpy( mTransferOutputBuffer + lBytesRead, lAnswerBuffer.uAnswer.mReadData.mData, lReceiveBuffer.uRequest.mReadData.mNumberOfBytesToRead );
            lBytesRead      += lReceiveBuffer.uRequest.mReadData.mNumberOfBytesToRead;
            lBytesToRead    -= lReceiveBuffer.uRequest.mReadData.mNumberOfBytesToRead;
        }
    }
    else
    {
        uint32_t lOutSize = static_cast<uint32_t>( sizeof( LdConnectionModbuStructures::sModbusHeader ) +
                            sizeof( LdConnectionModbuStructures::sModbusSendOpCodeReq ) );
        uint32_t lInSize = static_cast<uint32_t>( sizeof( LdConnectionModbuStructures::sModbusHeader ) +
                           sizeof( LdConnectionModbuStructures::sModbusSendOpCodeAnswer ) +
                           MODBUS_CRC_SIZE );

        lReceiveBuffer.mHeader.mModbusAddress               = mConnectionInfoModbus->GetModbusAddr();
        lReceiveBuffer.mHeader.mFunctionCode                = 0x44;
        lReceiveBuffer.uRequest.mSendOpCode.mOpCode         = aOpCode;
        lReceiveBuffer.uRequest.mSendOpCode.mOptionalArg    = 0;

        while( aCRCTry >= 0 )
        {
            try
            {
                mInterfaceModbus->SendRawRequest( ( uint8_t * )&lReceiveBuffer, lOutSize );
                mInterfaceModbus->ReceiveRawConfirmation( ( uint8_t * )&lAnswerBuffer, lInSize );
                break;
            }
            catch( ... )
            {
                aCRCTry--;

                if( aCRCTry < 0 )
                    throw;
            }
        }

        memcpy( mTransferOutputBuffer, &lAnswerBuffer.uAnswer.mSendOpCode.mRetVal, sizeof( lAnswerBuffer.uAnswer.mSendOpCode.mRetVal ) );
    }

}

// *****************************************************************************
// Function: LdConnectionUniversalModbus::Write
//
/// \brief   Write data from mTransferInputBuffer to device
///
/// \param  aOpCode Hexa corresponding to the function (read, write, read status, ...) sent to the device
/// \param  aAddress Address of the register.
/// \param  aDataSize Size of the data to send to device.
/// \param  aPostIsReadyTimeout Whether to verify if the device timeout.
/// \param  aWaitAfterOpCode Wait time in ms after the OpCode is sent. Some command need a sleep after the command is sent.
///
/// \exception LtNotConnectedException when device timeout during operation.
///
/// \author  Patrick Boulay
/// \author  Frédéric Parent
///
/// \since   September 2016
// *****************************************************************************

void
LdConnectionUniversalModbus::Write( uint8_t aOpCode, uint32_t aAddress, const uint32_t &aDataSize, int16_t /*aCRCTry*/,
                                    const int16_t &aPostIsReadyTimeout, const int16_t & /*aPreIsReadyTimeout*/, const uint16_t &aWaitAfterOpCode )
{
    LdConnectionModbuStructures::sModbusPacket lWriteBuffer;
    LdConnectionModbuStructures::sModbusPacket lAnswerBuffer;

    if( aOpCode == REGMAP_WRITE )
    {
        uint16_t lBytesToWrite;
        uint8_t lDataSize = static_cast<uint8_t>( sizeof( ( ( LdConnectionModbuStructures::sModbusWriteDataReq * ) 0 )->mData ) );

        lWriteBuffer.mHeader.mModbusAddress = mConnectionInfoModbus->GetModbusAddr();
        lWriteBuffer.mHeader.mFunctionCode  = 0x43;

        lBytesToWrite = aDataSize;
        uint16_t lBytesWritten = 0;

        while( lBytesToWrite != 0 )
        {
            uint32_t lOutSize = static_cast<uint32_t>( sizeof( LdConnectionModbuStructures::sModbusHeader ) +
                                offsetof( LdConnectionModbuStructures::sModbusWriteDataReq, mData ) );
            uint32_t lInSize = static_cast<uint32_t>( sizeof( LdConnectionModbuStructures::sModbusHeader ) +
                               sizeof( LdConnectionModbuStructures::sModbusWriteDataAnswer ) +
                               MODBUS_CRC_SIZE );

            lWriteBuffer.uRequest.mWriteData.mNumberOfBytesToWrite  = ( lBytesToWrite > lDataSize ? lDataSize : lBytesToWrite );
            lWriteBuffer.uRequest.mWriteData.mBaseAddress           = aAddress + lBytesWritten;
            memcpy( lWriteBuffer.uRequest.mWriteData.mData, mTransferInputBuffer + lBytesWritten, lWriteBuffer.uRequest.mWriteData.mNumberOfBytesToWrite );

            mInterfaceModbus->SendRawRequest( ( uint8_t * )&lWriteBuffer, lOutSize + lWriteBuffer.uRequest.mWriteData.mNumberOfBytesToWrite );
            mInterfaceModbus->ReceiveRawConfirmation( ( uint8_t * )&lAnswerBuffer, lInSize );

            lBytesWritten += lWriteBuffer.uRequest.mWriteData.mNumberOfBytesToWrite;
            lBytesToWrite -= lWriteBuffer.uRequest.mWriteData.mNumberOfBytesToWrite;

            if( aPostIsReadyTimeout > 0 )
            {
                if( !IsDeviceReady( aPostIsReadyTimeout ) )
                {
                    throw LeddarException::LtTimeoutException("(LdConnectionUniversalModbus::Write) Timeout expired. Device not ready for other operation ( timeout: " + LeddarUtils::LtStringUtils::IntToString(aPostIsReadyTimeout) + " ).", true);
                }
            }
        }
    }
    else
    {
        uint32_t lOutSize = static_cast<uint32_t>( sizeof( LdConnectionModbuStructures::sModbusHeader ) +
                            sizeof( LdConnectionModbuStructures::sModbusSendOpCodeReq ) );
        uint32_t lInSize = static_cast<uint32_t>( sizeof( LdConnectionModbuStructures::sModbusHeader ) +
                           sizeof( LdConnectionModbuStructures::sModbusSendOpCodeAnswer ) +
                           MODBUS_CRC_SIZE );

        lWriteBuffer.mHeader.mModbusAddress             = mConnectionInfoModbus->GetModbusAddr();
        lWriteBuffer.mHeader.mFunctionCode              = 0x44;
        lWriteBuffer.uRequest.mSendOpCode.mOpCode       = aOpCode;
        lWriteBuffer.uRequest.mSendOpCode.mOptionalArg  = 0;
        memcpy( &lWriteBuffer.uRequest.mSendOpCode.mOptionalArg, mTransferInputBuffer, aDataSize );

        mInterfaceModbus->SendRawRequest( ( uint8_t * )&lWriteBuffer, lOutSize );
        mInterfaceModbus->ReceiveRawConfirmation( ( uint8_t * )&lAnswerBuffer, lInSize );

        if( aWaitAfterOpCode > 0 )
        {
            LeddarUtils::LtTimeUtils::Wait( aWaitAfterOpCode );
        }
    }
}

// *****************************************************************************
// Function: LdConnectionUniversalModbus::ReadDeviceType
//
/// \brief   Read the device type of the connected sensor.
///
/// \return  Return the device type.
///
/// \exception LtNotConnectedException when device is not connected.
///
/// \author  Patrick Boulay
/// \author  Frédéric Parent
///
/// \since   September 2016
// *****************************************************************************

uint16_t
LdConnectionUniversalModbus::ReadDeviceType( void )
{
    LdConnectionModbuStructures::sModbusPacket lRequestBuffer;
    LdConnectionModbuStructures::sModbusPacket lAnswerBuffer;

    uint32_t lOutSize = static_cast<uint32_t>( sizeof( LdConnectionModbuStructures::sModbusHeader ) );
    uint32_t lInSize = static_cast<uint32_t>( sizeof( LdConnectionModbuStructures::sModbusHeader ) +
                       sizeof( LdConnectionModbuStructures::sModbusServerId ) +
                       MODBUS_CRC_SIZE );

    lRequestBuffer.mHeader.mModbusAddress   = mConnectionInfoModbus->GetModbusAddr();
    lRequestBuffer.mHeader.mFunctionCode    = 0x11;

    mInterfaceModbus->SendRawRequest( ( uint8_t * )&lRequestBuffer, lOutSize );
    mInterfaceModbus->ReceiveRawConfirmation( ( uint8_t * )&lAnswerBuffer, lInSize );

    return lAnswerBuffer.uAnswer.mServerId.mDeviceType;
}

// *****************************************************************************
// Function: LdConnectionUniversalModbus::InternalBuffers
//
/// \brief   Gets the internal communcation buffer.
///
/// \param[out]  aInputBuffer: Input buffer.
/// \param[out]  aOutputBuffer: Output buffer.
///
/// \return  Buffer size.
///

/// \author  Vincent Simard Bilodeau
///
/// \since   April 2016
// *****************************************************************************

uint16_t
LdConnectionUniversalModbus::InternalBuffers( uint8_t *( &aInputBuffer ), uint8_t *( &aOutputBuffer ) )
{
    aInputBuffer = mTransferInputBuffer;
    aOutputBuffer = mTransferOutputBuffer;
    return ( MODBUS_MAX_SIZE - 9 );
}


// *****************************************************************************
// Function: LdConnectionUniversalModbus::Reset
//
/// \brief   Perform a device reset.
///
/// \param[in]  aEnterBootloader: Enter bootloader flag.
///
/// \author  Vincent Simard Bilodeau
///
/// \since   April 2016
// *****************************************************************************

void
LdConnectionUniversalModbus::Reset( LeddarDefines::eResetType /*aType*/, bool aEnterBootloader )
{
    mTransferInputBuffer[ 0 ] = ( aEnterBootloader ? 0x82 : 0 );
    Write( REGMAP_SWRST, 0, 1 );
}
