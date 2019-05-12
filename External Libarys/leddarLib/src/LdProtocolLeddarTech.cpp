// *****************************************************************************
// Module..: LeddarPrivate
//
/// \file    LdProtocolLeddarTech.h
///
/// \brief   Class definition of LdProtocolLeddarTech
///
/// \author  Patrick Boulay
///
/// \since   February 2017
//
// Copyright (c) 2017 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdProtocolLeddarTech.h"

#include "comm/LtComLeddarTechPublic.h"

#include "LdInterfaceUsb.h"
#include "LtExceptions.h"
#include "LtStringUtils.h"

#include <cstring>

using namespace LeddarConnection;

// *****************************************************************************
// Function: LdProtocolLeddarTech::LdProtocolLeddarTech
//
/// \brief   Constructor
///
/// \param  aConnectionInfo Connection information
/// \param  aInterface      Interface to connect to
///
/// \author  Patrick Boulay
///
/// \since   February 2017
// *****************************************************************************

LdProtocolLeddarTech::LdProtocolLeddarTech( const LdConnectionInfo *aConnectionInfo, LdConnection *aInterface ) :
    LdConnection( aConnectionInfo, aInterface ),
    mIsConnected( false ),
    mIsDataServer( false ),
    mIdentityInfo(),
    mProtocolVersion( LT_COMM_CFG_PROT_VERSION ),
    mAnswerCode( 0 ),
    mRequestCode( 0 ),
    mMessageSize( 0 ),
    mTotalMessageSize( nullptr ),
    mElementOffset( 0 ),
    mElementId( 0 ),
    mElementCount( 0 ),
    mElementSize( 0 ),
    mElementValueOffset( 0 )
{
    mTransferBufferSize = 19000;
    mTransferInputBuffer = new uint8_t[mTransferBufferSize];
    mTransferOutputBuffer = new uint8_t[mTransferBufferSize];
}

// *****************************************************************************
// Function: LdProtocolLeddarTech::~LdProtocolLeddarTech
//
/// \brief   Destructor
///
/// \author  Patrick Boulay
///
/// \since   February 2017
// *****************************************************************************

LdProtocolLeddarTech::~LdProtocolLeddarTech( void )
{
    if( mTransferInputBuffer != nullptr )
    {
        delete[] mTransferInputBuffer;
        mTransferInputBuffer = nullptr;
    }

    if( mTransferOutputBuffer != nullptr )
    {
        delete[] mTransferOutputBuffer;
        mTransferOutputBuffer = nullptr;
    }
}

// *****************************************************************************
// Function: LdProtocolLeddarTech::Connect
//
/// \brief   Establish a connection with de device.
///
/// \author  Patrick Boulay
///
/// \since   February 2017
// *****************************************************************************

void
LdProtocolLeddarTech::Connect( void )
{
    try
    {
        // Connect interface
        mInterface->Connect();
        mIsConnected = true;

        // Query device info
        QueryDeviceInfo();

    }
    catch( std::exception )
    {
        throw;
    }
}

// *****************************************************************************
// Function: LdProtocolLeddarTech::Disconnect
//
/// \brief   Disconnect the device.
///
/// \author  Patrick Boulay
///
/// \since   February 2017
// *****************************************************************************

void
LdProtocolLeddarTech::Disconnect( void )
{

    if( mInterface != nullptr && IsConnected() )
    {
        mInterface->Disconnect();
        mIsConnected = false;
    }

}

// *****************************************************************************
// Function: LdProtocolLeddarTech::VerifyConnection
//
/// \brief   Verify if there is an interface and if the interface is connected.
///
/// \throw   LtComException If the device is not connected.
///
/// \author  Patrick Boulay
///
/// \since   February 2017
// *****************************************************************************

void
LdProtocolLeddarTech::VerifyConnection( void ) const
{
    if( mInterface == nullptr )
    {
        throw LeddarException::LtComException( "No communication interface assign for this protocol." );
    }

    if( !IsConnected() )
    {
        throw LeddarException::LtComException( "Device not connected." );
    }
}

// *****************************************************************************
// Function: LdProtocolLeddarTech::StartRequest
//
/// \brief   Start to prepare a request to device
///
/// \param   aCode Request code
///
/// \throw   Throw LdComException, see VerifyConnection
///
/// \author  Patrick Boulay
///
/// \since   February 2017
// *****************************************************************************
void
LdProtocolLeddarTech::StartRequest( uint16_t aCode )
{
    VerifyConnection();

    LtComLeddarTechPublic::sLtCommRequestHeader *lRequest = reinterpret_cast<LtComLeddarTechPublic::sLtCommRequestHeader *>( mTransferInputBuffer );
    lRequest->mRequestCode = aCode;
    lRequest->mRequestTotalSize = sizeof( LtComLeddarTechPublic::sLtCommRequestHeader );
    lRequest->mSrvProtVersion = mProtocolVersion;
    mTotalMessageSize = &lRequest->mRequestTotalSize;
    mMessageSize = sizeof( LtComLeddarTechPublic::sLtCommRequestHeader );
    mElementOffset = sizeof( LtComLeddarTechPublic::sLtCommRequestHeader );
    mRequestCode = aCode;
}

// *****************************************************************************
// Function: LdProtocolLeddarTech::ReadRequest
//
/// \brief   Read request on de device
///
/// \throw   Throw LdComException, see VerifyConnection
///
/// \author  Patrick Boulay
///
/// \since   February 2017
// *****************************************************************************
void
LdProtocolLeddarTech::ReadRequest()
{
    VerifyConnection();

    // Read the header
    Read( sizeof( LtComLeddarTechPublic::sLtCommRequestHeader ) );
    LtComLeddarTechPublic::sLtCommRequestHeader *lRequestHeader = reinterpret_cast<LtComLeddarTechPublic::sLtCommRequestHeader *>( mTransferOutputBuffer );

    mRequestCode = lRequestHeader->mRequestCode;
    mMessageSize = lRequestHeader->mRequestTotalSize - sizeof( LtComLeddarTechPublic::sLtCommRequestHeader );
    mElementOffset = sizeof( LtComLeddarTechPublic::sLtCommRequestHeader );
}

// *****************************************************************************
// Function: LdProtocolLeddarTech::SendRequest
//
/// \brief   Send request to device.
///
/// \throw   Throw LdComException, see VerifyConnection
///
/// \author  Patrick Boulay
///
/// \since   February 2017
// *****************************************************************************
void
LdProtocolLeddarTech::SendRequest( void )
{
    VerifyConnection();

    Write( static_cast<uint32_t>( mMessageSize ) );
}

// *****************************************************************************
// Function: LdProtocolLeddarTech::AddElement
//
/// \brief   Add element in the buffer to send to device
///
/// \param   aId Id of the element
/// \param   aCount Count of the element array
/// \param   aSize Size of one element in the array
/// \param   aData Data buffer to send
/// \param   aStride  Number of bytes between values.
///
/// \throw   Throw LdComException, see VerifyConnection
///
/// \author  Patrick Boulay
///
/// \since   February 2017
// *****************************************************************************

void
LdProtocolLeddarTech::AddElement( uint16_t aId, uint16_t aCount, uint32_t aSize, const void *aData, uint32_t aStride )
{
    VerifyConnection();

    const uint32_t lAddedSize = aCount * aSize + sizeof( LtComLeddarTechPublic::sLtCommElementHeader );

    if( lAddedSize + mMessageSize > mTransferBufferSize )
    {
        // If the buffer is resized, we need to update the pointer to the total message size
        uint32_t lOffsetMessageSize = static_cast<uint32_t>( reinterpret_cast<uint8_t *>( mTotalMessageSize ) - mTransferInputBuffer );
        ResizeInternalBuffers( mTransferBufferSize + lAddedSize );
        mTotalMessageSize = reinterpret_cast<uint32_t *>( mTransferInputBuffer + lOffsetMessageSize );
    }

    LtComLeddarTechPublic::sLtCommElementHeader *lElementHeader = reinterpret_cast<LtComLeddarTechPublic::sLtCommElementHeader *>
            ( mTransferInputBuffer + mElementOffset );

    lElementHeader->mElementId = aId;
    lElementHeader->mElementCount = aCount;
    lElementHeader->mElementSize = aSize;
    mMessageSize += lAddedSize;
    mElementOffset += sizeof( LtComLeddarTechPublic::sLtCommElementHeader );

    if( aStride == aSize )
    {
        memcpy( mTransferInputBuffer + mElementOffset, aData, aCount * aSize );
    }
    else
    {
        switch( aSize )
        {
            case 1:
            {
                const uint8_t *lSrc = static_cast<const uint8_t *>( aData );
                uint8_t *lDest = mTransferInputBuffer + mElementOffset;

                for( uint16_t i = 0; i < aCount; ++i )
                {
                    lDest[ i ] = *lSrc;
                    lSrc += aStride;
                }
            }
            break;

            case 2:
            {
                const uint16_t *lSrc = static_cast<const uint16_t *>( aData );
                uint16_t *lDest = reinterpret_cast<uint16_t *>( mTransferInputBuffer + mElementOffset );

                for( uint16_t i = 0; i < aCount; ++i )
                {
                    lDest[ i ] = *lSrc;
                    lSrc = reinterpret_cast<const uint16_t *>( reinterpret_cast<const uint8_t *>( lSrc ) + aStride );
                }
            }
            break;

            case 4:
            {
                const uint32_t *lSrc = static_cast<const uint32_t *>( aData );
                uint32_t *lDest = reinterpret_cast<uint32_t *>( mTransferInputBuffer + mElementOffset );

                for( uint16_t i = 0; i < aCount; ++i )
                {
                    lDest[ i ] = *lSrc;
                    lSrc = reinterpret_cast<const uint32_t *>( reinterpret_cast<const uint8_t *>( lSrc ) + aStride );
                }
            }
            break;

            case 8:
            {
                const uint64_t *lSrc = static_cast<const uint64_t *>( aData );
                uint64_t *lDest = reinterpret_cast<uint64_t *>( mTransferInputBuffer + mElementOffset );

                for( uint16_t i = 0; i < aCount; ++i )
                {
                    lDest[ i ] = *lSrc;
                    lSrc = reinterpret_cast<const uint64_t *>( reinterpret_cast<const uint8_t *>( lSrc ) + aStride );
                }
            }
            break;

            default:
            {
                const uint8_t *lSrc = static_cast<const uint8_t *>( aData );
                uint8_t *lDest = mTransferInputBuffer + mElementOffset;

                for( uint16_t i = 0; i < aCount; ++i )
                {
                    memcpy( lDest, lSrc, aSize );
                    lDest += aSize;
                    lSrc += aStride;
                }
            }
            break;
        }
    }

    mElementOffset += aCount * aSize;
    *mTotalMessageSize += lAddedSize;
}


// *****************************************************************************
// Function: LdProtocolLeddarTech::ReadElement
//
/// \brief   Read a single element
///
/// \return  Return true if there is other element to read
///
/// \throw   Throw LdComException, see VerifyConnection
///
/// \author  Patrick Boulay
///
/// \since   February 2017
// *****************************************************************************

bool
LdProtocolLeddarTech::ReadElement( void )
{
    VerifyConnection();

    if( mMessageSize  > 0 )
    {
        LtComLeddarTechPublic::sLtCommElementHeader *lElementHeader = reinterpret_cast<LtComLeddarTechPublic::sLtCommElementHeader *>
                ( mTransferOutputBuffer + mElementOffset );
        mElementSize = lElementHeader->mElementSize;
        mElementId = lElementHeader->mElementId;
        mElementCount = lElementHeader->mElementCount;
        mElementValueOffset = mElementOffset + sizeof( LtComLeddarTechPublic::sLtCommElementHeader );
        mElementOffset += sizeof( LtComLeddarTechPublic::sLtCommElementHeader ) + ( mElementSize * mElementCount );
        mMessageSize -= sizeof( LtComLeddarTechPublic::sLtCommElementHeader ) + ( mElementSize * mElementCount );
    }
    else
    {
        // no more element
        return false;
    }

    return true;
}

// *****************************************************************************
// Function: LdProtocolLeddarTech::GetElementData
//
/// \brief   Return the pointer to data buffer of the current element
///
/// \return  Return the pointer to data buffer
///
/// \author  Patrick Boulay
///
/// \since   March 2017
// *****************************************************************************

void *
LdProtocolLeddarTech::GetElementData( void ) const
{
    return mTransferOutputBuffer + mElementValueOffset;
}

// *****************************************************************************
// Function: LdProtocolLeddarTech::ReadElementToProperty
//
/// \brief   Read a single element and store it in a property
///
/// \param   aProperties Properties container
///
/// \return  Return true if there is other element to read
///
/// \throw   Throw LdComException, see VerifyConnection
///
/// \author  Patrick Boulay
///
/// \since   February 2017
// *****************************************************************************

bool
LdProtocolLeddarTech::ReadElementToProperty( LeddarCore::LdPropertiesContainer *aProperties )
{
    if( ReadElement() )
    {
        LeddarCore::LdProperty *lProperty = aProperties->FindDeviceProperty( mElementId );

        if( lProperty != nullptr )
        {
            lProperty->SetCount( mElementCount );
            lProperty->SetRawStorage( mTransferOutputBuffer + mElementValueOffset, mElementCount, mElementSize );
        }
    }
    else
    {
        return false;
    }

    return true;
}

// *****************************************************************************
// Function: LdProtocolLeddarTech::ReadElementToProperties
//
/// \brief   Read all elements and store it in a property
///
/// \param   aProperties Properties container
///
/// \return  Return true if there is other element to read
///
/// \throw   Throw LdComException, see VerifyConnection
///
/// \author  Patrick Boulay
///
/// \since   February 2017
// *****************************************************************************

void
LdProtocolLeddarTech::ReadElementToProperties( LeddarCore::LdPropertiesContainer *aProperties )
{
    while( ReadElementToProperty( aProperties ) )
    {
    }
}

// *****************************************************************************
// Function: LdProtocolLeddarTech::PushElementDataToBuffer
//
/// \brief   Push the element data into the struture passed throught the destination buffer.
///
/// \param   aDest Destination buffer to fill.
/// \param   aCount Number of element.
/// \param   aSize Size of each element.
/// \param   aStride Spacing in bytes between each value.
///
/// \return  Return true if there is other element to read
///
/// \throw   Throw LdComException, see VerifyConnection
///
/// \author  Patrick Boulay
///
/// \since   February 2017
// *****************************************************************************

void
LdProtocolLeddarTech::PushElementDataToBuffer( void *aDest, uint16_t aCount, uint32_t aSize, size_t aStride )
{
    if( ( aCount != mElementCount )
            || ( aSize != mElementSize ) )
    {
        throw LeddarException::LtComException( "Unable to push the element in the buffer, count or size do not match." );
    }

    if( aStride == mElementSize )
    {
        memcpy( aDest, GetElementData(),
                mElementCount * mElementSize );
    }
    else
    {
        switch( mElementSize )
        {
            case 1:
            {
                const uint8_t *const lSrc = static_cast<const uint8_t *>( GetElementData() );
                uint8_t *lDest = static_cast<uint8_t *>( aDest );

                for( uint16_t i = 0; i < mElementCount; ++i )
                {
                    *lDest = lSrc[i];
                    lDest += aStride;
                }
            }
            break;

            case 2:
            {
                const uint16_t *const lSrc = reinterpret_cast<const uint16_t *>( GetElementData() );
                uint16_t *lDest = reinterpret_cast<uint16_t *>( aDest );

                for( uint16_t i = 0; i < mElementCount; ++i )
                {
                    *lDest = lSrc[i];
                    lDest = reinterpret_cast<uint16_t *>( reinterpret_cast<uint8_t *>( lDest ) + aStride );
                }
            }
            break;

            case 4:
            {
                const uint32_t *const lSrc = reinterpret_cast<const uint32_t *>( GetElementData() );
                uint32_t *lDest = reinterpret_cast<uint32_t *>( aDest );

                for( uint16_t i = 0; i < mElementCount; ++i )
                {
                    *lDest = lSrc[i];
                    lDest = reinterpret_cast<uint32_t *>( reinterpret_cast<uint8_t *>( lDest ) + aStride );
                }
            }
            break;

            case 8:
            {
                const uint64_t *const lSrc = reinterpret_cast<const uint64_t *>( GetElementData() );
                uint64_t *lDest = reinterpret_cast<uint64_t *>( aDest );

                for( uint16_t i = 0; i < mElementCount; ++i )
                {
                    *lDest = lSrc[i];
                    lDest = reinterpret_cast<uint64_t *>( reinterpret_cast<uint8_t *>( lDest ) + aStride );
                }
            }
            break;

            default:
            {
                const uint8_t *const lSrc = static_cast<const uint8_t *>( GetElementData() );
                uint8_t *lDest = static_cast<uint8_t *>( aDest );

                for( uint16_t i = 0; i < mElementCount; ++i )
                {
                    memcpy( lDest, lSrc + i * mElementSize, mElementSize );
                    lDest += aStride;
                }
            }
            break;
        }
    }
}