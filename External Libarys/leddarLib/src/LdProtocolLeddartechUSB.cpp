// *****************************************************************************
// Module..: LeddarPrivate
//
/// \file    LdProtocolLeddartechUSB.h
///
/// \brief   Class definition of LdProtocolLeddartechUSB
///
/// \author  Patrick Boulay
///
/// \since   February 2017
//
// Copyright (c) 2017 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdProtocolLeddartechUSB.h"

#include "comm/LtComUSBPublic.h"
#include "comm/LtComLeddarTechPublic.h"

#include "LdInterfaceUsb.h"
#include "LtExceptions.h"
#include "LtStringUtils.h"

#include <cstring>

using namespace LeddarConnection;

// *****************************************************************************
// Function: LdProtocolLeddartechUSB::LdProtocolLeddartechUSB
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

LdProtocolLeddartechUSB::LdProtocolLeddartechUSB( const LdConnectionInfo *aConnectionInfo, LdConnection *aInterface ) :
    LdProtocolLeddarTech( aConnectionInfo, aInterface ),
    mEndPoint( EP_CONFIG )
{
    mInterfaceUSB = dynamic_cast< LdInterfaceUsb * >( aInterface );
}

// *****************************************************************************
// Function: LdProtocolLeddartechUSB::LdProtocolLeddartechUSB
//
/// \brief   Constructor. Use this constructor for other endpoints to share the same connection objects.
///
/// \param  aConnectionInfo Connection information.
/// \param  aProtocol     Interface to connect to (with a different protocol)
/// \param  aEndPoint       End point of the protocol.
///
/// \author  Patrick Boulay
///
/// \since   March 2017
// *****************************************************************************

LdProtocolLeddartechUSB::LdProtocolLeddartechUSB( const LdConnectionInfo *aConnectionInfo, LdConnection *aProtocol, eEndPoint aEndPoint ) :
    LdProtocolLeddarTech( aConnectionInfo, aProtocol ),
    mEndPoint( aEndPoint )
{
    mDeviceType = aProtocol->GetDeviceType();
    mIsConnected = aProtocol->IsConnected();
    mInterfaceUSB = dynamic_cast<LdInterfaceUsb *>( aProtocol->GetInterface() );
}

// *****************************************************************************
// Function: LdProtocolLeddartechUSB::~LdProtocolLeddartechUSB
//
/// \brief   Destructor
///
/// \author  Patrick Boulay
///
/// \since   February 2017
// *****************************************************************************

LdProtocolLeddartechUSB::~LdProtocolLeddartechUSB( void )
{
}

// *****************************************************************************
// Function: LdProtocolLeddartechUSB::Write
/// \brief   Write to device
///
/// \param   aSize Size of the data to send.
///
/// \author  David Levy
///
/// \since   March 2017
// *****************************************************************************
void LdProtocolLeddartechUSB::Write( uint32_t aSize )
{
    mInterfaceUSB->Write( mEndPoint, mTransferInputBuffer, aSize );
}

// *****************************************************************************
// Function: LdProtocolLeddartechUSB::Read
/// \brief   Read from device
///
/// \param   aSize Size of the data to received. Not used with USB.
///
/// \author  David Levy
///
/// \since   March 2017
// *****************************************************************************
void LdProtocolLeddartechUSB::Read( uint32_t /*aSize*/ )
{
    mInterfaceUSB->Read( mEndPoint, mTransferOutputBuffer, mTransferBufferSize );
}

// *****************************************************************************
// Function: LdProtocolLeddartechUSB::QueryDeviceInfo
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
LdProtocolLeddartechUSB::QueryDeviceInfo( void )
{
    VerifyConnection();

    LtComUSBPublic::LtComUsbIdtAnswerIdentify lInfo;
    LeddarConnection::LdInterfaceUsb *lUsbInterface = dynamic_cast< LdInterfaceUsb * >( mInterface );
    lUsbInterface->ControlTransfert( 0xC0, LtComUSBPublic::LT_COM_USB_SETUP_REQ_CMD_IDENTIFY, reinterpret_cast<uint8_t *>( &lInfo ),
                                     static_cast<uint32_t>( sizeof( lInfo ) ), 1000 );

    mIdentityInfo.mDeviceName = lInfo.mDeviceName;
    mIdentityInfo.mDeviceSerialNumber = lInfo.mSerialNumber;
    mIdentityInfo.mDeviceType = lInfo.mDeviceType;
    mDeviceType = lInfo.mDeviceType;
}

// *****************************************************************************
// Function: LdProtocolLeddartechUSB::ReadAnswer
//
/// \brief   Read the header of the answer of the previous request
///
/// \throw   Throw LdComException, see VerifyConnection
///
/// \author  Patrick Boulay
///
/// \since   February 2017
// *****************************************************************************

void
LdProtocolLeddartechUSB::ReadAnswer( void )
{
    VerifyConnection();

    LtComLeddarTechPublic::sLtCommAnswerHeader *lHeader = reinterpret_cast<LtComLeddarTechPublic::sLtCommAnswerHeader *>( mTransferOutputBuffer );

    Read( sizeof( LtComLeddarTechPublic::sLtCommAnswerHeader ) );

    //if ( lHeader->mSrvProtVersion != mProtocolVersion )
    //{
    //    throw LeddarException::LtComException( "Read answer from a different protocol version, expected: " + LeddarUtils::LtStringUtils::IntToString( mProtocolVersion ) + " received: " + LeddarUtils::LtStringUtils::IntToString( lHeader->mSrvProtVersion ) );
    //}
    if( lHeader->mRequestCode != mRequestCode )
    {
        throw LeddarException::LtComException( "Received a different request code than the request, expected: " + LeddarUtils::LtStringUtils::IntToString(
                mRequestCode ) + " received: " + LeddarUtils::LtStringUtils::IntToString( lHeader->mRequestCode ) );
    }

    mAnswerCode = lHeader->mAnswerCode;
    mMessageSize = lHeader->mAnswerSize - sizeof( LtComUSBPublic::sLtCommAnswerHeader );
    mElementOffset = sizeof( LtComUSBPublic::sLtCommAnswerHeader );
}