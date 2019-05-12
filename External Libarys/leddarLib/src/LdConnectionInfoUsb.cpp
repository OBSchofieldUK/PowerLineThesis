// *****************************************************************************
// Module..: Leddar
//
/// \file    LdConnectionInfoUsb.cpp
///
/// \brief   Connection information on Usb devices.
///
/// \author  David Levy
///
/// \since   January 2017
//
// Copyright (c) 2017 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdConnectionInfoUsb.h"

#include "comm/LtComUSBPublic.h"

using namespace LeddarConnection;

// *****************************************************************************
// Function: LdConnectionInfoUsb::LdConnectionInfoUsb
//
/// \brief   Constructor.
///
/// \param  aConnectionType Type of connection
/// \param  aDisplayName    Name to display for this device.
/// \param  aVendorID       Vendor ID of the device.
/// \param  aProductID      Product ID of the device.
/// \param  aBusNumber      Bus number.
/// \param  aDeviceAddress  Device address.
/// \param  aAlreadyOpen    If the connection is busy, opened by an other connection.
/// \param  aInfos          USB identity of the device
///
/// \author  David Levy
///
/// \since   January 2017
// *****************************************************************************
LdConnectionInfoUsb::LdConnectionInfoUsb( eConnectionType aConnectionType, const std::string &aDisplayName, uint16_t aVendorID, uint16_t aProductID,
        uint8_t aBusNumber, uint8_t aDeviceAddress, const std::string& aSerialNumber, LtComUSBPublic::LtComUsbIdtAnswerIdentify &aInfos, bool aAlreadyopen ) :
    LdConnectionInfo( aConnectionType, aDisplayName ),
    mVendorID( aVendorID ),
    mProductID( aProductID ),
    mBusNumber( aBusNumber ),
    mDeviceAddress( aDeviceAddress ),
    mSerialNumber( aSerialNumber ),
    mAlreadyOpen( aAlreadyopen ),
    mInfos( nullptr )
{
    mInfos = new LtComUSBPublic::LtComUsbIdtAnswerIdentify();
    *mInfos = aInfos;
}

// *****************************************************************************
// Function: LdConnectionInfoUsb::~LdConnectionInfoUsb
//
/// \brief   Destructor.
///
/// \author  David Levy
///
/// \since   January 2017
// *****************************************************************************
LdConnectionInfoUsb::~LdConnectionInfoUsb()
{
    delete mInfos;
}


// *****************************************************************************
// Function: LdConnectionInfoUsb::GetInfos
//
/// \brief   Get identify info.
///
/// \return Pointerto identify info
///
/// \author  David Levy
///
/// \since   January 2017
// *****************************************************************************
LtComUSBPublic::LtComUsbIdtAnswerIdentify LeddarConnection::LdConnectionInfoUsb::GetInfos( void ) const
{
    return *mInfos;
}
