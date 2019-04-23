// *****************************************************************************
// Module..: Leddar
//
/// \file    LdCarrierEnhancedModbus.cpp
///
/// \brief   Definition of the LeddarVu8 carrier board connecting by Modbus
///
/// \author  Patrick Boulay
///
/// \since   September 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdCarrierEnhancedModbus.h"

#include "LdConnection.h"
#include "LdConnectionInfoModbus.h"
#include "LdConnectionModbusStructures.h"
#include "LdPropertyIds.h"

#include <sstream>
#include <cstring>


// *****************************************************************************
// Function: LdCarrierEnhancedModbus::LdCarrierEnhancedModbus
//
/// \brief   Constructor - Take ownership of aConnection (and the 2 pointers used to build it)
///
/// \param   aConnection
/// \param   aProperties Properties set  if already define, 0 to create a new set
///
/// \author  Patrick Boulay
///
/// \since   September 2016
// *****************************************************************************

LeddarDevice::LdCarrierEnhancedModbus::LdCarrierEnhancedModbus( LeddarConnection::LdConnection *aConnection, LeddarCore::LdPropertiesContainer *aProperties ) :
    LdSensor( aConnection, aProperties )
{
    mModbusConnection = dynamic_cast< LeddarConnection::LdConnectionUniversalModbus * >( aConnection );
}

// *****************************************************************************
// Function: LdCarrierEnhancedModbus::GetConstants
//
/// \brief   Get contants of the carrier.
///
/// \author  Patrick Boulay
/// \author  Frédéric Parent
///
/// \since   September 2016
// *****************************************************************************

void
LeddarDevice::LdCarrierEnhancedModbus::GetConstants( void )
{
    LeddarConnection::LdConnectionModbuStructures::sModbusPacket lPacket = {};
    uint32_t lInSize, lOutSize;

    // Get connection
    const LeddarConnection::LdConnectionInfoModbus *lConnectionInfo = dynamic_cast< const LeddarConnection::LdConnectionInfoModbus * >( GetConnection()->GetConnectionInfo() );
    LeddarConnection::LdInterfaceModbus *lInterfaceModbus = dynamic_cast< LeddarConnection::LdInterfaceModbus * >( mModbusConnection->GetInterface() );

    // Get carrier part number
    lOutSize = static_cast<uint32_t>( sizeof( LeddarConnection::LdConnectionModbuStructures::sModbusHeader ) +
                                      sizeof( LeddarConnection::LdConnectionModbuStructures::sModbusGetCarrierDeviceInfoReq ) );
    lInSize = static_cast<uint32_t>( sizeof( LeddarConnection::LdConnectionModbuStructures::sModbusHeader ) +
                                     sizeof( LeddarConnection::LdConnectionModbuStructures::sModbusGetCarrierDeviceInfoAnswer ) +
                                     MODBUS_CRC_SIZE );

    lPacket.mHeader.mModbusAddress                          = lConnectionInfo->GetModbusAddr();
    lPacket.mHeader.mFunctionCode                           = 0x45;
    lPacket.uRequest.mGetCarrierDeviceInfo.mSubFunctionCode = 3;

    lInterfaceModbus->SendRawRequest( ( uint8_t * )&lPacket, lOutSize );
    lInterfaceModbus->ReceiveRawConfirmation( ( uint8_t * )&lPacket, lInSize );

    mProperties->GetTextProperty( LeddarCore::LdPropertyIds::ID_CARRIER_PART_NUMBER )->SetValue( 0, lPacket.uAnswer.mGetCarrierDeviceInfo.mCarrierDeviceInfo.mHardwarePartNumber );
    mProperties->GetTextProperty( LeddarCore::LdPropertyIds::ID_CARRIER_SERIAL_NUMBER )->SetValue( 0, lPacket.uAnswer.mGetCarrierDeviceInfo.mCarrierDeviceInfo.mHardwareSerialNumber );
    mProperties->GetBitProperty( LeddarCore::LdPropertyIds::ID_CARRIER_OPTIONS )->SetValue( 0, lPacket.uAnswer.mGetCarrierDeviceInfo.mCarrierDeviceInfo.mCarrierDeviceOption );

    // Get carrier firmware version
    lOutSize = static_cast<uint32_t>( sizeof( LeddarConnection::LdConnectionModbuStructures::sModbusHeader ) +
                                      sizeof( LeddarConnection::LdConnectionModbuStructures::sModbusGetCarrierFirmwareInfoReq ) );
    lInSize = static_cast<uint32_t>( sizeof( LeddarConnection::LdConnectionModbuStructures::sModbusHeader ) +
                                     sizeof( LeddarConnection::LdConnectionModbuStructures::sModbusGetCarrierFirmwareInfoAnswer ) +
                                     MODBUS_CRC_SIZE );

    memset( &lPacket, 0, sizeof( lPacket ) );
    lPacket.mHeader.mModbusAddress                          = lConnectionInfo->GetModbusAddr();
    lPacket.mHeader.mFunctionCode                           = 0x45;
    lPacket.uRequest.mGetCarrierDeviceInfo.mSubFunctionCode = 2;

    lInterfaceModbus->SendRawRequest( ( uint8_t * )&lPacket, lOutSize );
    lInterfaceModbus->ReceiveRawConfirmation( ( uint8_t * )&lPacket, lInSize );

    mProperties->GetTextProperty( LeddarCore::LdPropertyIds::ID_CARRIER_SOFTWARE_PART_NUMBER )->SetValue( 0, lPacket.uAnswer.mGetCarrierFirwwareInfo.mFirmwarePartNumber );
    std::ostringstream lStr;
    lStr << ( int )lPacket.uAnswer.mGetCarrierFirwwareInfo.mFirmwareVersion[0] << "." <<
         ( int )lPacket.uAnswer.mGetCarrierFirwwareInfo.mFirmwareVersion[1] << "." <<
         ( int )lPacket.uAnswer.mGetCarrierFirwwareInfo.mFirmwareVersion[2] << "." <<
         ( int )lPacket.uAnswer.mGetCarrierFirwwareInfo.mFirmwareVersion[3];
    mProperties->GetTextProperty( LeddarCore::LdPropertyIds::ID_CARRIER_FIRMWARE_VERSION )->SetValue( 0, lStr.str() );
}