// *****************************************************************************
// Module..: Leddar
//
/// \file    LdDeviceFactory.cpp
///
/// \brief   Factory to create devices.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdDeviceFactory.h"

#include "LdCarrierEnhancedModbus.h"
#include "LdSensorVu8.h"
#include "LdSensorOneModbus.h"
#include "LdSensorM16Modbus.h"
#include "LdSensorM16.h"
#include "LdSensorM16Laser.h"

#include "comm/LtComLeddarTechPublic.h"

using namespace LeddarDevice;

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn LdSensor * LdDeviceFactory::CreateSensor( LeddarConnection::LdConnection *aConnection )
///
/// \brief  Create device associated to a connection type.
///         The device will connect if its not already done to get the device type.
///         Take ownership of aConnection
///
/// \exception  std::invalid_argument   Thrown when aConnectionInfo is nullptr.
/// \exception  LtConnectionFailed      If the connection failed.
///
/// \param [in,out] aConnection Connection to the sensor
///
/// \return nullptr if it fails, else a pointer to the new sensor.
///
/// \author Patrick Boulay
/// \date   March 2016
////////////////////////////////////////////////////////////////////////////////////////////////////
LdSensor *
LdDeviceFactory::CreateSensor( LeddarConnection::LdConnection *aConnection )
{
    if( aConnection == nullptr )
    {
        throw std::invalid_argument( "Connection not valid." );
    }

    if( !aConnection->IsConnected() )
    {
        aConnection->Connect();
    }

    uint32_t lDeviceType = aConnection->GetDeviceType();

    if( lDeviceType == LtComLeddarTechPublic::LT_COMM_DEVICE_TYPE_VU8 )
    {
        LeddarDevice::LdSensor *lSensor = new LdSensorVu8( aConnection );

        if( aConnection->GetConnectionInfo()->GetType() == LeddarConnection::LdConnectionInfo::CT_LIB_MODBUS )
        {
            lSensor->SetCarrier( new LdCarrierEnhancedModbus( aConnection, lSensor->GetProperties() ) );
        }

        return lSensor;
    }
    else if( LtComLeddarTechPublic::LT_COMM_DEVICE_TYPE_SCH_EVALKIT == lDeviceType || LtComLeddarTechPublic::LT_COMM_DEVICE_TYPE_SCH_LONG_RANGE == lDeviceType )
    {
        return new LdSensorOneModbus( aConnection );
    }
    else if( lDeviceType == LtComLeddarTechPublic::LT_COMM_DEVICE_TYPE_M16_EVALKIT || lDeviceType == LtComLeddarTechPublic::LT_COMM_DEVICE_TYPE_IS16 ||
             lDeviceType == LtComLeddarTechPublic::LT_COMM_DEVICE_TYPE_M16 )
    {
        if( aConnection->GetConnectionInfo()->GetType() == LeddarConnection::LdConnectionInfo::CT_USB )
            return new LdSensorM16( aConnection );
        else
            return new LdSensorM16Modbus( aConnection );

    }
    else if( lDeviceType == LtComLeddarTechPublic::LT_COMM_DEVICE_TYPE_M16_LASER )
    {
        if( aConnection->GetConnectionInfo()->GetType() == LeddarConnection::LdConnectionInfo::CT_USB )
            return new LdSensorM16Laser( aConnection );
        else
            return new LdSensorM16Modbus( aConnection );
    }

    return nullptr;
}
