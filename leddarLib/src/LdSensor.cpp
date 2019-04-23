// *****************************************************************************
// Module..: Leddar
//
/// \file    LdSensor.cpp
///
/// \brief   Definition of all sensors.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdSensor.h"
#include "LdConnection.h"
#include "LdIntegerProperty.h"
#include "LdPropertyIds.h"
#include "comm/LtComLeddarTechPublic.h"

using namespace LeddarDevice;

// *****************************************************************************
// Function: LdSensor::LdSensor
//
/// \brief   Constructor - Take ownership of aConnection (and the 2 pointers used to build it)
///
/// \param   aConnection Connection object.
/// \param   aProperties Properties container for this sensor.
///                      If null, the sensor will declare a new instance of LdPropertiesContainer.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

LdSensor::LdSensor( LeddarConnection::LdConnection *aConnection, LeddarCore::LdPropertiesContainer *aProperties ) :
    LdDevice( aConnection, aProperties ),
    mEchoes(),
    mStates(),
    mDataMask( 0 ),
    mCarrier( nullptr )
{

}

// *****************************************************************************
// Function: LdSensorVuLdSensor::~LdSensor
//
/// \brief   Destructor.
///
/// \author  David Levy
///
/// \since   June 2017
// *****************************************************************************
LdSensor::~LdSensor()
{
    if( mCarrier )
    {
        delete mCarrier;
        mCarrier = nullptr;
    }
}

// *****************************************************************************
// Function: LdSensor::GetData
//
/// \brief   Get the data from the sensor.
///          The function SetDataMask must be call first to set the data level.
///
/// \return  True is new data was processed, otherwise false.
///
/// \author  Patrick Boulay
///
/// \since   March 2017
// *****************************************************************************
bool
LdSensor::GetData( void )
{
    bool lDataReceived = false;

    if( ( mDataMask & DM_ECHOES ) == DM_ECHOES )
    {
        lDataReceived = GetEchoes();
    }

    if( ( mDataMask & DM_STATES ) == DM_STATES )
    {
        GetStates();
        lDataReceived = true;
    }

    return lDataReceived;
}

// *****************************************************************************
// Function: LdSensor::SetCarrier
//
/// \brief   Define the carrier for modbus communication
///             Take ownership of aCarrier, and aCarrier release ownership of connection, because its not the actual sensor
///
/// \author  David Levy
///
/// \since   June 2017
// *****************************************************************************
void
LdSensor::SetCarrier( LdSensor *aCarrier )
{
    mCarrier = aCarrier;
    mCarrier->mDeleteConnection = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn uint32_t LeddarDevice::LdSensor::ConvertDataMaskToLTDataMask( uint32_t aMask )
///
/// \brief  Convert SDK data mask to LeddarTech internal data mask
///
/// \param  aMask   The mask.
///
/// \return The data converted mask to LeddarTech data mask.
///
/// \author David Levy
/// \date   August 2018
////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t LeddarDevice::LdSensor::ConvertDataMaskToLTDataMask( uint32_t aMask )
{
    uint32_t lLTDataMask = 0;

    if( ( aMask & DM_ECHOES ) == DM_ECHOES )
        lLTDataMask |= LtComLeddarTechPublic::LT_DATA_LEVEL_ECHOES;

    if( ( aMask & DM_STATES ) == DM_STATES )
        lLTDataMask |= LtComLeddarTechPublic::LT_DATA_LEVEL_STATE;

    return lLTDataMask;
}