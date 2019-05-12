/// *****************************************************************************
/// Module..: Leddar
///
/// \file    LdSensorM16Modbus.cpp
///
/// \brief   Definition of M16 sensor using modbus protocol
///
/// \author  David Levy
///
/// \since   September 2017
///
/// Copyright (c) 2017 LeddarTech Inc. All rights reserved.
/// *****************************************************************************

#include "LdSensorM16Modbus.h"

#include "comm/Modbus/LtComLeddarM16Modbus.h"

#include "LdPropertyIds.h"

#include "LtExceptions.h"
#include "LtIntUtilities.h"
#include "LtStringUtils.h"
#include "LtTimeUtils.h"

#include <string.h>

using namespace LeddarDevice;

/// *****************************************************************************
/// Function: LdSensorM16Modbus::LdSensorM16Modbus
///
/// \brief   Constructor - Take ownership of aConnection (and the 2 pointers used to build it)
///
/// \author  Patrick Boulay
///
/// \since   April 2017
/// *****************************************************************************
LdSensorM16Modbus::LdSensorM16Modbus( LeddarConnection::LdConnection *aConnection ) :
    LdSensor( aConnection )
{
    using namespace LeddarCore;

    mConnectionInfoModbus = dynamic_cast< const LeddarConnection::LdConnectionInfoModbus * >( aConnection->GetConnectionInfo() );
    mInterface = dynamic_cast< LeddarConnection::LdLibModbusSerial * >( aConnection );

    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_SERIAL_NUMBER, 0, 32 ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_DEVICE_NAME, 0, 32 ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_PART_NUMBER, 0, 32 ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_SOFTWARE_PART_NUMBER, 0, 32 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_FIRMWARE_VERSION_STR, 0, 2 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_FPGA_VERSION, 0, 2 ) );
    mProperties->AddProperty( new LdBitFieldProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_OPTIONS, 0, 4 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_DEVICE_TYPE, 0, 2 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONSTANT, LdProperty::F_NONE, LdPropertyIds::ID_DISTANCE_SCALE, 0, 4 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONSTANT, LdProperty::F_NONE, LdPropertyIds::ID_AMP_SCALE, 0, 4 ) );

    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_ACCUMULATION_EXP,
                              LtComLeddarM16Modbus::DID_ACCUMULATION_EXP, 2 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_OVERSAMPLING_EXP,
                              LtComLeddarM16Modbus::DID_OVERSAMPLING_EXP, 2 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_BASE_POINT_COUNT,
                              LtComLeddarM16Modbus::DID_BASE_POINT_COUNT, 2 ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_SENSIVITY,
                              LtComLeddarM16Modbus::DID_THRESHOLD_OFFSET, 2, M16_SENSITIVITY_SCALE, 3 ) );
    mProperties->AddProperty( new LdEnumProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_LED_INTENSITY,
                              LtComLeddarM16Modbus::DID_LED_INTENSITY, 1, false ) );
    mProperties->AddProperty( new LdBitFieldProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_ACQ_OPTIONS,
                              LtComLeddarM16Modbus::DID_ACQ_OPTIONS, 2 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_CHANGE_DELAY,
                              LtComLeddarM16Modbus::DID_CHANGE_DELAY, 2 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_COM_SERIAL_PORT_MAX_ECHOES,
                              LtComLeddarM16Modbus::DID_COM_SERIAL_PORT_MAX_ECHOES, 1 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_PRECISION,
                              LtComLeddarM16Modbus::DID_PRECISION, 2 ) );
    mProperties->AddProperty( new LdEnumProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_COM_SERIAL_PORT_ECHOES_RES,
                              LtComLeddarM16Modbus::DID_COM_SERIAL_PORT_ECHOES_RES, 1 ) );
    mProperties->AddProperty( new LdBitFieldProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_SEGMENT_ENABLE,
                              LtComLeddarM16Modbus::DID_SEGMENT_ENABLE, 2 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_COM_SERIAL_PORT_STOP_BITS,
                              LtComLeddarM16Modbus::DID_COM_SERIAL_PORT_STOP_BITS, 1 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_COM_SERIAL_PORT_PARITY,
                              LtComLeddarM16Modbus::DID_COM_SERIAL_PORT_PARITY, 1 ) );
    mProperties->AddProperty( new LdEnumProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_COM_SERIAL_PORT_BAUDRATE,
                              LtComLeddarM16Modbus::DID_COM_SERIAL_PORT_BAUDRATE, 2 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_COM_SERIAL_PORT_ADDRESS,
                              LtComLeddarM16Modbus::DID_COM_SERIAL_PORT_ADDRESS, 1 ) );


    // Set limits and enums
    GetProperties()->GetIntegerProperty( LdPropertyIds::ID_COM_SERIAL_PORT_ADDRESS )->SetLimits( 1, MODBUS_MAX_ADDR );
    GetProperties()->GetIntegerProperty( LdPropertyIds::ID_COM_SERIAL_PORT_MAX_ECHOES )->SetLimits( 1, M16_MAX_SERIAL_DETECTIONS );

    LeddarCore::LdEnumProperty *lSerialBaud = GetProperties()->GetEnumProperty( LdPropertyIds::ID_COM_SERIAL_PORT_BAUDRATE );
    lSerialBaud->AddEnumPair( 9600, "9600" );
    lSerialBaud->AddEnumPair( 19200, "19200" );
    lSerialBaud->AddEnumPair( 38400, "38400" );
    lSerialBaud->AddEnumPair( 57600, "57600" );
    lSerialBaud->AddEnumPair( 115200, "115200" );
    LeddarCore::LdEnumProperty *lSerialResolution = GetProperties()->GetEnumProperty( LdPropertyIds::ID_COM_SERIAL_PORT_ECHOES_RES );
    lSerialResolution->AddEnumPair( 1, "m" );
    lSerialResolution->AddEnumPair( 10, "dm" );
    lSerialResolution->AddEnumPair( 100, "cm" );
    lSerialResolution->AddEnumPair( 1000, "mm" );
    LeddarCore::LdEnumProperty *lLedPower = GetProperties()->GetEnumProperty( LdPropertyIds::ID_LED_INTENSITY );
    lLedPower->AddEnumPair( 10, "10" );
    lLedPower->AddEnumPair( 20, "20" );
    lLedPower->AddEnumPair( 35, "35" );
    lLedPower->AddEnumPair( 50, "50" );
    lLedPower->AddEnumPair( 65, "65" );
    lLedPower->AddEnumPair( 80, "80" );
    lLedPower->AddEnumPair( 90, "90" );
    lLedPower->AddEnumPair( 100, "100" );

    //States
    GetResultStates()->GetProperties()->AddProperty( new LdFloatProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_RS_SYSTEM_TEMP, 0, 4, 0, 1,
            "System Temperature" ) );
}

/// *****************************************************************************
/// Function: LdSensorM16Modbus::~LdSensorM16Modbus
///
/// \brief   Destructor.
///
/// \author  David Levy
///
/// \since   September 2017
/// *****************************************************************************
LdSensorM16Modbus::~LdSensorM16Modbus()
{
}

/// *****************************************************************************
/// Function: LdSensorM16Modbus::Connect
///
/// \brief   Connect to the sensor
///
/// \author  Patrick Boulay
///
/// \since   April 2017
/// *****************************************************************************
void
LdSensorM16Modbus::Connect( void )
{
    LdDevice::Connect();
}

/// *****************************************************************************
/// Function: LdSensorM16Modbus::GetEchoes
///
/// \brief   Get the echoes
///
/// \return  Return true if there is new echoes
///
/// \author  David Levy
///
/// \since   September 2017
/// *****************************************************************************
bool
LdSensorM16Modbus::GetEchoes( void )
{
    uint8_t lRawRequest[ 2 ] = { mConnectionInfoModbus->GetModbusAddr(), 0x41 };
    uint8_t lResponse[LTMODBUS_RTU_MAX_ADU_LENGTH] = { 0 };

    mInterface->SendRawRequest( lRawRequest, 2 );
    size_t lReceivedSize = mInterface->ReceiveRawConfirmationLT( lResponse, GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_DEVICE_TYPE )->ValueT<uint16_t>() );

    LeddarUtils::LtTimeUtils::Wait( M16_WAIT_AFTER_REQUEST );

    if( lReceivedSize <= MODBUS_DATA_OFFSET )
    {
        mInterface->Flush();
        throw LeddarException::LtComException( "Received size too small: " + LeddarUtils::LtStringUtils::IntToString( lReceivedSize ) );
    }

    uint8_t lEchoCount = lResponse[MODBUS_DATA_OFFSET];

    if( lReceivedSize > MODBUS_DATA_OFFSET + lEchoCount * 5u + 6 )
    {
        uint32_t lLastTimeStamp = mEchoes.GetTimestamp();
        uint32_t lTimeStamp = *reinterpret_cast<uint32_t *>( &lResponse[MODBUS_DATA_OFFSET + lEchoCount * 5u] );

        if( lLastTimeStamp != lTimeStamp )
        {
            mEchoes.SetEchoCount( lEchoCount );
            LtComLeddarM16Modbus::sLeddarM16Detections *lDetections = reinterpret_cast< LtComLeddarM16Modbus::sLeddarM16Detections * >( &lResponse[MODBUS_DATA_OFFSET + 1] );

            for( uint8_t i = 0; ( i < lEchoCount ) && ( i < M16_MAX_SERIAL_DETECTIONS ); ++i )
            {
                LeddarConnection::LdEcho &lEcho = ( *mEchoes.GetEchoes( LeddarConnection::B_SET ) )[i];
                lEcho.mDistance = lDetections->mDistance;
                lEcho.mAmplitude = lDetections->mAmplitude;
                lEcho.mFlag = lDetections->mFlags & 0x0F;
                lEcho.mChannelIndex = ( lDetections->mFlags & 0xF0 ) >> 4;
                lDetections++;
            }

            uint16_t lLedPower = *reinterpret_cast<uint16_t *>( &lResponse[MODBUS_DATA_OFFSET + lEchoCount * 5u + 4] );
            mEchoes.SetTimestamp( lTimeStamp );
            mEchoes.SetCurrentLedPower( lLedPower );
            mEchoes.Swap();
            mEchoes.UpdateFinished();
        }
        else
        {
            return false;
        }
    }
    else
    {
        mInterface->Flush();
        throw LeddarException::LtComException( "Not enough data received, size: " + LeddarUtils::LtStringUtils::IntToString( lReceivedSize ) );
    }

    return true;
}

/// *****************************************************************************
/// Function: LdSensorM16Modbus::GetStates
///
/// \brief   Get the states
///
/// \author  David Levy
///
/// \since   September 2017
/// *****************************************************************************
void
LdSensorM16Modbus::GetStates( void )
{
    uint16_t lResponse[LTMODBUS_RTU_MAX_ADU_LENGTH / 2] = { 0 };
    mInterface->ReadInputRegisters( 0, 1, lResponse );
    LeddarUtils::LtTimeUtils::Wait( M16_WAIT_AFTER_REQUEST );

    GetResultStates()->GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_RS_SYSTEM_TEMP )->SetRawValue( 0, lResponse[0] );
}

/// *****************************************************************************
/// Function: LdSensorM16Modbus::GetConfig
///
/// \brief   Get config properties from the sensor
///
/// \author  David Levy
///
/// \since   September 2017
/// *****************************************************************************
void
LdSensorM16Modbus::GetConfig( void )
{
    uint16_t lResponse[LTMODBUS_RTU_MAX_ADU_LENGTH / 2] = { 0 };
    mInterface->ReadRegisters( 0, 3, lResponse );
    LeddarUtils::LtTimeUtils::Wait( M16_WAIT_AFTER_REQUEST );
    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_ACCUMULATION_EXP )->SetValue( 0, lResponse[0] );
    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_ACCUMULATION_EXP )->SetClean();
    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_OVERSAMPLING_EXP )->SetValue( 0, lResponse[1] );
    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_OVERSAMPLING_EXP )->SetClean();
    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_BASE_POINT_COUNT )->SetValue( 0, lResponse[2] );
    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_BASE_POINT_COUNT )->SetClean();

    memset( lResponse, 0, LTMODBUS_RTU_MAX_ADU_LENGTH );
    mInterface->ReadRegisters( 4, 5, lResponse );
    LeddarUtils::LtTimeUtils::Wait( M16_WAIT_AFTER_REQUEST );
    GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_SENSIVITY )->SetRawValue( 0, lResponse[0] );
    GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_SENSIVITY )->SetClean();
    GetProperties()->GetEnumProperty( LeddarCore::LdPropertyIds::ID_LED_INTENSITY )->SetValue( 0, lResponse[1] );
    GetProperties()->GetEnumProperty( LeddarCore::LdPropertyIds::ID_LED_INTENSITY )->SetClean();
    GetProperties()->GetBitProperty( LeddarCore::LdPropertyIds::ID_ACQ_OPTIONS )->SetValue( 0, lResponse[2] );
    GetProperties()->GetBitProperty( LeddarCore::LdPropertyIds::ID_ACQ_OPTIONS )->SetClean();
    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_CHANGE_DELAY )->SetValue( 0, lResponse[3] );
    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_CHANGE_DELAY )->SetClean();
    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_COM_SERIAL_PORT_MAX_ECHOES )->SetValue( 0, lResponse[4] );
    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_COM_SERIAL_PORT_MAX_ECHOES )->SetClean();

    memset( lResponse, 0, LTMODBUS_RTU_MAX_ADU_LENGTH );
    mInterface->ReadRegisters( 11, 1, lResponse );
    LeddarUtils::LtTimeUtils::Wait( M16_WAIT_AFTER_REQUEST );
    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_PRECISION )->SetValue( 0, lResponse[0] );
    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_PRECISION )->SetClean();

    memset( lResponse, 0, LTMODBUS_RTU_MAX_ADU_LENGTH );
    mInterface->ReadRegisters( 14, 2, lResponse );
    LeddarUtils::LtTimeUtils::Wait( M16_WAIT_AFTER_REQUEST );
    GetProperties()->GetEnumProperty( LeddarCore::LdPropertyIds::ID_COM_SERIAL_PORT_ECHOES_RES )->SetValue( 0, lResponse[0] );
    GetProperties()->GetEnumProperty( LeddarCore::LdPropertyIds::ID_COM_SERIAL_PORT_ECHOES_RES )->SetClean();
    GetProperties()->GetBitProperty( LeddarCore::LdPropertyIds::ID_SEGMENT_ENABLE )->SetValue( 0, lResponse[1] );
    GetProperties()->GetBitProperty( LeddarCore::LdPropertyIds::ID_SEGMENT_ENABLE )->SetClean();

    memset( lResponse, 0, LTMODBUS_RTU_MAX_ADU_LENGTH );
    mInterface->ReadRegisters( 27, 4, lResponse );
    LeddarUtils::LtTimeUtils::Wait( M16_WAIT_AFTER_REQUEST );
    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_COM_SERIAL_PORT_STOP_BITS )->SetValue( 0, lResponse[0] );
    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_COM_SERIAL_PORT_STOP_BITS )->SetClean();
    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_COM_SERIAL_PORT_PARITY )->SetValue( 0, lResponse[1] );
    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_COM_SERIAL_PORT_PARITY )->SetClean();
    GetProperties()->GetEnumProperty( LeddarCore::LdPropertyIds::ID_COM_SERIAL_PORT_BAUDRATE )->SetValueIndex( 0, lResponse[2] );
    GetProperties()->GetEnumProperty( LeddarCore::LdPropertyIds::ID_COM_SERIAL_PORT_BAUDRATE )->SetClean();
    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_COM_SERIAL_PORT_ADDRESS )->SetValue( 0, lResponse[3] );
    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_COM_SERIAL_PORT_ADDRESS )->SetClean();
}

/// *****************************************************************************
/// Function: LdSensorM16Modbus::SetConfig
///
/// \brief   Set config properties on the sensor
///
/// \author  David Levy
///
/// \since   September 2017
/// *****************************************************************************
void
LdSensorM16Modbus::SetConfig( void )
{
    std::vector<LeddarCore::LdProperty *> lProperties = mProperties->FindPropertiesByCategories( LeddarCore::LdProperty::CAT_CONFIGURATION );

    for( std::vector<LeddarCore::LdProperty *>::iterator lPropertyIter = lProperties.begin(); lPropertyIter != lProperties.end(); ++lPropertyIter )
    {
        if( ( *lPropertyIter )->Modified() )
        {
            int lValue;

            switch( ( *lPropertyIter )->GetType() )
            {
                case LeddarCore::LdProperty::TYPE_BITFIELD:
                    lValue = dynamic_cast<LeddarCore::LdBitFieldProperty *>( ( *lPropertyIter ) )->Value();
                    break;

                case LeddarCore::LdProperty::TYPE_BOOL:
                    lValue = dynamic_cast<LeddarCore::LdBoolProperty *>( ( *lPropertyIter ) )->Value();
                    break;

                case LeddarCore::LdProperty::TYPE_ENUM:
                    if( ( *lPropertyIter )->GetId() == LeddarCore::LdPropertyIds::ID_LED_INTENSITY ||
                            ( *lPropertyIter )->GetId() == LeddarCore::LdPropertyIds::ID_COM_SERIAL_PORT_ECHOES_RES )
                        lValue = dynamic_cast< LeddarCore::LdEnumProperty * >( ( *lPropertyIter ) )->Value();
                    else if( ( *lPropertyIter )->GetId() == LeddarCore::LdPropertyIds::ID_COM_SERIAL_PORT_BAUDRATE )
                        lValue = dynamic_cast< LeddarCore::LdEnumProperty * >( ( *lPropertyIter ) )->ValueIndex();
                    else
                        assert( false ); //Check for other enum if we return the value or the index

                    break;

                case LeddarCore::LdProperty::TYPE_FLOAT:
                    if( dynamic_cast< LeddarCore::LdFloatProperty * >( *lPropertyIter )->GetScale() != 0 )
                    {
                        lValue = ( *lPropertyIter )->RawValue();
                    }
                    else
                    {
                        throw std::logic_error( "Float properties must have a scale for modbus communication." );
                    }

                    break;

                case LeddarCore::LdProperty::TYPE_INTEGER:
                    lValue = dynamic_cast<LeddarCore::LdIntegerProperty *>( ( *lPropertyIter ) )->ValueT<int32_t>();
                    break;

                case LeddarCore::LdProperty::TYPE_TEXT:
                    lValue = 0 ;
                    assert( false ); //No text property available at the moment
                    break;

                default:
                    lValue = 0 ;
                    assert( false ); //No text property available at the moment
                    break;
            }

            mInterface->WriteRegister( ( *lPropertyIter )->GetDeviceId(), lValue );
            ( *lPropertyIter )->SetClean();
            LeddarUtils::LtTimeUtils::Wait( M16_WAIT_AFTER_REQUEST );
        }
    }
}

/// *****************************************************************************
/// Function: LdSensorM16Modbus::GetConstants
///
/// \brief   Get constants properties on the sensor
///
/// \author  Patrick Boulay
///
/// \since   April 2017
/// *****************************************************************************
void
LdSensorM16Modbus::GetConstants( void )
{
    LeddarCore::LdIntegerProperty *lDistScale = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_DISTANCE_SCALE );
    LeddarCore::LdIntegerProperty *lAmpScale = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_AMP_SCALE );

    lDistScale->SetValue( 0, M16_DISTANCE_SCALE );
    lAmpScale->SetValue( 0, M16_AMPLITUDE_SCALE );

    mEchoes.Init( lDistScale->ValueT<uint32_t>(), lAmpScale->ValueT<uint32_t>(), M16_MAX_SERIAL_DETECTIONS );
    mStates.Init( M16_TEMPERATURE_SCALE, 1 );

    //0x11 get the device info
    uint8_t lRawRequest[2] = { mConnectionInfoModbus->GetModbusAddr(), 0x11 };
    uint8_t lResponse[LTMODBUS_RTU_MAX_ADU_LENGTH] = { 0 };

    mInterface->SendRawRequest( lRawRequest, 2 );
    size_t lReceivedSize = mInterface->ReceiveRawConfirmation( lResponse, 0 );
    LeddarUtils::LtTimeUtils::Wait( M16_WAIT_AFTER_REQUEST );

    if( lReceivedSize <= MODBUS_DATA_OFFSET )
    {
        mInterface->Flush();
        throw LeddarException::LtComException( "No data received." );
    }
    else if( lReceivedSize < static_cast<uint8_t>( lResponse[MODBUS_DATA_OFFSET] ) )
    {
        mInterface->Flush();
        throw LeddarException::LtComException( "Received size too small, received: " + LeddarUtils::LtStringUtils::IntToString( lReceivedSize ) + " expected: " +
                                               LeddarUtils::LtStringUtils::IntToString( static_cast<uint8_t>( lResponse[MODBUS_DATA_OFFSET] ) ) );
    }

    LtComLeddarM16Modbus::sLeddarM16ServerId *lDeviceInfo = reinterpret_cast<LtComLeddarM16Modbus::sLeddarM16ServerId *>( &lResponse[MODBUS_DATA_OFFSET] );

    if( lDeviceInfo->mRunStatus != 0xFF )
    {
        throw LeddarException::LtInfoException( "Wrong run status. Received " + LeddarUtils::LtStringUtils::IntToString( lDeviceInfo->mRunStatus, 16 ) + " expected: 0xFF" );
    }

    GetProperties()->GetTextProperty( LeddarCore::LdPropertyIds::ID_SERIAL_NUMBER )->SetValue( 0, lDeviceInfo->mSerialNumber );
    GetProperties()->GetTextProperty( LeddarCore::LdPropertyIds::ID_DEVICE_NAME )->SetValue( 0, lDeviceInfo->mDeviceName );
    GetProperties()->GetTextProperty( LeddarCore::LdPropertyIds::ID_SOFTWARE_PART_NUMBER )->SetValue( 0, lDeviceInfo->mSoftwarePartNumber );
    GetProperties()->GetTextProperty( LeddarCore::LdPropertyIds::ID_PART_NUMBER )->SetValue( 0, lDeviceInfo->mHardwarePartNumber );

    LeddarCore::LdIntegerProperty *lFirmwareVersion = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_FIRMWARE_VERSION_STR );
    lFirmwareVersion->SetCount( 4 );
    lFirmwareVersion->SetValue( 0, lDeviceInfo->mFirmwareVersion[0] );
    lFirmwareVersion->SetValue( 1, lDeviceInfo->mFirmwareVersion[1] );
    lFirmwareVersion->SetValue( 2, lDeviceInfo->mFirmwareVersion[2] );
    lFirmwareVersion->SetValue( 3, lDeviceInfo->mFirmwareVersion[3] );

    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_FPGA_VERSION )->SetValue( 0, lDeviceInfo->mFPGAVersion );
    GetProperties()->GetBitProperty( LeddarCore::LdPropertyIds::ID_OPTIONS )->SetValue( 0, lDeviceInfo->mDeviceOptions );
    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_DEVICE_TYPE )->SetValue( 0, lDeviceInfo->mDeviceId );
}
