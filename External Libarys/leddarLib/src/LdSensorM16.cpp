////////////////////////////////////////////////////////////////////////////////////////////////////
/// Module..: Leddar
///
/// \file   LdSensorM16.cpp.
///
/// \brief  Implements the M16 sensor class
///
/// \since  August 2018
///
/// Copyright (c) 2018 LeddarTech Inc. All rights reserved.
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "LdSensorM16.h"

#include "LdPropertyIds.h"
#include "LdBitFieldProperty.h"
#include "LdBoolProperty.h"
#include "LdEnumProperty.h"
#include "LdFloatProperty.h"
#include "LdIntegerProperty.h"
#include "LdTextProperty.h"

#include "LtExceptions.h"
#include "LtStringUtils.h"
#include "LtTimeUtils.h"

#include "comm/LtComLeddarTechPublic.h"
#include "comm/Legacy/M16/LtComM16.h"
#include "comm/LtComUSBPublic.h"

#include <cstring>

using namespace LeddarCore;

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn LeddarDevice::LdSensorM16::LdSensorM16( LeddarConnection::LdConnection *aConnection )
///
/// \brief  Constructor
///
/// \param [in,out] aConnection If non-null, the connection.
///
/// \author David Levy
/// \date   August 2018
////////////////////////////////////////////////////////////////////////////////////////////////////
LeddarDevice::LdSensorM16::LdSensorM16( LeddarConnection::LdConnection *aConnection ):
    LdSensor( aConnection ),
    mHasBeamRange( false ),
    mHasThresholdMin( false ),
    mHasIntensityMapping( false )
{
    mProtocolConfig = dynamic_cast<LeddarConnection::LdProtocolLeddartechUSB *>( aConnection );
    mProtocolData = new LeddarConnection::LdProtocolLeddartechUSB( mProtocolConfig->GetConnectionInfo(), mProtocolConfig, LeddarConnection::LdProtocolLeddartechUSB::EP_DATA );
    mResultStatePropeties = GetResultStates()->GetProperties();
    InitProperties();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn LeddarDevice::LdSensorM16::~LdSensorM16( void )
///
/// \brief  Destructor
///
/// \author David Levy
/// \date   August 2018
////////////////////////////////////////////////////////////////////////////////////////////////////
LeddarDevice::LdSensorM16::~LdSensorM16( void )
{
    if( mProtocolData != nullptr )
    {
        delete mProtocolData;
        mProtocolData = nullptr;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarDevice::LdSensorM16::InitProperties( void )
///
/// \brief  Initializes the properties for this sensor
///
/// \author Patrick Boulay
/// \date   March 2016
////////////////////////////////////////////////////////////////////////////////////////////////////
void
LeddarDevice::LdSensorM16::InitProperties( void )
{
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_FPGA_VERSION, 0, 2, "FPGA version" ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_FIRMWARE_VERSION_STR, 0, 4, "Firmware version" ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_CONSTANT, LdProperty::F_NONE, LdPropertyIds::ID_SERIAL_NUMBER, LtComLeddarTechPublic::LT_COMM_ID_SERIAL_NUMBER,
                              LT_COMM_SERIAL_NUMBER_LENGTH, LdTextProperty::TYPE_ASCII, "Serial Number" ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_SOFTWARE_PART_NUMBER, 0, LT_COMM_PART_NUMBER_LENGTH,
                              LdTextProperty::TYPE_ASCII, "Software part number" ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_CONSTANT, LdProperty::F_NONE, LdPropertyIds::ID_PART_NUMBER, LtComLeddarTechPublic::LT_COMM_ID_HW_PART_NUMBER,
                              LT_COMM_PART_NUMBER_LENGTH, LdTextProperty::TYPE_ASCII, "Hardware part number" ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_CRC32, 0, 4, "Firmware checksum" ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_DEVICE_NAME,
                              LtComLeddarTechPublic::LT_COMM_ID_DEVICE_NAME, LT_COMM_DEVICE_NAME_LENGTH, LdTextProperty::TYPE_UTF16, "Device name" ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_DEVICE_TYPE, 0, 2, "Device type" ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_HSEGMENT,
                              LtComLeddarTechPublic::LT_COMM_ID_NUMBER_OF_SEGMENTS, 2, "Number of horizontal segments" ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_VSEGMENT, 0, 2, "Number of vertical segments" ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_RSEGMENT, 0, 2, "Number of reference segment" ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_ACCUMULATION_EXP,
                              LtComLeddarTechPublic::LT_COMM_ID_CFG_ACCUMULATION_EXPONENT, 4, "Accumulation exponent" ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_ACCUMULATION_LIMITS,
                              LtComLeddarTechPublic::LT_COMM_ID_LIMIT_CFG_ACCUMULATION_EXPONENT, 4, "Accumulation exponent limits" ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_OVERSAMPLING_EXP,
                              LtComLeddarTechPublic::LT_COMM_ID_CFG_OVERSAMPLING_EXPONENT, 4, "Oversampling exponent" ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_OVERSAMPLING_LIMITS,
                              LtComLeddarTechPublic::LT_COMM_ID_LIMIT_CFG_OVERSAMPLING_EXPONENT, 4, "Oversampling exponent limits" ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_BASE_POINT_COUNT,
                              LtComLeddarTechPublic::LT_COMM_ID_CFG_BASE_SAMPLE_COUNT, 4 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_BASE_POINT_COUNT_LIMITS,
                              LtComLeddarTechPublic::LT_COMM_ID_LIMIT_CFG_BASE_SAMPLE_COUNT, 4 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_PRECISION,
                              LtComM16::M16_ID_CFG_BAYES_PRECISION, 4 ) );
    mProperties->AddProperty( new LdEnumProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_LED_INTENSITY,
                              LtComM16::M16_ID_CFG_LED_INTENSITY, 1, true, "Led power %" ) );
    mProperties->AddProperty( new LdBitFieldProperty( LdProperty::CAT_CONSTANT, LdProperty::F_NONE, LdPropertyIds::ID_OPTIONS,
                              LtComLeddarTechPublic::LT_COMM_ID_DEVICE_OPTIONS, 4 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_OTHER, LdProperty::F_EDITABLE, LdPropertyIds::ID_CHANGE_DELAY,
                              LtComM16::M16_ID_CFG_AUTO_ACQ_AVG_FRM, 2 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_CHANGE_DELAY_LIMITS,
                              LtComM16::M16_ID_LIMIT_CFG_AUTO_ACQ_AVG_FRM, 2 ) );
    mProperties->AddProperty( new LdBoolProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_GAIN_ENABLE,
                              LtComM16::M16_ID_CFG_TRANS_IMP_GAIN ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_DISTANCE_SCALE,
                              LtComLeddarTechPublic::LT_COMM_ID_DISTANCE_SCALE, 4, "Distance scaling between received value and distance in meter" ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_FILTERED_AMP_SCALE,
                              LtComLeddarTechPublic::LT_COMM_ID_FILTERED_SCALE, 4, "Amplitude scaling" ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_DETECTION_LENGTH,
                              LtComM16::M16_ID_BEAM_RANGE, 4, 0, 1 ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_SENSIVITY,
                              LtComM16::M16_ID_CFG_THRESHOLD_TABLE_OFFSET, 4, 1000, 2 ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_SENSIVITY_LIMITS,
                              LtComM16::M16_ID_LIMIT_CFG_THRESHOLD_TABLE_OFFSET, 4, 0, -1 ) );

    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_ORIGIN_X,
                              LtComM16::M16_ID_CFG_SENSOR_POSITION_X, 4, 2, 1, "Position of the sensor (X)" ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_ORIGIN_Y,
                              LtComM16::M16_ID_CFG_SENSOR_POSITION_Y, 4, 2, 1, "Position of the sensor (Y)" ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_ORIGIN_Z,
                              LtComM16::M16_ID_CFG_SENSOR_POSITION_Z, 4, 2, 1, "Position of the sensor (Z)" ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_YAW,
                              LtComM16::M16_ID_CFG_SENSOR_ORIENTATION_YAW, 4, 0, 1, "Position of the sensor (Yaw)" ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_PITCH,
                              LtComM16::M16_ID_CFG_SENSOR_ORIENTATION_PITCH, 4, 0, 1, "Position of the sensor (Pitch)" ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_ROLL,
                              LtComM16::M16_ID_CFG_SENSOR_ORIENTATION_ROLL, 4, 0, 1, "Position of the sensor (Roll)" ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_COM_SERIAL_PORT_BAUDRATE,
                              LtComM16::M16_ID_CFG_SERIAL_PORT_BAUDRATE, 4 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_COM_SERIAL_PORT_DATA_BITS,
                              LtComM16::M16_ID_CFG_SERIAL_PORT_DATA_BITS, 1 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_COM_SERIAL_PORT_PARITY,
                              LtComM16::M16_ID_CFG_SERIAL_PORT_PARITY, 1 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_COM_SERIAL_PORT_STOP_BITS,
                              LtComM16::M16_ID_CFG_SERIAL_PORT_STOP_BITS, 1 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_COM_SERIAL_PORT_ADDRESS,
                              LtComM16::M16_ID_CFG_SERIAL_PORT_ADDRESS, 1 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_COM_SERIAL_PORT_FLOW_CONTROL,
                              LtComM16::M16_ID_CFG_SERIAL_PORT_FLOW_CONTROL, 1 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_COM_SERIAL_PORT_BAUDRATE_OPTIONS,
                              LtComM16::M16_ID_SERIAL_PORT_BAUDRATE_OPTIONS_MASK, 2 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_COM_CAN_PORT_BAUDRATE,
                              LtComM16::M16_ID_CFG_CAN_PORT_BAUDRATE, 4 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_COM_CAN_PORT_TX_MSG_BASE_ID,
                              LtComM16::M16_ID_CFG_CAN_PORT_TX_MSG_BASE_ID, 4 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_COM_CAN_PORT_RX_MSG_BASE_ID,
                              LtComM16::M16_ID_CFG_CAN_PORT_RX_MSG_BASE_ID, 4 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_COM_CAN_PORT_FRAME_FORMAT,
                              LtComM16::M16_ID_CFG_CAN_PORT_FRAME_FORMAT, 1 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_COM_CAN_PORT_PORT_OPTIONS,
                              LtComM16::M16_ID_CFG_CAN_PORT_OPTIONS, 2 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE,
                              LdPropertyIds::ID_COM_CAN_PORT_PORT_MAILBOX_DELAY, LtComM16::M16_ID_CFG_CAN_PORT_MAILBOX_DELAY, 2 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE,
                              LdPropertyIds::ID_COM_CAN_PORT_PORT_ACQCYCLE_DELAY, LtComM16::M16_ID_CFG_CAN_PORT_ACQCYCLE_DELAY, 2 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE,
                              LdPropertyIds::ID_COM_CAN_PORT_PORT_MAX_ECHOES, LtComM16::M16_ID_CFG_CAN_PORT_MAX_ECHOES, 1 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE,
                              LdPropertyIds::ID_COM_CAN_PORT_PORT_OPTIONS_MASK, LtComM16::M16_ID_CAN_PORT_OPTIONS_MASK, 2 ) );

    mProperties->AddProperty( new LdBitFieldProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_ACQ_OPTIONS,
                              LtComM16::M16_ID_CFG_ACQ_OPTIONS, 2 ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_REAL_DISTANCE_OFFSET,
                              LtComLeddarTechPublic::LT_COMM_ID_REAL_DIST_OFFSET, 4, 65536, 2 ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_TRACE_POINT_STEP,
                              LtComLeddarTechPublic::LT_COMM_ID_TRACE_POINT_STEP, 4, 0, 3 ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_BASE_SAMPLE_DISTANCE,
                              LtComLeddarTechPublic::LT_COMM_ID_BASE_SAMPLE_DISTANCE, 4, 0, 3 ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_REFRESH_RATE,
                              LtComLeddarTechPublic::LT_COMM_ID_REFRESH_RATE, 4, 0, 2 ) );

    mProperties->AddProperty( new LdBufferProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_LICENSE,
                              LtComLeddarTechPublic::LT_COMM_ID_LICENSE, LT_COMM_LICENSE_KEY_LENGTH ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_OTHER, LdProperty::F_NONE, LdPropertyIds::ID_LICENSE_INFO,
                              LtComLeddarTechPublic::LT_COMM_ID_LICENSE_INFO, 4, "License type / subtype" ) );

    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_CALIBRATION, LdProperty::F_NONE, LdPropertyIds::ID_TIMEBASE_DELAY,
                              LtComM16::M16_ID_CAL_CHAN_TIMEBASE_DELAY, 4, 65536, 2 ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_CALIBRATION, LdProperty::F_NONE, LdPropertyIds::ID_INTENSITY_COMPENSATIONS,
                              LtComM16::M16_ID_CAL_LED_INTENSITY, 4, 65536, 2 ) );
    mProperties->AddProperty( new LdEnumProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_DISTANCE_RESOLUTION, LtComM16::M16_ID_CFG_LWECHOES_DIST_RES, 2 ) );

    LdEnumProperty *lDistRes = GetProperties()->GetEnumProperty( LeddarCore::LdPropertyIds::ID_DISTANCE_RESOLUTION );
    lDistRes->AddEnumPair( 1000, "millimeter" );
    lDistRes->AddEnumPair( 100, "centimeter" );
    lDistRes->AddEnumPair( 10, "decimeter" );
    lDistRes->AddEnumPair( 1, "meter" );

    // Extra result state properties
    mResultStatePropeties->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_RS_TIMESTAMP )->SetDeviceId( LtComLeddarTechPublic::LT_COMM_ID_TIMESTAMP );
    mResultStatePropeties->AddProperty( new LdFloatProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_RS_SYSTEM_TEMP,
                                        LtComLeddarTechPublic::LT_COMM_ID_SYS_TEMP, 4, 0, 1, "System Temperature" ) );
    mResultStatePropeties->AddProperty( new LdFloatProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_RS_PREDICT_TEMP,
                                        LtComM16::M16_ID_PREDICTED_TEMP, 4, 0, 1, "Predicted Temperature" ) );
    mResultStatePropeties->AddProperty( new LdIntegerProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_RS_DISCRETE_OUTPUTS,
                                        LtComM16::M16_ID_DISCRETE_OUTPUTS, 4, "Discrete Outputs" ) );
    mResultStatePropeties->AddProperty( new LdIntegerProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_RS_ACQ_CURRENT_PARAMS,
                                        LtComM16::M16_ID_ACQ_CURRENT_PARAMS, 4, "Acquisition Current Parameters" ) );
    mResultStatePropeties->AddProperty( new LdFloatProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_RS_CPU_LOAD,
                                        LtComLeddarTechPublic::LT_COMM_ID_CPU_LOAD_V2, 4, 0, 2 ) );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarDevice::LdSensorM16::Connect( void )
///
/// \brief  Connect to device
///
/// \author Patrick Boulay
/// \date   March 2016
////////////////////////////////////////////////////////////////////////////////////////////////////
void
LeddarDevice::LdSensorM16::Connect( void )
{
    LdDevice::Connect();
    mProtocolData->SetConnected( true );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarDevice::LdSensorM16::GetConstants( void )
///
/// \brief  Gets the constants from the device
///
/// \author Patrick Boulay
/// \date   February 2017
////////////////////////////////////////////////////////////////////////////////////////////////////
void
LeddarDevice::LdSensorM16::GetConstants( void )
{
    // Get info from the USB identify package
    const LeddarConnection::LdConnectionInfoUsb *lUsbConnection = dynamic_cast<const LeddarConnection::LdConnectionInfoUsb *>
            ( GetConnection()->GetConnectionInfo() );
    GetProperties()->GetTextProperty( LdPropertyIds::ID_SERIAL_NUMBER )->SetValue( 0, lUsbConnection->GetInfos().mSerialNumber );
    GetProperties()->GetTextProperty( LdPropertyIds::ID_PART_NUMBER )->SetValue( 0, lUsbConnection->GetInfos().mHardwarePartNumber );
    GetProperties()->GetTextProperty( LdPropertyIds::ID_SOFTWARE_PART_NUMBER )->SetValue( 0, lUsbConnection->GetInfos().mSoftwarePartNumber );
    GetProperties()->GetIntegerProperty( LdPropertyIds::ID_FIRMWARE_VERSION_STR )->SetValue( 0, lUsbConnection->GetInfos().mSoftwareVersion );
    GetProperties()->GetIntegerProperty( LdPropertyIds::ID_FPGA_VERSION )->SetValue( 0, lUsbConnection->GetInfos().mFpgaFirmwareVersion );
    GetProperties()->GetIntegerProperty( LdPropertyIds::ID_CRC32 )->SetValue( 0, lUsbConnection->GetInfos().mSoftwareCRC32 );
    GetProperties()->GetIntegerProperty( LdPropertyIds::ID_DEVICE_TYPE )->SetValue( 0, lUsbConnection->GetInfos().mDeviceType );

    GetListing();

    mProtocolConfig->StartRequest( LtComLeddarTechPublic::LT_COMM_CFGSRV_REQUEST_GET_DEVICE );
    mProtocolConfig->SendRequest();
    mProtocolConfig->ReadAnswer();
    mProtocolConfig->ReadElementToProperties( GetProperties() );

    uint16_t lIds[] =
    {
        LtComLeddarTechPublic::LT_COMM_ID_NUMBER_OF_SEGMENTS,
        LtComLeddarTechPublic::LT_COMM_ID_DISTANCE_SCALE,
        LtComLeddarTechPublic::LT_COMM_ID_FILTERED_SCALE,
        LtComLeddarTechPublic::LT_COMM_ID_AMPLITUDE_SCALE,
        LtComLeddarTechPublic::LT_COMM_ID_REAL_DIST_OFFSET,
        LtComLeddarTechPublic::LT_COMM_ID_TRACE_POINT_STEP,
        LtComLeddarTechPublic::LT_COMM_ID_BASE_SAMPLE_DISTANCE,
        LtComLeddarTechPublic::LT_COMM_ID_LIMIT_CFG_BASE_SAMPLE_COUNT,
        LtComLeddarTechPublic::LT_COMM_ID_LIMIT_CFG_ACCUMULATION_EXPONENT,
        LtComLeddarTechPublic::LT_COMM_ID_LIMIT_CFG_OVERSAMPLING_EXPONENT,
        LtComLeddarTechPublic::LT_COMM_ID_REFRESH_RATE,
        LtComM16::M16_ID_BEAM_RANGE,
        LtComM16::M16_ID_LIMIT_CFG_THRESHOLD_TABLE_OFFSET,
        LtComM16::M16_ID_LIMIT_CFG_AUTO_ACQ_AVG_FRM,
        LtComM16::M16_ID_ACQUISITION_OPTION_MASK,
        LtComM16::M16_ID_LIMIT_CFG_CAN_PORT_MAX_ECHOES,
        LtComM16::M16_ID_CAN_PORT_OPTIONS_MASK,
        LtComM16::M16_ID_SERIAL_PORT_BAUDRATE_OPTIONS_MASK,
        LtComM16::M16_ID_TEST_MODE
    };

    mProtocolConfig->StartRequest( LtComLeddarTechPublic::LT_COMM_CFGSRV_REQUEST_GET );
    mProtocolConfig->AddElement( LtComLeddarTechPublic::LT_COMM_ID_ELEMENT_LIST, LT_ALEN( lIds ), sizeof( lIds[ 0 ] ), lIds, sizeof( lIds[ 0 ] ) );
    mProtocolConfig->SendRequest();

    mProtocolConfig->ReadAnswer();
    mProtocolConfig->ReadElementToProperties( GetProperties() );

    uint32_t lDistanceScale = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_DISTANCE_SCALE )->ValueT<uint32_t>();
    uint32_t lFilteredScale = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_FILTERED_AMP_SCALE )->ValueT<uint32_t>();

    //CPU temp use the same scale as distance
    GetResultStates()->GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_RS_SYSTEM_TEMP )->SetScale( lDistanceScale );
    GetResultStates()->GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_RS_PREDICT_TEMP )->SetScale( 65536 );

    // Detection length is in fixed-point in sensor, but direct floating-point
    // in files so we must manipulate a little bit...
    LeddarCore::LdFloatProperty *lDetectionLength = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_DETECTION_LENGTH );

    if( lDetectionLength->Count() > 0 )
    {
        lDetectionLength->SetScale( lDistanceScale );
        float lDetectionLengthValue = lDetectionLength->Value();
        lDetectionLength->SetScale( 1 );
        lDetectionLength->SetValue( 0, lDetectionLengthValue );
    }

    GetIntensityMappings();

    UpdateConstants();

    GetResultEchoes()->Init( lDistanceScale, lFilteredScale, M16_MAX_ECHOES );
    mProtocolEchoes.resize( M16_MAX_ECHOES );

    std::vector<LeddarCore::LdProperty *> lProperties = mProperties->FindPropertiesByCategories( LeddarCore::LdProperty::CAT_CONSTANT );

    for( std::vector<LeddarCore::LdProperty *>::iterator lIter = lProperties.begin(); lIter != lProperties.end(); ++lIter )
    {
        if( ( *lIter )->Modified() )
        {
            ( *lIter )->SetClean();
        }
    }

    lProperties = mProperties->FindPropertiesByCategories( LeddarCore::LdProperty::CAT_INFO );

    for( std::vector<LeddarCore::LdProperty *>::iterator lIter = lProperties.begin(); lIter != lProperties.end(); ++lIter )
    {
        if( ( *lIter )->Modified() )
        {
            ( *lIter )->SetClean();
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarDevice::LdSensorM16::GetConfig( void )
///
/// \brief  Gets the device configuration properties
///
/// \author Patrick Boulay
/// \date   February 2017
////////////////////////////////////////////////////////////////////////////////////////////////////
void
LeddarDevice::LdSensorM16::GetConfig( void )
{
    mProtocolConfig->StartRequest( LtComLeddarTechPublic::LT_COMM_CFGSRV_REQUEST_GET_CONFIG );
    mProtocolConfig->SendRequest();
    mProtocolConfig->ReadAnswer();
    mProtocolConfig->ReadElementToProperties( GetProperties() );

    std::vector<LeddarCore::LdProperty *> lProperties = mProperties->FindPropertiesByCategories( LeddarCore::LdProperty::CAT_CONFIGURATION );

    for( std::vector<LeddarCore::LdProperty *>::iterator lIter = lProperties.begin(); lIter != lProperties.end(); ++lIter )
    {
        if( ( *lIter )->Modified() )
        {
            ( *lIter )->SetClean();
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarDevice::LdSensorM16::SetConfig( void )
///
/// \brief  Sets the configuration to the device
///
/// \author Patrick Boulay
/// \date   March 2017
////////////////////////////////////////////////////////////////////////////////////////////////////
void
LeddarDevice::LdSensorM16::SetConfig( void )
{
    mProtocolConfig->StartRequest( LtComLeddarTechPublic::LT_COMM_CFGSRV_REQUEST_SET_CONFIG );

    std::vector<LeddarCore::LdProperty *> lProperties = mProperties->FindPropertiesByCategories( LeddarCore::LdProperty::CAT_CONFIGURATION );

    for( std::vector<LeddarCore::LdProperty *>::iterator lIter = lProperties.begin(); lIter != lProperties.end(); ++lIter )
    {
        if( ( *lIter )->Modified() )
        {
            mProtocolConfig->AddElement( ( *lIter )->GetDeviceId(), static_cast<uint16_t>( ( *lIter )->Count() ), ( *lIter )->UnitSize(), ( *lIter )->CStorage(),
                                         static_cast<uint32_t>( ( *lIter )->Stride() ) );
        }
    }

    mProtocolConfig->SendRequest();
    mProtocolConfig->ReadAnswer();

    for( std::vector<LeddarCore::LdProperty *>::iterator lIter = lProperties.begin(); lIter != lProperties.end(); ++lIter )
    {
        if( ( *lIter )->Modified() )
        {
            ( *lIter )->SetClean();
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarDevice::LdSensorM16::WriteConfig( void )
///
/// \brief  Writes the configuration on the device
///
/// \author Patrick Boulay
/// \date   March 2017
////////////////////////////////////////////////////////////////////////////////////////////////////
void
LeddarDevice::LdSensorM16::WriteConfig( void )
{
    mProtocolConfig->StartRequest( LtComLeddarTechPublic::LT_COMM_CFGSRV_REQUEST_WRITE_CONFIG );
    mProtocolConfig->SendRequest();
    mProtocolConfig->ReadAnswer();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarDevice::LdSensorM16::RestoreConfig( void )
///
/// \brief  Restore configuration on the device. Call GetConfig() to update local (SDK) values
///         after doing a RestoreConfig
///
/// \author Patrick Boulay
/// \date   March 2017
////////////////////////////////////////////////////////////////////////////////////////////////////
void
LeddarDevice::LdSensorM16::RestoreConfig( void )
{
    mProtocolConfig->StartRequest( LtComLeddarTechPublic::LT_COMM_CFGSRV_REQUEST_RESTORE_CONFIG );
    mProtocolConfig->SendRequest();
    mProtocolConfig->ReadAnswer();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarDevice::LdSensorM16::GetCalib()
///
/// \brief  Gets the calibration from the sensor
///
/// \author David Levy
/// \date   June 2018
////////////////////////////////////////////////////////////////////////////////////////////////////
void
LeddarDevice::LdSensorM16::GetCalib()
{
    mProtocolConfig->StartRequest( LtComLeddarTechPublic::LT_COMM_CFGSRV_REQUEST_GET_CAL );
    mProtocolConfig->SendRequest();

    mProtocolConfig->ReadAnswer();
    mProtocolConfig->ReadElementToProperties( mProperties );

    std::vector<LeddarCore::LdProperty *> lProperties = mProperties->FindPropertiesByCategories( LeddarCore::LdProperty::CAT_CALIBRATION );

    for( std::vector<LeddarCore::LdProperty *>::iterator lIter = lProperties.begin(); lIter != lProperties.end(); ++lIter )
    {
        if( ( *lIter )->Modified() )
        {
            ( *lIter )->SetClean();
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarDevice::LdSensorM16::GetListing( void )
///
/// \brief  Read the listing of all supported commands and ids and store it so that it can be
///     queried.
///
/// \author Patrick Boulay
/// \date   March 2017
////////////////////////////////////////////////////////////////////////////////////////////////////
void
LeddarDevice::LdSensorM16::GetListing( void )
{
    mProtocolConfig->StartRequest( LtComLeddarTechPublic::LT_COMM_CFGSRV_REQUEST_LISTING );
    mProtocolConfig->SendRequest();
    mProtocolConfig->ReadAnswer();

    uint16_t lElementCout = 0;
    LtComLeddarTechPublic::sLtCommElementRequestInfo *lElementReqInfo = nullptr;

    while( mProtocolConfig->ReadElement() )
    {
        if( mProtocolConfig->GetElementId() == LtComLeddarTechPublic::LT_COMM_ID_REQUEST_ELEMENT_LIST )
        {
            lElementReqInfo = static_cast<LtComLeddarTechPublic::sLtCommElementRequestInfo *>( mProtocolConfig->GetElementData() );
            lElementCout = mProtocolConfig->GetElementCount();
            break;
        }
    }

    bool lValidDataLevel = false;

    for( int i = 0; i < lElementCout; ++i )
    {
        if( lElementReqInfo[ i ].mRequestCode == LtComM16::M16_CFGSRV_REQUEST_BASE_SAMPLE_COUNT_TO_BEAM_RANGE )
        {
            mHasBeamRange = true;
        }

        if( lElementReqInfo[ i ].mRequestCode == LtComM16::M16_CFGSRV_REQUEST_PARAMS_TO_THRES_TABLE_OFFSET_MIN )
        {
            mHasThresholdMin = true;
        }

        if( lElementReqInfo[ i ].mRequestCode == LtComM16::M16_CFGSRV_REQUEST_PARAMS_TO_LED_POWER )
        {
            mHasIntensityMapping = true;
        }

        if( lElementReqInfo[ i ].mElementId == LtComM16::M16_ID_DATA_LEVEL )
        {
            lValidDataLevel = true;
        }
    }

    if( !lValidDataLevel )
    {
        throw std::runtime_error( "Your firmware is incompatible with the SDK, please update your firmware." );
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarDevice::LdSensorM16::GetIntensityMappings( void )
///
/// \brief  Get the intensity percentage for each value of LED intensity.
///
/// \author Patrick Boulay
/// \date   March 2017
////////////////////////////////////////////////////////////////////////////////////////////////////
void
LeddarDevice::LdSensorM16::GetIntensityMappings( void )
{
    if( mHasIntensityMapping )
    {
        LdIntegerProperty lIntensityMapping( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_LED_INTENSITY_LIST, 0, 1 );
        lIntensityMapping.SetCount( M16_LED_INTENSITY_MAX + 1 );
        uint8_t lCount = 0;
        uint8_t lOldValue = -1;

        for( uint8_t i = 0; i <= M16_LED_INTENSITY_MAX; ++i )
        {
            mProtocolConfig->StartRequest( LtComM16::M16_CFGSRV_REQUEST_PARAMS_TO_LED_POWER );
            mProtocolConfig->AddElement( LtComM16::M16_ID_CFG_LED_INTENSITY, 1, 1, &i, 1 );
            mProtocolConfig->SendRequest();

            mProtocolConfig->ReadAnswer();

            while( mProtocolConfig->ReadElement() )
            {
                if( mProtocolConfig->GetElementId() == LtComM16::M16_ID_LED_POWER )
                {
                    uint8_t lValue = *( static_cast<uint8_t *>( mProtocolConfig->GetElementData() ) );
                    lIntensityMapping.SetValue( i, lValue );

                    if( lOldValue != lValue )
                    {
                        ++lCount;
                        lOldValue = lValue;
                    }
                }
            }
        }

        //Populate intensity list with it
        LdEnumProperty *lIntensity = GetProperties()->GetEnumProperty( LeddarCore::LdPropertyIds::ID_LED_INTENSITY );
        lIntensity->SetEnumSize( lCount );
        uint16_t lMax = 110;

        for( int i = M16_LED_INTENSITY_MAX; i >= 0; --i )
        {
            if( lIntensityMapping.Value( i ) != lMax )
            {
                lMax = lIntensityMapping.ValueT<uint16_t>( i );
                lIntensity->AddEnumPair( i, LeddarUtils::LtStringUtils::IntToString( lMax ) );
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarDevice::LdSensorM16::UpdateConstants( void )
///
/// \brief  Updates the constants
///
/// \author Patrick Boulay
/// \date   March 2017
////////////////////////////////////////////////////////////////////////////////////////////////////
void
LeddarDevice::LdSensorM16::UpdateConstants( void )
{
    LdFloatProperty *lThresholdOffset = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_SENSIVITY );
    lThresholdOffset->SetScale( GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_FILTERED_AMP_SCALE )->ValueT<uint32_t>() );
    GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_SENSIVITY_LIMITS )->SetScale( lThresholdOffset->Scale() );

    // For older devices that don't report it.
    LdIntegerProperty *lHChannelCount = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_HSEGMENT );

    if( lHChannelCount->Count() == 0 )
    {
        lHChannelCount->SetCount( 1 );
        lHChannelCount->SetValue( 0, M16_NUMBER_CHANNELS );
    }

    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_VSEGMENT )->SetValue( 0, 1 );
    GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_RSEGMENT )->SetValue( 0, 0 );

    // Adjust some properties limits.
    // Base sample count
    int64_t lMin = 2;
    int64_t lMax = 64;

    LdIntegerProperty *lPointCountLimits = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_BASE_POINT_COUNT_LIMITS );

    if( lPointCountLimits->Count() == 2 )
    {
        lMin = lPointCountLimits->Value( 0 );
        lMax = lPointCountLimits->Value( 1 );
    }

    LdIntegerProperty *lBaseSampleCount = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_BASE_POINT_COUNT );
    lBaseSampleCount->SetLimits( lMin, lMax );

    // Accumulation
    lMin = 0;
    lMax = 8;

    LdIntegerProperty *lAccumulationLimits = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_ACCUMULATION_LIMITS );

    if( lAccumulationLimits->Count() == 2 )
    {
        lMin = lAccumulationLimits->Value( 0 );
        lMax = lAccumulationLimits->Value( 1 );
    }

    LdIntegerProperty *lAccumulationExp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_ACCUMULATION_EXP );
    lAccumulationExp->SetLimits( lMin, lMax );

    // Oversampling
    lMin = 0;
    lMax = 3;

    LdIntegerProperty *lOversamplingLimits = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_OVERSAMPLING_LIMITS );

    if( lOversamplingLimits->Count() == 2 )
    {
        lMin = lOversamplingLimits->Value( 0 );
        lMax = lOversamplingLimits->Value( 1 );
    }

    LdIntegerProperty *lOversamplingExp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_OVERSAMPLING_EXP );
    lOversamplingExp->SetLimits( lMin, lMax );

    // Threshold offset
    float lMinf = 0;
    float lMaxf = 100;

    LdFloatProperty *lThresoldOffsetLimits = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_SENSIVITY_LIMITS );

    if( lThresoldOffsetLimits->Count() == 2 )
    {
        lMinf = lThresoldOffsetLimits->Value( 0 );
        lMaxf = lThresoldOffsetLimits->Value( 1 );
    }

    LdFloatProperty *lThresoldOffset = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_SENSIVITY );
    lThresoldOffset->SetLimits( lMinf, lMaxf );

    // Change delay
    lMin = 0;
    lMax = 32767;

    LdIntegerProperty *lChangeDelayLimits = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_CHANGE_DELAY_LIMITS );

    if( lChangeDelayLimits->Count() == 2 )
    {
        lMin = lChangeDelayLimits->Value( 0 );
        lMax = lChangeDelayLimits->Value( 1 );
    }

    LdIntegerProperty *lChangeDelay = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_CHANGE_DELAY );
    lChangeDelay->SetLimits( lMin, lMax );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarDevice::LdSensorM16::SetDataMask( uint32_t aDataMask )
///
/// \brief  Set data mask for the data end point.
///
/// \param  aDataMask   The data mask.
///
/// \author Patrick Boulay
/// \date   March 2017
////////////////////////////////////////////////////////////////////////////////////////////////////
void
LeddarDevice::LdSensorM16::SetDataMask( uint32_t aDataMask )
{
    mDataMask = aDataMask;
    uint32_t lDataMask = this->ConvertDataMaskToLTDataMask( aDataMask );

    mProtocolConfig->StartRequest( LtComLeddarTechPublic::LT_COMM_CFGSRV_REQUEST_SET );
    mProtocolConfig->AddElement( LtComM16::M16_ID_DATA_LEVEL, 1, sizeof( lDataMask ), &lDataMask, sizeof( lDataMask ) );
    mProtocolConfig->SendRequest();
    mProtocolConfig->ReadAnswer();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn bool LeddarDevice::LdSensorM16::GetData( void )
///
/// \brief  Gets the data from the device
///
/// \return True is new data was processed, false a timeout was catch from ReadRequest.
///
/// \author Patrick Boulay
/// \date   March 2017
////////////////////////////////////////////////////////////////////////////////////////////////////
bool
LeddarDevice::LdSensorM16::GetData( void )
{
    // Verify if the data mask is set
    if( mDataMask == DM_NONE || ( ( mDataMask & DM_STATES ) == 0 ) ) //states are required for timestamp
    {
        SetDataMask( DM_ALL );
    }

    // Read available data on the data channel
    try
    {
        mProtocolData->ReadRequest();
    }
    catch( LeddarException::LtTimeoutException & )
    {
        return false;
    }

    uint16_t lRequestCode = mProtocolData->GetRequestCode();

    //Return true only on states because they are received last for a frame, and they hold the timestamp (trace and echo dont know the timestamp by themselves)
    if( lRequestCode == LtComLeddarTechPublic::LT_COMM_DATASRV_REQUEST_SEND_ECHOES )
    {
        ProcessEchoes();
        return false;
    }
    else if( lRequestCode == LtComLeddarTechPublic::LT_COMM_DATASRV_REQUEST_SEND_STATES )
    {
        return ProcessStates();
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn bool LeddarDevice::LdSensorM16::ProcessStates( void )
///
/// \brief  Process the states during a GetData and update timestamp on echoes
///
/// \return True if new states are received.
///
/// \author David Levy
/// \date   August 2018
////////////////////////////////////////////////////////////////////////////////////////////////////
bool
LeddarDevice::LdSensorM16::ProcessStates( void )
{
    uint32_t lPreviousTimeStamp = mStates.GetTimestamp();
    mProtocolData->ReadElementToProperties( mResultStatePropeties );

    if( lPreviousTimeStamp != mStates.GetTimestamp() )
    {
        //Only the states know the timestamp, so update echoes timestamp, swap buffers, and trigger the update finished
        mEchoes.SetTimestamp( mStates.GetTimestamp() );
        mEchoes.UnLock( LeddarConnection::B_SET );
        mEchoes.Swap();
        mEchoes.UpdateFinished();

        //And Finally Get specific data from sensor
        RequestProperties( GetResultStates()->GetProperties(), std::vector<uint16_t>( 1, LtComLeddarTechPublic::LT_COMM_ID_CPU_LOAD_V2 ) );
        mStates.UpdateFinished();
        return true;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarDevice::LdSensorM16::ProcessEchoes( void )
///
/// \brief  Process the echoes read by GetData
///
/// \author Patrick Boulay
/// \date   March 2017
////////////////////////////////////////////////////////////////////////////////////////////////////
void
LeddarDevice::LdSensorM16::ProcessEchoes( void )
{
    if( mProtocolData->GetMessageSize() == 0 )
    {
        mEchoes.SetEchoCount( 0 );
    }

    //Read data into protocol specific buffer
    while( mProtocolData->ReadElement() )
    {
        switch( mProtocolData->GetElementId() )
        {
            case LtComLeddarTechPublic::LT_COMM_ID_ECHOES_AMPLITUDE:
                mEchoes.SetEchoCount( mProtocolData->GetElementCount() );
                mProtocolData->PushElementDataToBuffer( &mProtocolEchoes[ 0 ].mAmplitude,
                                                        mProtocolData->GetElementCount(),
                                                        sizeof( mProtocolEchoes[ 0 ].mAmplitude ),
                                                        sizeof( mProtocolEchoes[ 0 ] ) );
                break;

            case LtComLeddarTechPublic::LT_COMM_ID_ECHOES_DISTANCE:
                mEchoes.SetEchoCount( mProtocolData->GetElementCount() );
                mProtocolData->PushElementDataToBuffer( &mProtocolEchoes[ 0 ].mDistance,
                                                        mProtocolData->GetElementCount(),
                                                        sizeof( mProtocolEchoes[ 0 ].mDistance ),
                                                        sizeof( mProtocolEchoes[ 0 ] ) );
                break;

            case LtComLeddarTechPublic::LT_COMM_ID_ECHOES_BASE:
                mEchoes.SetEchoCount( mProtocolData->GetElementCount() );
                mProtocolData->PushElementDataToBuffer( &mProtocolEchoes[ 0 ].mBase,
                                                        mProtocolData->GetElementCount(),
                                                        sizeof( mProtocolEchoes[ 0 ].mBase ),
                                                        sizeof( mProtocolEchoes[ 0 ] ) );
                break;

            case LtComLeddarTechPublic::LT_COMM_ID_ECHOES_MAX_INDEX:
                mEchoes.SetEchoCount( mProtocolData->GetElementCount() );
                mProtocolData->PushElementDataToBuffer( &mProtocolEchoes[ 0 ].mMaxIndex,
                                                        mProtocolData->GetElementCount(),
                                                        sizeof( mProtocolEchoes[ 0 ].mMaxIndex ),
                                                        sizeof( mProtocolEchoes[ 0 ] ) );
                break;

            case LtComLeddarTechPublic::LT_COMM_ID_ECHOES_CHANNEL_INDEX:
                mEchoes.SetEchoCount( mProtocolData->GetElementCount() );
                mProtocolData->PushElementDataToBuffer( &mProtocolEchoes[ 0 ].mChannelIndex,
                                                        mProtocolData->GetElementCount(),
                                                        sizeof( uint8_t ),
                                                        sizeof( mProtocolEchoes[ 0 ] ) );
                break;

            case LtComLeddarTechPublic::LT_COMM_ID_ECHOES_VALID:
                mEchoes.SetEchoCount( mProtocolData->GetElementCount() );
                mProtocolData->PushElementDataToBuffer( &mProtocolEchoes[ 0 ].mValid,
                                                        mProtocolData->GetElementCount(),
                                                        sizeof( mProtocolEchoes[ 0 ].mValid ),
                                                        sizeof( mProtocolEchoes[ 0 ] ) );
                break;

            case LtComLeddarTechPublic::LT_COMM_ID_ECHOES_AMPLITUDE_LOW_SCALE:
                mEchoes.SetEchoCount( mProtocolData->GetElementCount() );
                mProtocolData->PushElementDataToBuffer( &mProtocolEchoes[ 0 ].mAmplitudeLowScale,
                                                        mProtocolData->GetElementCount(),
                                                        sizeof( mProtocolEchoes[ 0 ].mAmplitudeLowScale ),
                                                        sizeof( mProtocolEchoes[ 0 ] ) );
                break;

            case LtComLeddarTechPublic::LT_COMM_ID_ECHOES_SATURATION_WIDTH:
                mEchoes.SetEchoCount( mProtocolData->GetElementCount() );
                mProtocolData->PushElementDataToBuffer( &mProtocolEchoes[ 0 ].mSaturationWidth,
                                                        mProtocolData->GetElementCount(),
                                                        sizeof( mProtocolEchoes[ 0 ].mSaturationWidth ),
                                                        sizeof( mProtocolEchoes[ 0 ] ) );
                break;
        }
    }

    //And copy echoes into standard echo container
    std::vector<LeddarConnection::LdEcho> *lEchoes = mEchoes.GetEchoes( LeddarConnection::B_SET );
    mEchoes.Lock( LeddarConnection::B_SET );

    for( size_t i = 0u; i < mProtocolEchoes.size(); ++i )
    {
        ( *lEchoes )[ i ].mAmplitude = mProtocolEchoes[ i ].mAmplitude;
        ( *lEchoes )[ i ].mAmplitudeLowScale = mProtocolEchoes[ i ].mAmplitudeLowScale;
        ( *lEchoes )[ i ].mBase = mProtocolEchoes[ i ].mBase;
        ( *lEchoes )[ i ].mChannelIndex = mProtocolEchoes[ i ].mChannelIndex;
        ( *lEchoes )[ i ].mDistance = mProtocolEchoes[ i ].mDistance;
        ( *lEchoes )[ i ].mFlag = mProtocolEchoes[ i ].mValid;
        ( *lEchoes )[ i ].mMaxIndex = mProtocolEchoes[ i ].mMaxIndex;
        ( *lEchoes )[ i ].mSaturationWidth = mProtocolEchoes[ i ].mSaturationWidth;

    }

    //We do not swap nor send the UpdateFinished signal here, the timestamp is only in the states so we need to wait for the states
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarDevice::LdSensorM16::Reset( LeddarDefines::eResetType aType, LeddarDefines::eResetOptions aOptions )
///
/// \brief  Resets the device
///
/// \exception  LeddarException::LtComException Thrown when a trying to do an unimplemented reset
///
/// \param  aType       Reset type. See \ref LeddarDefines::eResetType
/// \param  aOptions    Reset options. See \ref LeddarDefines::eResetOptions
///
/// \author David Levy
/// \date   August 2018
////////////////////////////////////////////////////////////////////////////////////////////////////
void
LeddarDevice::LdSensorM16::Reset( LeddarDefines::eResetType aType, LeddarDefines::eResetOptions aOptions )
{
    if( aType == LeddarDefines::RT_CONFIG_RESET )
    {
        mProtocolConfig->StartRequest( LtComLeddarTechPublic::LT_COMM_CFGSRV_REQUEST_RESET_CONFIG );
        mProtocolConfig->SendRequest();
        mProtocolConfig->ReadAnswer();
    }
    else if( aType == LeddarDefines::RT_SOFT_RESET )
    {
        mProtocolConfig->StartRequest( LtComLeddarTechPublic::LT_COMM_CFGSRV_REQUEST_RESET );
        uint8_t lSoftwareType = 0;

        if( aOptions == LeddarDefines::RO_MAIN )
        {
            lSoftwareType = LtComLeddarTechPublic::LT_COMM_SOFTWARE_TYPE_MAIN;
        }
        else if( aOptions == LeddarDefines::RO_FACTORY )
        {
            lSoftwareType = LtComLeddarTechPublic::LT_COMM_SOFTWARE_TYPE_FACTORY;
        }
        else
        {
            throw LeddarException::LtComException( "Reset option not valid: " + LeddarUtils::LtStringUtils::IntToString( aOptions ) + "." );
        }

        mProtocolConfig->AddElement( LtComLeddarTechPublic::LT_COMM_ID_SOFTWARE_TYPE, 1, sizeof( lSoftwareType ), &lSoftwareType, sizeof( lSoftwareType ) );
        mProtocolConfig->SendRequest();
        mProtocolConfig->ReadAnswer();

        LeddarUtils::LtTimeUtils::Wait( 1500 );

        Disconnect();
        LeddarUtils::LtTimeUtils::Wait( 3000 );
        Connect();
    }
    else
    {
        throw LeddarException::LtComException( "Reset type: " + LeddarUtils::LtStringUtils::IntToString( aType ) + " not implemented." );
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarDevice::LdSensorM16::RequestProperties( LeddarCore::LdPropertiesContainer *aProperties, std::vector<uint16_t> aDeviceIds )
///
/// \brief  Update requested properties from the sensor
///
/// \param [in,out] aProperties Property container to store the update properties into.
/// \param          aDeviceIds  List of device ids for the properties.
///
/// \author David Levy
/// \date   June 2017
////////////////////////////////////////////////////////////////////////////////////////////////////
void
LeddarDevice::LdSensorM16::RequestProperties( LeddarCore::LdPropertiesContainer *aProperties, std::vector<uint16_t> aDeviceIds )
{
    mProtocolConfig->StartRequest( LtComLeddarTechPublic::LT_COMM_CFGSRV_REQUEST_GET );
    mProtocolConfig->AddElement( LtComLeddarTechPublic::LT_COMM_ID_ELEMENT_LIST, static_cast<uint16_t>( aDeviceIds.size() ), sizeof( aDeviceIds[0] ), &aDeviceIds[0],
                                 sizeof( aDeviceIds[0] ) );
    mProtocolConfig->SendRequest();

    mProtocolConfig->ReadAnswer();
    mProtocolConfig->ReadElementToProperties( aProperties );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarDevice::LdSensorM16::SetProperties( LeddarCore::LdPropertiesContainer *aProperties, std::vector<uint16_t> aDeviceIds, unsigned int aRetryNbr )
///
/// \brief  Sets the properties
///
/// \exception  LeddarException::LtComException Thrown when a Lt Com error condition occurs.
///
/// \param [in,out] aProperties If non-null, the properties.
/// \param          aDeviceIds  List of identifiers for the devices.
/// \param          aRetryNbr   The retry number.
///
/// \author David Levy
/// \date   August 2018
////////////////////////////////////////////////////////////////////////////////////////////////////
void
LeddarDevice::LdSensorM16::SetProperties( LeddarCore::LdPropertiesContainer *aProperties, std::vector<uint16_t> aDeviceIds, unsigned int aRetryNbr )
{
    for( size_t i = 0; i < aDeviceIds.size(); ++i )
    {
        LeddarCore::LdProperty *lProperty = aProperties->FindDeviceProperty( aDeviceIds[ i ] );

        if( lProperty != nullptr )
        {
            mProtocolConfig->StartRequest( LtComLeddarTechPublic::LT_COMM_CFGSRV_REQUEST_SET );
            mProtocolConfig->AddElement( aDeviceIds[ i ], static_cast<uint16_t>( lProperty->GetCount() ), lProperty->UnitSize(), lProperty->CStorage(),
                                         static_cast<uint32_t>( lProperty->Stride() ) );
            mProtocolConfig->SendRequest();

            unsigned int lCount = aRetryNbr;
            bool lRetry = false;

            do
            {
                try
                {
                    lRetry = false;
                    mProtocolConfig->ReadAnswer();
                }
                catch( LeddarException::LtComException &e )
                {
                    if( e.GetDisconnect() == true )
                        throw;

                    ( lCount-- != 0 ) ? lRetry = true : throw;

                }
            }
            while( lRetry );
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarDevice::LdSensorM16::RemoveLicense( const std::string &aLicense )
///
/// \brief  Removes a specific license from sensor
///
/// \exception  LeddarException::LtComException Thrown when the device answer code indicate an error.
///
/// \param  aLicense    The license.
///
/// \author David Levy
/// \date   August 2018
////////////////////////////////////////////////////////////////////////////////////////////////////
void
LeddarDevice::LdSensorM16::RemoveLicense( const std::string &aLicense )
{
    LdBufferProperty *lLicenseProp = GetProperties()->GetBufferProperty( LdPropertyIds::ID_LICENSE );
    std::string lCurrentLicense = lLicenseProp->GetStringValue();
    std::transform( lCurrentLicense.begin(), lCurrentLicense.end(), lCurrentLicense.begin(), ::toupper );

    std::string lToRemove = aLicense;
    std::transform( lToRemove.begin(), lToRemove.end(), lToRemove.begin(), ::toupper );

    if( lToRemove == lCurrentLicense )
    {
        uint8_t lEmptyLicense[ LT_COMM_LICENSE_KEY_LENGTH ];
        memset( lEmptyLicense, 0, LT_COMM_LICENSE_KEY_LENGTH );

        try
        {
            SendLicense( lEmptyLicense, false );
        }
        catch( std::runtime_error &e )
        {
            //Invalid license sent on purpose to remove the real license
            if( strcmp( e.what(), "Invalid license." ) != 0 )
                throw;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarDevice::LdSensorM16::RemoveAllLicenses( void )
///
/// \brief  Removes all licenses from the sensor
///
/// \exception  LeddarException::LtComException Thrown when the device answer code indicate an error.
///
/// \author David Levy
/// \date   August 2018
////////////////////////////////////////////////////////////////////////////////////////////////////
void
LeddarDevice::LdSensorM16::RemoveAllLicenses( void )
{
    uint8_t lEmptyLicense[ LT_COMM_LICENSE_KEY_LENGTH ];
    memset( lEmptyLicense, 0, LT_COMM_LICENSE_KEY_LENGTH );

    try
    {
        SendLicense( lEmptyLicense, false );
    }
    catch( std::runtime_error &e )
    {
        //Invalid license sent on purpose to remove the real license
        if( strcmp( e.what(), "Invalid license." ) != 0 )
            throw;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn LeddarDevice::sLicense LeddarDevice::LdSensorM16::SendLicense( const std::string &aLicense )
///
/// \brief  Sends a license to the sensor
///
/// \exception  std::length_error               Raised when license length is invalid.
/// \exception  LeddarException::LtComException Thrown when the device answer code indicate an error.
/// \exception  std::runtime_error              If the license sent is invalid.
///
/// \param  aLicense    The license to set.
///
/// \return A LeddarDevice::sLicense containing the license string and the authorisation level.
///
/// \author David Levy
/// \date   August 2018
////////////////////////////////////////////////////////////////////////////////////////////////////
LeddarDefines::sLicense
LeddarDevice::LdSensorM16::SendLicense( const std::string &aLicense )
{
    return SendLicense( aLicense, false );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn LeddarDevice::sLicense LeddarDevice::LdSensorM16::SendLicense( const std::string &aLicense, bool aVolatile )
///
/// \brief  Sends a license to the sensor
///
/// \exception  std::length_error               Raised when license length is invalid.
/// \exception  LeddarException::LtComException Thrown when the device answer code indicate an error.
/// \exception  std::runtime_error              If the license sent is invalid.
///
/// \param  aLicense    The license to send.
/// \param  aVolatile   True if license is volatile / temporary. [LeddarTech internal use]
///
/// \return A LeddarDevice::sLicense containing the license string and the authorisation level.
///
/// \author David Levy
/// \date   August 2018
////////////////////////////////////////////////////////////////////////////////////////////////////
LeddarDefines::sLicense
LeddarDevice::LdSensorM16::SendLicense( const std::string &aLicense, bool aVolatile )
{
    if( aLicense.length() != LT_COMM_LICENSE_KEY_LENGTH * 2 )
        throw std::length_error( "Invalid license length." );

    // Convert the user string to 16 bytes license
    uint8_t lBuffer[LT_COMM_LICENSE_KEY_LENGTH];

    for( size_t i = 0; i < aLicense.length(); i += 2 )
    {
        lBuffer[i / 2] = ( uint8_t )strtoul( aLicense.substr( i, 2 ).c_str(), nullptr, 16 );
    }

    return SendLicense( lBuffer, aVolatile );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn LeddarDevice::sLicense LeddarDevice::LdSensorM16::SendLicense( const uint8_t *aLicense, bool aVolatile )
///
/// \brief  Sends a license to the sensor
///
/// \exception  LeddarException::LtComException Thrown when the device answer code indicate an error.
/// \exception  std::runtime_error              If the license sent is invalid.
///
/// \param  aLicense    The license to send.
/// \param  aVolatile   True if license is volatile / temporary. [LeddarTech internal use]
///
/// \return A LeddarDevice::sLicense containing the license string and the authorisation level.
///
/// \author David Levy
/// \date   August 2018
////////////////////////////////////////////////////////////////////////////////////////////////////
LeddarDefines::sLicense
LeddarDevice::LdSensorM16::SendLicense( const uint8_t *aLicense, bool aVolatile )
{
    //Store the license in the property, send it to the sensor, and read back from the sensor license info
    LdBufferProperty *lLicenseProp = GetProperties()->GetBufferProperty( aVolatile ? LdPropertyIds::ID_VOLATILE_LICENSE : LdPropertyIds::ID_LICENSE );

    if( lLicenseProp->Count() == 0 )
        lLicenseProp->SetCount( 1 );

    lLicenseProp->SetValue( 0, aLicense, LT_COMM_LICENSE_KEY_LENGTH );
    lLicenseProp->SetClean();

    std::vector<uint16_t> lDeviceIds;
    lDeviceIds.push_back( aVolatile ? LtComLeddarTechPublic::LT_COMM_ID_VOLATILE_LICENSE : LtComLeddarTechPublic::LT_COMM_ID_LICENSE );
    SetProperties( GetProperties(), lDeviceIds );

    LeddarDefines::sLicense lResultLicense;

    if( mProtocolConfig->GetAnswerCode() != LtComLeddarTechPublic::LT_COMM_ANSWER_OK )
    {
        throw LeddarException::LtComException( "Wrong answer code : " + LeddarUtils::LtStringUtils::IntToString( mProtocolConfig->GetAnswerCode() ) );
    }


    lDeviceIds.push_back( aVolatile ? LtComLeddarTechPublic::LT_COMM_ID_VOLATILE_LICENSE_INFO : LtComLeddarTechPublic::LT_COMM_ID_LICENSE_INFO );
    RequestProperties( GetProperties(), lDeviceIds );

    if( mProtocolConfig->GetAnswerCode() != LtComLeddarTechPublic::LT_COMM_ANSWER_OK )
    {
        throw LeddarException::LtComException( "Wrong answer code : " + LeddarUtils::LtStringUtils::IntToString( mProtocolConfig->GetAnswerCode() ) );
    }

    LdIntegerProperty *lLicenseInfo = GetProperties()->GetIntegerProperty( aVolatile ?  LdPropertyIds::ID_VOLATILE_LICENSE_INFO : LdPropertyIds::ID_LICENSE_INFO );
    lResultLicense.mLicense = lLicenseProp->GetStringValue( 0 );
    lResultLicense.mType = lLicenseInfo->Value() & 0xFFFF;
    lResultLicense.mSubType = static_cast<uint8_t>( lLicenseInfo->ValueT<uint32_t>() >> 16 );

    if( lResultLicense.mType == 0 )
    {
        throw std::runtime_error( "Invalid license." );
    }

    return lResultLicense;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn std::vector<LeddarDevice::sLicense> LeddarDevice::LdSensorM16::GetLicenses( void )
///
/// \brief  Gets the licenses on the sensor
///
/// \return The licenses on the sensor.
///
/// \author David Levy
/// \date   August 2018
////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<LeddarDefines::sLicense>
LeddarDevice::LdSensorM16::GetLicenses( void )
{
    std::vector<LeddarDefines::sLicense> lLicenses;
    std::vector<uint16_t> lDeviceIds;
    lDeviceIds.push_back( LtComLeddarTechPublic::LT_COMM_ID_LICENSE );
    lDeviceIds.push_back( LtComLeddarTechPublic::LT_COMM_ID_LICENSE_INFO );
    RequestProperties( GetProperties(), lDeviceIds );

    if( mProtocolConfig->GetAnswerCode() != LtComLeddarTechPublic::LT_COMM_ANSWER_OK )
    {
        return lLicenses;
    }

    LdIntegerProperty *lLicenseInfo = GetProperties()->GetIntegerProperty( LdPropertyIds::ID_LICENSE_INFO );
    LdBufferProperty *lLicenseProp = GetProperties()->GetBufferProperty( LdPropertyIds::ID_LICENSE );

    for( size_t i = 0; i < lLicenseProp->Count(); ++i )
    {
        LeddarDefines::sLicense lResultLicense;
        lResultLicense.mLicense = lLicenseProp->GetStringValue( i );
        lResultLicense.mType = lLicenseInfo->Value( i ) & 0xFFFF;
        lResultLicense.mSubType = static_cast<uint8_t>( lLicenseInfo->ValueT<uint32_t>( i ) >> 16 );

        lLicenses.push_back( lResultLicense );
    }

    return lLicenses;
}