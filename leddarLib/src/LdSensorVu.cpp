// *****************************************************************************
// Module..: Leddar
//
/// \file    LdSensorVu.cpp
///
/// \brief   Definition of sensor Leddar Vu.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdSensorVu.h"
#include "LdPropertyIds.h"

#include "LdBitFieldProperty.h"
#include "LdBoolProperty.h"
#include "LdEnumProperty.h"
#include "LdFloatProperty.h"
#include "LdIntegerProperty.h"
#include "LdTextProperty.h"

#include "LdConnectionUniversal.h"
#include "LdResultProvider.h"

#define _VU8
#include "comm/PlatformM7DefinitionsShared.h"
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include "comm/registerMap.h"
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
#undef _VU8

#include "LtExceptions.h"
#include "LtStringUtils.h"
#include "LdResultEchoes.h"
#include "LtSystemUtils.h"
#include "LtTimeUtils.h"

#include <cstddef>
#include <cstring>
#include <iomanip>
#include <sstream>

using namespace LeddarCore;
using namespace LeddarConnection;

#define LICENSE_USER_SIZE 32
#define LICENSE_NUMBER 3

using namespace LeddarDevice;

// *****************************************************************************
// Function: LdSensorVu::LdSensorVu
//
/// \brief   Constructor - Take ownership of aConnection (and the 2 pointers used to build it)
///
/// \author  Patrick Boulay
///
/// \since   March 2016
// *****************************************************************************

LdSensorVu::LdSensorVu( LeddarConnection::LdConnection *aConnection ) :
    LdSensor( aConnection ),
    mConnectionUniversal( nullptr ),
    mChannelCount( 0 ),
    mRepair( false ),
    mErrorFlag( false ),
    mBackupFlagAvailable( true )
{
    InitProperties();
    mConnectionUniversal = dynamic_cast<LdConnectionUniversal *>( aConnection );
}

// *****************************************************************************
// Function: LdSensorVu::~LdSensorVu
//
/// \brief   Destructor.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
// *****************************************************************************

LdSensorVu::~LdSensorVu()
{
}


// *****************************************************************************
// Function: LdSensorVu::InitProperties
//
/// \brief   Create properties for this specific sensor.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
// *****************************************************************************
void
LdSensorVu::InitProperties( void )
{
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_DEVICE_TYPE, 0, 2 ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_DEVICE_NAME, 0, REGMAP_PRODUCT_NAME_LENGTH ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_PART_NUMBER, 0, REGMAP_PRODUCT_ID_LENGTH ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_SOFTWARE_PART_NUMBER, 0, REGMAP_PRODUCT_NAME_LENGTH ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_MANUFACTURER_NAME, 0, REGMAP_MFG_NAME_LENGTH ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_SERIAL_NUMBER, 0, REGMAP_SERIAL_NUMBER_LENGTH ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_BUILD_DATE, 0, REGMAP_BUILD_DATE ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_FIRMWARE_VERSION_STR, 0, REGMAP_FIRMWATE_VERSION_LENGTH ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_BOOTLOADER_VERSION, 0, REGMAP_BOOTLOADER_VERSION_LENGTH ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_ASIC_VERSION, 0, REGMAP_ASIC_VERSION_LENGTH ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_FPGA_VERSION, 0, REGMAP_FPGA_VERSION_LENGTH ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_GROUP_ID_NUMBER, 0, REGMAP_GROUP_ID_LENGTH ) );
    mProperties->AddProperty( new LdBitFieldProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_OPTIONS, 0, 4 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_ACCUMULATION_EXP, 0, 1 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_OVERSAMPLING_EXP, 0, 1 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_BASE_POINT_COUNT, 0, 1 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_NB_SAMPLE_MAX, 0, 2 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_VSEGMENT, 0, 2 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_HSEGMENT, 0, 2 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_RSEGMENT, 0, 2 ) );
    mProperties->AddProperty( new LdBitFieldProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_REF_SEG_MASK, 0, 4 ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_BASE_SAMPLE_DISTANCE, 0, 4, 0, 3 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_DETECTION_PER_SEGMENT, 0, 1 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_DISTANCE_SCALE, 0, 4 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_RAW_AMP_SCALE_BITS, 0, 1 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_RAW_AMP_SCALE, 0, 4 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_FILTERED_AMP_SCALE, 0, 4 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_FILTERED_AMP_SCALE_BITS, 0, 1 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_AMP_SCALE, 0, 4 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_PRECISION, 0, 1, "", true ) );
    mProperties->AddProperty( new LdBoolProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_PRECISION_ENABLE, 0 ) );
    mProperties->AddProperty( new LdBitFieldProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_SEGMENT_ENABLE, 0, 4 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_REF_PULSE_RATE, 0, 4 ) );
    mProperties->AddProperty( new LdBoolProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_XTALK_ECHO_REMOVAL_ENABLE, 0 ) );
    mProperties->AddProperty( new LdBoolProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_XTALK_REMOVAL_ENABLE, 0 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_PULSE_RATE, 0, 4 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_CPU_LOAD_SCALE, 0, 4 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_TEMPERATURE_SCALE, 0, 4 ) );
    mProperties->AddProperty( new LdBoolProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_SATURATION_COMP_ENABLE, 0 ) );
    mProperties->AddProperty( new LdBoolProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_OVERSHOOT_MNG_ENABLE, 0 ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_SENSIVITY, 0, 4, 0, 2 ) );
    mProperties->AddProperty( new LdEnumProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_LED_INTENSITY, 0, 4 ) );
    mProperties->AddProperty( new LdBoolProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_LED_PWR_ENABLE, 0 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_LED_PWR_POURCENTAGE, 0, 1 ) );
    mProperties->AddProperty( new LdBitFieldProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_LED_AUTO_PWR_ENABLE, 0, 1 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_LED_AUTO_FRAME_AVG, 0, 2 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_LED_AUTO_ECHO_AVG, 0, 1 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_LEARNED_TRACE_OPTIONS, 0, 1 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_LED_USR_PWR_COUNT, 0, 1 ) );
    mProperties->AddProperty( new LdBoolProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_DEMERGING_ENABLE, 0 ) );
    mProperties->AddProperty( new LdIntegerProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_TRACE_BUFFER_TYPE, 0, 1 ) );
    mProperties->AddProperty( new LdBoolProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_STATIC_NOISE_REMOVAL_ENABLE, 0 ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_REAL_DISTANCE_OFFSET, 0, 4, 0, 3 ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_ORIGIN_X, 0, 4, 0, 3 ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_ORIGIN_Y, 0, 4, 0, 3 ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_ORIGIN_Z, 0, 4, 0, 3 ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_YAW, 0, 4, 0, 3 ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_PITCH, 0, 4, 0, 3 ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_ROLL, 0, 4, 0, 3 ) );
    mProperties->AddProperty( new LdFloatProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_EDITABLE, LdPropertyIds::ID_FOV, 0, 4, 0, 3 ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_CARRIER_FIRMWARE_VERSION, 0, REGMAP_FIRMWATE_VERSION_LENGTH ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_CARRIER_SOFTWARE_PART_NUMBER, 0,
                              REGMAP_FIRMWATE_VERSION_LENGTH ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_CARRIER_PART_NUMBER, 0, REGMAP_PRODUCT_ID_LENGTH ) );
    mProperties->AddProperty( new LdBitFieldProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_CARRIER_OPTIONS, 0, 4 ) );
    mProperties->AddProperty( new LdTextProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_CARRIER_SERIAL_NUMBER, 0, REGMAP_PRODUCT_ID_LENGTH ) );
    mProperties->AddProperty( new LdBitFieldProperty( LdProperty::CAT_CONFIGURATION, LdProperty::F_NONE, LdPropertyIds::ID_TEMP_COMP, 0, 1,
                              "Temperature compensation. Possible value are 0 (no compensation), 1 (reference pulse), 2( compensation table)" ) );
    mProperties->GetBitProperty( LeddarCore::LdPropertyIds::ID_TEMP_COMP )->SetExclusivityMask( 3 ); //0000 0011

    // Result State properties
    GetResultStates()->GetProperties()->AddProperty( new LdFloatProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_RS_SYSTEM_TEMP, 0, 4, 0, 1,
            "System Temperature" ) );
    GetResultStates()->GetProperties()->AddProperty( new LdFloatProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_RS_CPU_LOAD, 0, 4, 0, 1, "CPU Load" ) );
    GetResultStates()->GetProperties()->AddProperty( new LdIntegerProperty( LdProperty::CAT_INFO, LdProperty::F_NONE, LdPropertyIds::ID_RS_BACKUP, 0, 4, "Calibration Backup Flag" ) );
}

// *****************************************************************************
// Function: LdSensorVu::GetConfig
//
/// \brief   Get configuration from the device, store result in the properties.
///
/// \exception LtConfigException  If the configuration value is not valid.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
// *****************************************************************************

void
LdSensorVu::GetConfig()
{
    try
    {
        uint32_t lDistanceScale = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_DISTANCE_SCALE )->ValueT<uint32_t>();

        if( lDistanceScale == 1 )
        {
            throw LeddarException::LtConfigException( "Distance scale should not be 1. Call GetConstants first." );
        }

        uint32_t lAmplitudeScale = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_RAW_AMP_SCALE )->ValueT<uint32_t>();

        if( lAmplitudeScale == 1 )
        {
            throw LeddarException::LtConfigException( "Amplitude scale should not be 1. Call GetConstants first." );
        }

        // ------------- Read configuration data from sensor -------------
        uint8_t *lInputBuffer, *lOutputBuffer;
        mConnectionUniversal->InternalBuffers( lInputBuffer, lOutputBuffer );
        mConnectionUniversal->Read( 0xb, GetBankAddress( REGMAP_CFG_DATA ), sizeof( sCfgData ), 5 );
        sCfgData *lCfgData = reinterpret_cast<sCfgData * >( lOutputBuffer );

        // Device name
        LeddarCore::LdTextProperty *lTextProp = GetProperties()->GetTextProperty( LeddarCore::LdPropertyIds::ID_DEVICE_NAME );
        lTextProp->SetValue( 0, ( char * )lCfgData->mDeviceName );
        lTextProp->SetClean();

        // Accumulation exponent
        LeddarCore::LdIntegerProperty *lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_ACCUMULATION_EXP );
        lIntProp->SetValue( 0, lCfgData->mAccumulationExp );
        lIntProp->SetClean();

        // Oversampling exponent
        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_OVERSAMPLING_EXP );
        lIntProp->SetValue( 0, lCfgData->mOversamplingExp );
        lIntProp->SetClean();

        // Base point count
        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_BASE_POINT_COUNT );
        lIntProp->SetValue( 0, lCfgData->mBasePointCount );
        lIntProp->SetClean();

        // Segment enable
        LeddarCore::LdBitFieldProperty *lBitProp = GetProperties()->GetBitProperty( LeddarCore::LdPropertyIds::ID_SEGMENT_ENABLE );
        lBitProp->SetValue( 0, lCfgData->mSegmentEnable );
        lBitProp->SetClean();

        // Reference pulse rate
        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_REF_PULSE_RATE );
        lIntProp->SetValue( 0, lCfgData->mRefPulseRate );
        lIntProp->SetClean();

        // Origin X
        LeddarCore::LdFloatProperty *lFloatProp = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_ORIGIN_X );
        lFloatProp->SetValue( 0, lCfgData->mX );
        lFloatProp->SetClean();

        // Origin Y
        lFloatProp = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_ORIGIN_Y );
        lFloatProp->SetValue( 0, lCfgData->mY );
        lFloatProp->SetClean();

        // Origin Z
        lFloatProp = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_ORIGIN_Z );
        lFloatProp->SetValue( 0, lCfgData->mZ );
        lFloatProp->SetClean();

        // YAW
        lFloatProp = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_YAW );
        lFloatProp->SetValue( 0, lCfgData->mYaw );
        lFloatProp->SetClean();

        // Pitch
        lFloatProp = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_PITCH );
        lFloatProp->SetValue( 0, lCfgData->mPitch );
        lFloatProp->SetClean();

        // Roll
        lFloatProp = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_ROLL );
        lFloatProp->SetValue( 0, lCfgData->mRoll );
        lFloatProp->SetClean();

        // Precision
        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_PRECISION );
        lIntProp->SetValue( 0, lCfgData->mPrecision );
        lIntProp->SetClean();

        // Precision enable
        LeddarCore::LdBoolProperty *lBoolProp = GetProperties()->GetBoolProperty( LeddarCore::LdPropertyIds::ID_PRECISION_ENABLE );
        lBoolProp->SetValue( 0, lCfgData->mPrecisionEnable == 1 );
        lBoolProp->SetClean();

        // Saturation compensation enable
        lBoolProp = GetProperties()->GetBoolProperty( LeddarCore::LdPropertyIds::ID_SATURATION_COMP_ENABLE );
        lBoolProp->SetValue( 0, lCfgData->mSatCompEnable == 1 );
        lBoolProp->SetClean();

        // Overshot managment enable
        lBoolProp = GetProperties()->GetBoolProperty( LeddarCore::LdPropertyIds::ID_OVERSHOOT_MNG_ENABLE );
        lBoolProp->SetValue( 0, lCfgData->mOvershootManagementEnable == 1 );
        lBoolProp->SetClean();

        // Sensivity
        lFloatProp = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_SENSIVITY );
        lFloatProp->SetScale( lAmplitudeScale );
        lFloatProp->SetRawValue( 0, lCfgData->mSensitivity );
        lFloatProp->SetClean();

        // LED power (in percent)
        LeddarCore::LdIntegerProperty *lLedPwdPourcentage = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_LED_PWR_POURCENTAGE );
        lLedPwdPourcentage->SetValue( 0, lCfgData->mLedUserCurrentPowerPercent );
        lLedPwdPourcentage->SetClean();
        uint8_t lLedIntensitySelected = lCfgData->mLedUserCurrentPowerPercent;

        // Auto LED power enable
        lBitProp = GetProperties()->GetBitProperty( LeddarCore::LdPropertyIds::ID_LED_AUTO_PWR_ENABLE );
        lBitProp->SetValue( 0, lCfgData->mLedUserAutoPowerEnable );
        lBitProp->SetClean();

        // Auto LED frame average
        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_LED_AUTO_FRAME_AVG );
        lIntProp->SetValue( 0, lCfgData->mLedUserAutoFrameAvg );
        lIntProp->SetClean();

        // Auto LED echo average
        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_LED_AUTO_ECHO_AVG );
        lIntProp->SetValue( 0, lCfgData->mLedUserAutoEchoAvg );
        lIntProp->SetClean();

        // Demerging enable
        lBoolProp = GetProperties()->GetBoolProperty( LeddarCore::LdPropertyIds::ID_DEMERGING_ENABLE );
        lBoolProp->SetValue( 0, lCfgData->mDemEnable == 1 );
        lBoolProp->SetClean();

        // Static noise removal enable
        lBoolProp = GetProperties()->GetBoolProperty( LeddarCore::LdPropertyIds::ID_STATIC_NOISE_REMOVAL_ENABLE );
        lBoolProp->SetValue( 0, lCfgData->mStNoiseRmvEnable == 1 );
        lBoolProp->SetClean();

        // ------------- Read advanced config data from sensor (part 1) -------------
        mConnectionUniversal->Read( 0xb, GetBankAddress( REGMAP_ADV_CFG_DATA ) + offsetof( sAdvCfgData, mTraceBufferType ),
                                    sizeof( uint8_t ) + sizeof( uint32_t ), 0 );

        // Trace buffer type
        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_TRACE_BUFFER_TYPE );
        lIntProp->SetValue( 0, *( ( uint8_t * )lOutputBuffer ) );
        lIntProp->SetClean();

        // Field of view
        lFloatProp = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_FOV );
        lFloatProp->SetScale( lDistanceScale );
        lFloatProp->SetRawValue( 0, *( ( uint32_t * )( &lOutputBuffer[ 1 ] ) ) );
        lFloatProp->SetClean();

        // -------------  Read advanced config data from sensor (part 2) -------------
        mConnectionUniversal->Read( 0xb, GetBankAddress( REGMAP_ADV_CFG_DATA ) + offsetof( sAdvCfgData, mPeakFilterSumBits ),
                                    sizeof( uint8_t ), 5 );

        // Filtered amplitude scale
        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_FILTERED_AMP_SCALE_BITS );
        lIntProp->SetValue( 0, *( lOutputBuffer ) );
        lIntProp->SetClean();
        uint8_t lFilteredAmpScaleBits = lIntProp->ValueT<uint8_t>();

        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_FILTERED_AMP_SCALE );
        lIntProp->SetValue( 0, 1 << lFilteredAmpScaleBits );
        lIntProp->SetClean();


        // Amplitude scale
        uint8_t lRawAmplBit = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_RAW_AMP_SCALE_BITS )->ValueT<uint8_t>();
        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_AMP_SCALE );
        lIntProp->SetValue( 0, 1 << ( lRawAmplBit + lFilteredAmpScaleBits ) );
        lIntProp->SetClean();

        // -------------  Read advanced config data from sensor (part 3) -------------
        mConnectionUniversal->Read( 0xb, GetBankAddress( REGMAP_ADV_CFG_DATA ) + offsetof( sAdvCfgData, mLedUserPowerEnable ),
                                    offsetof( sAdvCfgData, mLedUserPowerLut ) - offsetof( sAdvCfgData, mLedUserPowerEnable ), 5 );

        // User led power count
        uint8_t lLedPwrCount = *( lOutputBuffer + offsetof( sAdvCfgData, mLedUsrPowerCount ) - offsetof( sAdvCfgData, mLedUserPowerEnable ) );
        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_LED_USR_PWR_COUNT );
        lIntProp->SetValue( 0, lLedPwrCount );
        lBoolProp->SetClean();

        // User led power enable
        lBoolProp = GetProperties()->GetBoolProperty( LeddarCore::LdPropertyIds::ID_LED_PWR_ENABLE );
        lBoolProp->SetValue( 0, *( lOutputBuffer ) == 1 );
        lBoolProp->SetClean();

        // -------------  Read advanced config data from sensor (part 4) -------------
        mConnectionUniversal->Read( 0xb, GetBankAddress( REGMAP_ADV_CFG_DATA ) + offsetof( sAdvCfgData, mLedUserPercentLut ),
                                    offsetof( sAdvCfgData, mDemAmpThrMin ) - offsetof( sAdvCfgData, mLedUserPercentLut ), 5 );

        // User led power lookup table
        uint8_t *lLedPwrLUT = lOutputBuffer;
        LeddarCore::LdEnumProperty *lLedIntensityProp = GetProperties()->GetEnumProperty( LeddarCore::LdPropertyIds::ID_LED_INTENSITY );
        lLedIntensityProp->SetEnumSize( lLedPwrCount );

        for( int i = 0; i < lLedPwrCount; ++i )
        {
            lLedIntensityProp->AddEnumPair( lLedPwrLUT[ i ], LeddarUtils::LtStringUtils::IntToString( lLedPwrLUT[ i ] ) );
        }


        try
        {
            lLedIntensityProp->SetValue( 0, lLedIntensitySelected );
        }
        catch( std::exception /*&e*/ )
        {
            // Invalid intensity, find the closest one
            size_t lClosestIndex = 0;
            int8_t lDeltaIntensity = 100;

            for( size_t i = 0; i < lLedIntensityProp->EnumSize(); ++i )
            {
                int8_t lDelta = lLedIntensitySelected - lLedIntensityProp->EnumValue( i );

                if( lDelta < 0 )
                    lDelta = -lDelta;

                if( lDelta <= lDeltaIntensity )
                {
                    lDeltaIntensity = lDelta;
                    lClosestIndex = i;
                }
            }

            lLedIntensityProp->SetValue( 0, lLedIntensityProp->EnumValue( lClosestIndex ) );
            lLedPwdPourcentage->SetValue( 0, lLedIntensityProp->EnumValue( lClosestIndex ) );
            mRepair = true;
        }

        lLedIntensityProp->SetClean();

        // -------------  Read advanced config data from sensor (part 5) -------------
        mConnectionUniversal->Read( 0xb, GetBankAddress( REGMAP_ADV_CFG_DATA ) + offsetof( sAdvCfgData, mPeakRealDistanceOffset ),
                                    offsetof( sAdvCfgData, mPeakNbSampleForBaseLevEst ) - offsetof( sAdvCfgData, mPeakRealDistanceOffset ), 5 );

        // Real distance offset
        uint32_t *lPeakRealDistanceOffset = ( uint32_t * )( lOutputBuffer );
        lFloatProp = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_REAL_DISTANCE_OFFSET );
        lFloatProp->SetScale( lDistanceScale );
        lFloatProp->SetCount( 1 );
        lFloatProp->SetRawValue( 0, *lPeakRealDistanceOffset );

        // -------------  Read advanced config data from sensor (part 6) -------------
        LeddarCore::LdBitFieldProperty *lBitFieldProp = GetProperties()->GetBitProperty( LeddarCore::LdPropertyIds::ID_TEMP_COMP );
        lBitFieldProp->SetValue( 0, 0 );
        mConnectionUniversal->Read( 0xb, GetBankAddress( REGMAP_ADV_CFG_DATA ) + offsetof( sAdvCfgData, mPeakRefDistEnable ), sizeof( uint8_t ), 5 );

        if( *lOutputBuffer != 0 )
        {
            lBitFieldProp->SetBit( 0, 0 );
        }

        mConnectionUniversal->Read( 0xb, GetBankAddress( REGMAP_ADV_CFG_DATA ) + offsetof( sAdvCfgData, mPeakTempEnable ), sizeof( uint8_t ), 5 );

        if( *lOutputBuffer != 0 )
        {
            lBitFieldProp->SetBit( 0, 1 );
        }

        // Initialize results.
        uint32_t lTotalSegments = mProperties->GetIntegerProperty( LdPropertyIds::ID_VSEGMENT )->ValueT<uint16_t>() *
                                  mProperties->GetIntegerProperty( LdPropertyIds::ID_HSEGMENT )->ValueT<uint16_t>();
        uint32_t lMaxDetections = lTotalSegments * mProperties->GetIntegerProperty( LdPropertyIds::ID_DETECTION_PER_SEGMENT )->ValueT<uint32_t>();
        uint32_t lTempeatureScale = mProperties->GetIntegerProperty( LdPropertyIds::ID_TEMPERATURE_SCALE )->ValueT<uint32_t>();
        uint32_t lCpuLoadScale = mProperties->GetIntegerProperty( LdPropertyIds::ID_CPU_LOAD_SCALE )->ValueT<uint32_t>();
        mEchoes.Init( mProperties->GetIntegerProperty( LdPropertyIds::ID_DISTANCE_SCALE )->ValueT<uint32_t>(),
                      mProperties->GetIntegerProperty( LdPropertyIds::ID_AMP_SCALE )->ValueT<uint32_t>(),
                      lMaxDetections );
        mStates.Init( lTempeatureScale, lCpuLoadScale );

    }
    catch( std::exception &e )
    {
        throw LeddarException::LtConfigException( e );
    }
}

// *****************************************************************************
// Function: LdSensorVu::SetConfig
//
/// \brief   Set configuration to the device, store result in the properties.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
// *****************************************************************************

void
LdSensorVu::SetConfig()
{
    try
    {
        uint8_t *lInputBuffer, *lOutputBuffer;
        mConnectionUniversal->InternalBuffers( lInputBuffer, lOutputBuffer );
        sCfgData *lCfgData = reinterpret_cast< sCfgData *>( lInputBuffer );

        // ------------- Set configuration data in sensor -------------
        // Device name
        LeddarCore::LdTextProperty *lTextProp = GetProperties()->GetTextProperty( LeddarCore::LdPropertyIds::ID_DEVICE_NAME );
        memcpy( lCfgData->mDeviceName, lTextProp->Value().c_str(), REGMAP_PRODUCT_NAME_LENGTH );

        // Accumulation Exponent
        LeddarCore::LdIntegerProperty *lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_ACCUMULATION_EXP );
        lCfgData->mAccumulationExp = ( uint8_t )lIntProp->Value();

        // Oversampling exponent
        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_OVERSAMPLING_EXP );
        lCfgData->mOversamplingExp = ( uint8_t )lIntProp->Value();

        // Base point count
        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_BASE_POINT_COUNT );
        lCfgData->mBasePointCount = ( uint8_t )lIntProp->Value();

        // Segment enable
        LeddarCore::LdBitFieldProperty *lBitFieldProp = GetProperties()->GetBitProperty( LeddarCore::LdPropertyIds::ID_SEGMENT_ENABLE );
        lCfgData->mSegmentEnable = lBitFieldProp->Value();

        // Reference Pulse Rate
        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_REF_PULSE_RATE );
        lCfgData->mRefPulseRate = lIntProp->ValueT<uint32_t>();

        // Origin X
        LeddarCore::LdFloatProperty *lFloatProp = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_ORIGIN_X );
        lCfgData->mX = lFloatProp->Value();

        // Origin Y
        lFloatProp = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_ORIGIN_Y );
        lCfgData->mY = lFloatProp->Value();

        // Origin Z
        lFloatProp = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_ORIGIN_Z );
        lCfgData->mZ = lFloatProp->Value();

        // YAW
        lFloatProp = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_YAW );
        lCfgData->mYaw = lFloatProp->Value();

        // Pitch
        lFloatProp = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_PITCH );
        lCfgData->mPitch = lFloatProp->Value();

        // Roll
        lFloatProp = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_ROLL );
        lCfgData->mRoll = lFloatProp->Value();

        // Precision
        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_PRECISION );
        lCfgData->mPrecision = ( int8_t )lIntProp->Value();

        // Precision enable
        LeddarCore::LdBoolProperty *lBoolProp = GetProperties()->GetBoolProperty( LeddarCore::LdPropertyIds::ID_PRECISION_ENABLE );
        lCfgData->mPrecisionEnable = ( uint8_t )( lBoolProp->Value() == true ? 1 : 0 );

        // Saturation compensation enable
        lBoolProp = GetProperties()->GetBoolProperty( LeddarCore::LdPropertyIds::ID_SATURATION_COMP_ENABLE );
        lCfgData->mSatCompEnable = ( uint8_t )( lBoolProp->Value() == true ? 1 : 0 );

        // Ovrshoot management enable
        lBoolProp = GetProperties()->GetBoolProperty( LeddarCore::LdPropertyIds::ID_OVERSHOOT_MNG_ENABLE );
        lCfgData->mOvershootManagementEnable = ( uint8_t )( lBoolProp->Value() == true ? 1 : 0 );

        // Sensivity setting
        lFloatProp = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_SENSIVITY );
        lCfgData->mSensitivity = lFloatProp->RawValue();

        // Current LED power level
        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_LED_PWR_POURCENTAGE );
        lCfgData->mLedUserCurrentPowerPercent = ( uint8_t )lIntProp->Value();

        // Auto LED power enable
        LeddarCore::LdBitFieldProperty *lBitProplBitProp = GetProperties()->GetBitProperty( LeddarCore::LdPropertyIds::ID_LED_AUTO_PWR_ENABLE );
        lCfgData->mLedUserAutoPowerEnable = lBitProplBitProp->Value();

        // Auto frame average
        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_LED_AUTO_FRAME_AVG );
        lCfgData->mLedUserAutoFrameAvg = ( uint16_t )lIntProp->Value();

        // Auto echo average
        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_LED_AUTO_ECHO_AVG );
        lCfgData->mLedUserAutoEchoAvg = ( uint8_t )lIntProp->Value();

        // Demerging enable
        lBoolProp = GetProperties()->GetBoolProperty( LeddarCore::LdPropertyIds::ID_DEMERGING_ENABLE );
        lCfgData->mDemEnable = ( uint8_t )( lBoolProp->Value() == true ? 1 : 0 );

        // Static noise removal enable
        lBoolProp = GetProperties()->GetBoolProperty( LeddarCore::LdPropertyIds::ID_STATIC_NOISE_REMOVAL_ENABLE );
        lCfgData->mStNoiseRmvEnable = ( uint8_t )( lBoolProp->Value() == true ? 1 : 0 );

        // Write config data into the sensor.
        mConnectionUniversal->Write( 0x2, GetBankAddress( REGMAP_CFG_DATA ), sizeof( sCfgData ), 5 );

        // -------------  Write advanced config data from sensor (part 6 of GetConfig) -------------
        //Need an integrator license to write it
        std::vector<LeddarDefines::sLicense> lLicenses = GetLicenses();

        for( size_t i = 0; i < lLicenses.size(); i++ )
        {
            if( lLicenses[i].mType == LICENSE_INTEGRATOR || lLicenses[i].mType == LICENSE_ADMIN )
            {
                lBitFieldProp = GetProperties()->GetBitProperty( LeddarCore::LdPropertyIds::ID_TEMP_COMP );
                *lInputBuffer = lBitFieldProp->BitState( 0, 0 );
                mConnectionUniversal->Write( 0x2, GetBankAddress( REGMAP_ADV_CFG_DATA ) + offsetof( sAdvCfgData, mPeakRefDistEnable ), sizeof( uint8_t ), 5 );
                *lInputBuffer = lBitFieldProp->BitState( 0, 1 );
                mConnectionUniversal->Write( 0x2, GetBankAddress( REGMAP_ADV_CFG_DATA ) + offsetof( sAdvCfgData, mPeakTempEnable ), sizeof( uint8_t ), 5 );
                break;
            }
        }


    }
    catch( ... )
    {
        throw;
    }
}

// *****************************************************************************
// Function: LdSensorVu::GetConstants
//
/// \brief   Get constants from the device, store result in the properties.
///
/// \exception LtConfigException If a constant value is not valid.
///
/// \author  Patrick Boulay
/// \author  Vincent Simard Bilodeau
///
/// \since   April 2016
// *****************************************************************************

void
LdSensorVu::GetConstants()
{
    try
    {
        // Get communication buffer.
        uint8_t *lInputBuffer, *lOutputBuffer;
        mConnectionUniversal->InternalBuffers( lInputBuffer, lOutputBuffer );

        // ---- DEVICE INFO ----
        mConnectionUniversal->Read( 0xb, GetBankAddress( REGMAP_DEV_INFO ), sizeof( sDevInfo ), 5 );
        sDevInfo *lDevInfo = reinterpret_cast< sDevInfo * >( lOutputBuffer );

        LeddarCore::LdIntegerProperty *lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_DEVICE_TYPE );
        lIntProp->SetValue( 0, lDevInfo->mDeviceType );
        lIntProp->SetClean();

        LeddarCore::LdTextProperty *lTextProp = GetProperties()->GetTextProperty( LeddarCore::LdPropertyIds::ID_PART_NUMBER );
        lTextProp->SetValue( 0, lDevInfo->mPartNumber );
        lTextProp->SetClean();

        lTextProp = GetProperties()->GetTextProperty( LeddarCore::LdPropertyIds::ID_SOFTWARE_PART_NUMBER );
        lTextProp->SetValue( 0, lDevInfo->mSoftPartNumber );
        lTextProp->SetClean();

        lTextProp = GetProperties()->GetTextProperty( LeddarCore::LdPropertyIds::ID_MANUFACTURER_NAME );
        lTextProp->SetValue( 0, lDevInfo->mMfgName );
        lTextProp->SetClean();

        lTextProp = GetProperties()->GetTextProperty( LeddarCore::LdPropertyIds::ID_SERIAL_NUMBER );
        lTextProp->SetValue( 0, lDevInfo->mSerialNumber );
        lTextProp->SetClean();

        lTextProp = GetProperties()->GetTextProperty( LeddarCore::LdPropertyIds::ID_BUILD_DATE );
        lTextProp->SetValue( 0, lDevInfo->mBuildDate );
        lTextProp->SetClean();

        lTextProp = GetProperties()->GetTextProperty( LeddarCore::LdPropertyIds::ID_FIRMWARE_VERSION_STR );
        lTextProp->SetValue( 0, lDevInfo->mFirmwareVersion );
        lTextProp->SetClean();

        lTextProp = GetProperties()->GetTextProperty( LeddarCore::LdPropertyIds::ID_BOOTLOADER_VERSION );
        lTextProp->SetValue( 0, lDevInfo->mBootldVersion );
        lTextProp->SetClean();

        lTextProp = GetProperties()->GetTextProperty( LeddarCore::LdPropertyIds::ID_ASIC_VERSION );
        lTextProp->SetValue( 0, lDevInfo->mASICVersion );
        lTextProp->SetClean();

        lTextProp = GetProperties()->GetTextProperty( LeddarCore::LdPropertyIds::ID_FPGA_VERSION );
        lTextProp->SetValue( 0, lDevInfo->mFPGAVersion );
        lTextProp->SetClean();

        lTextProp = GetProperties()->GetTextProperty( LeddarCore::LdPropertyIds::ID_GROUP_ID_NUMBER );
        lTextProp->SetValue( 0, lDevInfo->mGroupIdenficationNumber );
        lTextProp->SetClean();

        LeddarCore::LdBitFieldProperty *lBitProp = GetProperties()->GetBitProperty( LeddarCore::LdPropertyIds::ID_OPTIONS );
        lBitProp->SetValue( 0, lDevInfo->mOptions );
        lBitProp->SetClean();

        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_ACCUMULATION_EXP );
        lIntProp->SetLimits( lDevInfo->mAccExpMin, lDevInfo->mAccExpMax );
        lIntProp->SetClean();

        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_OVERSAMPLING_EXP );
        lIntProp->SetLimits( lDevInfo->mOvrExpMin, lDevInfo->mOvrExpMax );
        lIntProp->SetClean();

        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_BASE_POINT_COUNT );
        lIntProp->SetLimits( lDevInfo->mBasePointMin, lDevInfo->mBasePointMax );
        lIntProp->SetClean();

        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_HSEGMENT );
        lIntProp->SetValue( 0, lDevInfo->mNbHonrizontalSegment );
        lIntProp->SetClean();

        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_VSEGMENT );
        lIntProp->SetValue( 0, lDevInfo->mNbVerticalSegment );
        lIntProp->SetClean();

        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_RSEGMENT );
        lIntProp->SetValue( 0, lDevInfo->mNbRefSegment );
        lIntProp->SetClean();

        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_NB_SAMPLE_MAX );
        lIntProp->SetValue( 0, lDevInfo->mNbSampleMax );
        lIntProp->SetClean();

        lBitProp = GetProperties()->GetBitProperty( LeddarCore::LdPropertyIds::ID_REF_SEG_MASK );
        lBitProp->SetValue( 0, lDevInfo->mRefSegMask );
        lBitProp->SetClean();

        LeddarCore::LdFloatProperty *lFloatProp = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_BASE_SAMPLE_DISTANCE );
        lFloatProp->SetScale( lDevInfo->mDistanceScale );
        lFloatProp->SetRawValue( 0, lDevInfo->mBaseSplDist );
        lIntProp->SetClean();

        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_DETECTION_PER_SEGMENT );
        lIntProp->SetValue( 0, lDevInfo->mDetectionPerSegmentCountMax );
        lIntProp->SetClean();

        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_DISTANCE_SCALE );
        lIntProp->SetValue( 0, lDevInfo->mDistanceScale );
        lIntProp->SetClean();

        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_RAW_AMP_SCALE_BITS );
        lIntProp->SetValue( 0, lDevInfo->mRawAmplitudeScaleBits );
        lIntProp->SetClean();

        uint32_t lAmplitudeScale = lDevInfo->mRawAmplitudeScale;
        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_RAW_AMP_SCALE );
        lIntProp->SetValue( 0, lDevInfo->mRawAmplitudeScale );
        lIntProp->SetClean();

        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_PRECISION );
        lIntProp->SetLimits( lDevInfo->mPrecisionMin, lDevInfo->mPrecisionMax );
        lIntProp->SetClean();

        lFloatProp = GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_SENSIVITY );
        lFloatProp->SetScale( lAmplitudeScale );
        lFloatProp->SetRawLimits( lDevInfo->mSensitivitytMin, lDevInfo->mSensitivitytMax );
        lFloatProp->SetClean();

        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_LED_AUTO_FRAME_AVG );
        lIntProp->SetLimits( lDevInfo->mLedUserAutoFrameAvgMin, lDevInfo->mLedUserAutoFrameAvgMax );
        lIntProp->SetClean();

        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_LED_PWR_POURCENTAGE );
        lIntProp->SetLimits( lDevInfo->mLedUserPowerPercentMin, lDevInfo->mLedUserPowerPercentMax );
        lIntProp->SetClean();

        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_LED_AUTO_ECHO_AVG );
        lIntProp->SetLimits( lDevInfo->mLedUserAutoEchoAvgMin, lDevInfo->mLedUserAutoEchoAvgMax );
        lIntProp->SetClean();

        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_LEARNED_TRACE_OPTIONS );
        lIntProp->SetValue( 0, lDevInfo->mStNoiseRmvCalibBy );
        lIntProp->SetClean();

        lIntProp = GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_CPU_LOAD_SCALE );
        lIntProp->SetValue( 0, lDevInfo->mCpuLoadScale );
        lIntProp->SetClean();
        GetResultStates()->GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_RS_CPU_LOAD )->SetScale( lDevInfo->mCpuLoadScale );

        mChannelCount = lDevInfo->mNbVerticalSegment * lDevInfo->mNbHonrizontalSegment + lDevInfo->mNbRefSegment;

        if( GetCarrier() != nullptr )
        {
            GetCarrier()->GetConstants();
        }

    }
    catch( std::exception &e )
    {
        throw LeddarException::LtConfigException( e );
    }
}

// *****************************************************************************
// Function: LdSensorVu::GetEchoes
//
/// \brief   Get echoes from the sensor and fill the result object.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
// *****************************************************************************

bool
LdSensorVu::GetEchoes()
{
    // Get echo data storage
    LdResultEchoes *lResultEchoes = GetResultEchoes();
    uint16_t lMaxEchoes = REGMAP_MAX_ECHOES_PER_CHANNEL * mChannelCount;

    // Get comm buffer
    uint8_t *lInputBuffer;
    uint8_t *lOutputBuffer;
    mConnectionUniversal->InternalBuffers( lInputBuffer, lOutputBuffer );

    try
    {
        uint32_t lTimeStamp = 0;
        uint16_t lEchoCount = 0;
        uint16_t lCurrentLwdPower = 0;
        uint16_t lEchoCountToRead = 0;
        //Static variables for robustness in a rare case where comm is stuck (only with a FTDI/SPI cable)
        //And we need to reset the transfer mode
        static int sStuckCounter = 0;
        static int sStuckMax = -1;

        // If last trasaction has failed, reset register locking
        // by resetting the partial blocking mode.
        if( mErrorFlag == true )
        {
            uint8_t lMode = 2;
            mConnectionUniversal->WriteRegister( GetBankAddress( REGMAP_TRN_CFG ) + offsetof( sTransactionCfg, mTransferMode ), &lMode, 1, 5 );
            mErrorFlag = false;
        }

        // Get the timestamp and the echoes number
        mConnectionUniversal->Read( 0xb, GetBankAddress( REGMAP_CMD_LIST ) + offsetof( sCmdList, mDetectionReady ), sizeof( ( ( sCmdList * )0 )->mDetectionReady ), 1 );

        if( lOutputBuffer[ 0 ] == 1 )
        {
            mConnectionUniversal->Read( 0xb, GetBankAddress( REGMAP_DETECTIONS ), offsetof( sDetections, mEchoes ), 1 );
            lTimeStamp = *( reinterpret_cast<uint32_t *>( lOutputBuffer + offsetof( sDetections, mTimestamp ) ) );
            lEchoCount = *( reinterpret_cast<uint16_t *>( lOutputBuffer + offsetof( sDetections, mNbDetection ) ) );
            lCurrentLwdPower = *( reinterpret_cast<uint16_t *>( lOutputBuffer + offsetof( sDetections, mCurrentUsrLedPower ) ) );
            lEchoCountToRead = lEchoCount;
            sStuckMax = sStuckCounter;
            sStuckCounter = 0;
        }
        else
        {
            sStuckCounter++;

            if( sStuckMax >= 0 && sStuckCounter > sStuckMax * 10 && sStuckCounter > sStuckMax + 10 )
            {
                mErrorFlag = true;
                sStuckMax = -1;
            }

            return false;
        }

        if( lEchoCount > ( lMaxEchoes ) )
        {
            return false;
        }

        if( lResultEchoes->GetTimestamp( LeddarConnection::B_GET ) != lTimeStamp )
        {
            lResultEchoes->SetTimestamp( lTimeStamp );
            uint32_t lEchoStartAddr = GetBankAddress( REGMAP_DETECTIONS ) + offsetof( sDetections, mEchoes );

            // Get echoes. If the size is over 512 bytes, do multiple read.
            while( lEchoCountToRead > 0 )
            {
                uint16_t lEchoCountToReadNow = lEchoCountToRead;

                if( sizeof( sEchoLigth )* lEchoCountToReadNow > 512 )
                {
                    lEchoCountToReadNow = 512 / sizeof( sEchoLigth );
                    lEchoCountToRead -= lEchoCountToReadNow;
                }
                else
                {
                    lEchoCountToRead -= lEchoCountToReadNow;
                }

                mConnectionUniversal->Read( 0xb, lEchoStartAddr, sizeof( sEchoLigth )* lEchoCountToReadNow, 1, 5000 );
                sEchoLigth *lDetections = reinterpret_cast<sEchoLigth *>( lOutputBuffer );
                std::vector<LdEcho> *lEchoes = lResultEchoes->GetEchoes( LeddarConnection::B_SET );

                for( int i = 0; i < lEchoCountToReadNow; ++i )
                {
                    ( *lEchoes )[ i ].mChannelIndex = lDetections[ i ].mSegment;
                    ( *lEchoes )[ i ].mDistance = lDetections[ i ].mDistance;
                    ( *lEchoes )[ i ].mAmplitude = lDetections[ i ].mAmplitude;
                    ( *lEchoes )[ i ].mFlag = lDetections[ i ].mFlag;
                }
            }

            lResultEchoes->SetEchoCount( lEchoCount );
            lResultEchoes->SetCurrentLedPower( lCurrentLwdPower );
        }
        else
        {
            return false;
        }
    }
    catch( ... )
    {
        mErrorFlag = true;
        throw;
    }

    lResultEchoes->Swap();
    lResultEchoes->UpdateFinished();
    return true;
}

// *****************************************************************************
// Function: LdSensorVu::GetStates
//
/// \brief   Get the states from the device.
///
/// \exception std::runtime_error If the device is not connected.
///
/// \author  Vincent Simard Bilodeau
///
/// \since   August 2016
// *****************************************************************************

void
LdSensorVu::GetStates()
{
    // Get state data storage
    LdResultStates &aResultStates = *GetResultStates();

    //We cannot get the timestamp on its own, so we copy it from the echoes
    uint32_t lOldTimestamp = aResultStates.GetTimestamp();
    uint32_t lNewTimestamp = mEchoes.GetTimestamp();
    assert( lNewTimestamp ); //We didnt get any echoes yet, so we dont know the timestamp

    if( lOldTimestamp == lNewTimestamp )
    {
        return;
    }

    aResultStates.SetTimestamp( lNewTimestamp );

    if( mErrorFlag == true )
    {
        uint8_t lMode = 2;
        mConnectionUniversal->WriteRegister( GetBankAddress( REGMAP_TRN_CFG ) + offsetof( sTransactionCfg, mTransferMode ), &lMode, 1, 5 );
        mErrorFlag = false;
    }

    try
    {
        // Read state from device
        uint32_t lCpuLoad;
        mConnectionUniversal->ReadRegister( GetBankAddress( REGMAP_CMD_LIST ) + offsetof( sCmdList, mCpuUsage ), ( uint8_t * )&lCpuLoad, sizeof( lCpuLoad ), 5 );
        aResultStates.GetProperties()->GetFloatProperty( LeddarCore::LdPropertyIds::ID_RS_CPU_LOAD )->SetRawValue( 0, lCpuLoad );

        if( mBackupFlagAvailable )
        {
            try
            {
                // Read backup flag
                uint32_t lBackupFlag;
                mConnectionUniversal->ReadRegister( GetBankAddress( REGMAP_CMD_LIST ) + offsetof( sCmdList, mBackupStatus ), ( uint8_t * )&lBackupFlag, sizeof( lBackupFlag ), 5 );
                aResultStates.GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_RS_BACKUP )->SetValue( 0, lBackupFlag );
            }
            catch( ... )
            {
                mBackupFlagAvailable = false;
                aResultStates.GetProperties()->GetIntegerProperty( LeddarCore::LdPropertyIds::ID_RS_BACKUP )->SetValue( 0, 0 );
                throw LeddarException::LtException( "Error to read the calibration backup flag, please update your sensor firmware." );
            }
        }
    }
    catch( ... )
    {
        mErrorFlag = true;
        throw;
    }

    // Emit state update completed
    aResultStates.UpdateFinished();
}


// *****************************************************************************
// Function: LdSensorVu::GetLicenses
//
/// \brief   Get the licenses on the device.
///
/// \return  Vector of sLicense
///
/// \author  Patrick Boulay
///
/// \since   May 2016
// *****************************************************************************
std::vector<LeddarDefines::sLicense>
LdSensorVu::GetLicenses( void )
{
    std::vector<LeddarDefines::sLicense> lLicenses;

    try
    {
        // Read license and info
        uint8_t lLicenseKey[ 3 ][ 16 ];
        uint32_t lLicenseInfo[ 3 ];
        mConnectionUniversal->ReadRegister( GetBankAddress( REGMAP_LICENSE_KEYS ), ( uint8_t * )lLicenseKey, REGMAP_KEY_LENGTH * LICENSE_NUMBER, 1 );
        mConnectionUniversal->ReadRegister( GetBankAddress( REGMAP_CMD_LIST ) + offsetof( sCmdList, mLicenceInfo ), ( uint8_t * )lLicenseInfo, sizeof( uint32_t )* LICENSE_NUMBER, 1 );

        for( uint8_t i = 0; i < LICENSE_NUMBER; ++i )
        {
            LeddarDefines::sLicense lLicense;
            lLicense.mType = lLicenseInfo[ i ] & 0xFFFF;
            lLicense.mSubType = lLicenseInfo[ i ] >> 16;

            for( uint8_t j = 0; j < REGMAP_KEY_LENGTH; ++j )
            {
                std::stringstream lStream;
                lStream << std::setfill( '0' ) << std::setw( 2 ) << std::hex << ( uint32_t )lLicenseKey[ i ][ j ];
                lLicense.mLicense += lStream.str();
            }

            lLicenses.push_back( lLicense );
        }
    }
    catch( ... )
    {
        throw;
    }

    return lLicenses;
}

// *****************************************************************************
// Function: LdSensorVu::SendLicense
//
/// \brief   Send license key to the device.
///
/// \param  aLicense    License to send
///
/// \author  Patrick Boulay
///
/// \since   May 2016
// *****************************************************************************
LeddarDefines::sLicense
LdSensorVu::SendLicense( const std::string &aLicense )
{

    // Get licences from device
    if( aLicense.length() != LICENSE_USER_SIZE )
        throw std::runtime_error( "Invalid license." );

    LeddarDefines::sLicense lResultLicense;
    std::vector<LeddarDefines::sLicense> lLicenses = GetLicenses();

    // Looking for empty slot
    uint8_t lBuffer[ 16 ];
    uint32_t lEmptySlotIndex = 0;

    for( lEmptySlotIndex = 0; lEmptySlotIndex < lLicenses.size(); ++lEmptySlotIndex )
    {
        if( lLicenses[ lEmptySlotIndex ].mType != 0 && lLicenses[ lEmptySlotIndex ].mLicense == LeddarUtils::LtStringUtils::ToLower( aLicense ) )
        {
            throw std::runtime_error( "License already on the device." );
        }

        if( lLicenses[ lEmptySlotIndex ].mType == 0 )
        {
            break;
        }
    }

    if( lEmptySlotIndex == LICENSE_NUMBER )
    {
        throw std::runtime_error( "No empty license slot available on the device." );
    }

    // Convert the user string to 16 bytes license
    for( size_t i = 0; i < aLicense.length(); i += 2 )
    {
        lBuffer[ i / 2 ] = ( uint8_t )strtoul( aLicense.substr( i, 2 ).c_str(), nullptr, 16 );
    }

    mConnectionUniversal->SetWriteEnable( true );

    // Write the license on the device
    try
    {
        mConnectionUniversal->WriteRegister( GetBankAddress( REGMAP_LICENSE_KEYS ) + ( lEmptySlotIndex * REGMAP_KEY_LENGTH ), lBuffer, sizeof( uint8_t )* REGMAP_KEY_LENGTH );
    }
    catch( ... )
    {
        mConnectionUniversal->SetWriteEnable( false );
        throw;
    }

    mConnectionUniversal->SetWriteEnable( false );


    // Read info about the license. This infor is needed to know if the license is valid
    lResultLicense.mLicense = aLicense;
    uint32_t lResultLicenseInfo;

    mConnectionUniversal->ReadRegister( GetBankAddress( REGMAP_CMD_LIST ) + offsetof( sCmdList, mLicenceInfo ) + ( lEmptySlotIndex * sizeof( uint32_t ) ),
                                        ( uint8_t * )&lResultLicenseInfo,
                                        sizeof( uint32_t ), 5 );
    lResultLicense.mType = lResultLicenseInfo & 0xFFFF;
    lResultLicense.mSubType = lResultLicenseInfo >> 16;

    if( lResultLicense.mType == 0 )
    {
        throw std::runtime_error( "Invalid license." );
    }

    return lResultLicense;
}

// *****************************************************************************
// Function: LdSensorVu::RemoveLicense
//
/// \brief   Remove license on the device
///
/// \param  aLicense License to remove.
///
/// \author  Patrick Boulay
///
/// \since   August 2016
// *****************************************************************************

void
LdSensorVu::RemoveLicense( const std::string &aLicense )
{

    // Get licience from device
    LeddarDefines::sLicense lResultLicense;
    std::vector<LeddarDefines::sLicense> lLicenses = GetLicenses();

    // Looking for license slot
    int8_t lSlotIndex = -1;

    for( uint8_t i = 0; i < lLicenses.size(); ++i )
    {
        if( lLicenses[ i ].mLicense == LeddarUtils::LtStringUtils::ToLower( aLicense ) )
        {
            lSlotIndex = i;
            break;
        }
    }

    // Write the empty license on the device
    if( lSlotIndex != -1 )
    {
        mConnectionUniversal->SetWriteEnable( true );

        try
        {
            uint8_t lEmptyLicense[ 16 ] = { 0 };
            mConnectionUniversal->WriteRegister( GetBankAddress( REGMAP_LICENSE_KEYS ) + ( lSlotIndex * REGMAP_KEY_LENGTH ), lEmptyLicense, sizeof( uint8_t )* REGMAP_KEY_LENGTH );
        }
        catch( ... )
        {
            mConnectionUniversal->SetWriteEnable( false );
            throw;
        }

        mConnectionUniversal->SetWriteEnable( false );
    }
}

// *****************************************************************************
// Function: LdSensorVu::RemoveAllLicenses
//
/// \brief   Remove all licenses on the device
///
/// \author  Patrick Boulay
///
/// \since   March 2017
// *****************************************************************************

void
LdSensorVu::RemoveAllLicenses( void )
{
    mConnectionUniversal->SetWriteEnable( true );

    try
    {
        uint8_t lEmptyLicense[ 16 ] = { 0 };

        for( uint8_t i = 0; i < LICENSE_NUMBER; ++i )
        {
            mConnectionUniversal->WriteRegister( GetBankAddress( REGMAP_LICENSE_KEYS ) + ( i * REGMAP_KEY_LENGTH ), lEmptyLicense, sizeof( uint8_t )* REGMAP_KEY_LENGTH );
        }
    }
    catch( ... )
    {
        mConnectionUniversal->SetWriteEnable( false );
        throw;
    }

    mConnectionUniversal->SetWriteEnable( false );
}

// *****************************************************************************
// Function: LdSensorVu::ResetToDefaultWithoutWriteEnable
//
/// \brief   Reset parameters to default by NOT handling the write enable
///
/// \author  Patrick Boulay
///
/// \since   April 2016
// *****************************************************************************

void
LdSensorVu::ResetToDefaultWithoutWriteEnable( int16_t aCRCTry )
{
    try
    {
        // Get comm internal buffer.
        uint8_t *lInputBuffer, *lOutputBuffer;
        mConnectionUniversal->InternalBuffers( lInputBuffer, lOutputBuffer );

        if( !mConnectionUniversal->IsWriteEnable() )
        {
            throw std::runtime_error( "Error to erease chip (write enable)." );
        }

        mConnectionUniversal->Write( 0xC7, 0, 0, aCRCTry, 0, 0, 5000 );
        mConnectionUniversal->IsDeviceReady( 4000 );
    }
    catch( ... )
    {
        throw;
    }
}

// *****************************************************************************
// Function: LdSensorVu::ResetToDefault
//
/// \brief   Reset parameters to default by handling the write enable
///
/// \author  Patrick Boulay
///
/// \since   April 2016
// *****************************************************************************

void
LdSensorVu::ResetToDefault()
{
    mConnectionUniversal->SetWriteEnable( true );
    LeddarUtils::LtTimeUtils::Wait( 10 );
    ResetToDefaultWithoutWriteEnable( 5 );
    Reset( LeddarDefines::RT_SOFT_RESET );
    LeddarUtils::LtTimeUtils::Wait( 10 );
}

// *****************************************************************************
// Function: LdSensorVu::Reset
//
/// \brief   Reset the device
///
/// \param   aType Reset type
///
/// \author  Patrick Boulay
///
/// \since   April 2016
// *****************************************************************************

void
LdSensorVu::Reset( LeddarDefines::eResetType aType, LeddarDefines::eResetOptions )
{
    mConnectionUniversal->Reset( aType, 0 );
}

// *****************************************************************************
// Function: LdSensorVu::GetBankAddress
//
/// \brief   Get bank address from the register map
///
/// \param   aBankType Bank type
///
/// \author  Patrick Boulay
///
/// \since   October 2016
// *****************************************************************************

uint32_t
LdSensorVu::GetBankAddress( uint8_t aBankType )
{
    static const sRegMap gRegMap[ REGMAP_NBBANK ] = REGMAP( REGMAP_PRIMARY_KEY_PUBLIC, REGMAP_PRIMARY_KEY_TRACE,
            REGMAP_PRIMARY_KEY_INTEGRATOR, REGMAP_PRIMARY_KEY_ADMIN,
            REGMAP_PRIMARY_KEY_NO );

    return gRegMap[ aBankType ].mStartAddr;
}

// *****************************************************************************
// Function: LdSensorVu::SetTransferMode
//
/// \brief   Set the transfer mode
///
/// \param   aMode Transfer mode
///
/// \author  Patrick Boulay
///
/// \since   June 2017
// *****************************************************************************

void
LdSensorVu::SetTransferMode( eTransfertMode aMode )
{
    mConnectionUniversal->WriteRegister( GetBankAddress( REGMAP_TRN_CFG ) + offsetof( sTransactionCfg, mTransferMode ), ( uint8_t * )&aMode, 1, 5 );
}

// *****************************************************************************
// Function: LdSensorVu::CreateBackup
//
/// \brief   Backup the calibration settings.
///          Integrator license is required for this command.
///
/// \author  Patrick Boulay
///
/// \since   January 2018
// *****************************************************************************
void
LdSensorVu::CreateBackup( void )
{
    mConnectionUniversal->SetWriteEnable( true );
    LeddarUtils::LtTimeUtils::Wait( 10 );

    try
    {
        uint16_t lCRCTry = 5;
        uint32_t lDataSize = 0;
        mConnectionUniversal->Write( 0x57, 0, lDataSize, lCRCTry, 0, 0, 5000 );
        mConnectionUniversal->IsDeviceReady( 4000 );
    }
    catch( ... )
    {
        mConnectionUniversal->SetWriteEnable( false );
        throw;
    }

    mConnectionUniversal->SetWriteEnable( false );
    LeddarUtils::LtTimeUtils::Wait( 1000 );
}

// *****************************************************************************
// Function: LdSensorVu::DeleteBackup
//
/// \brief   Delete the calibation backup.
///          Integrator license is required for this command.
///
/// \author  Patrick Boulay
///
/// \since   January 2018
// *****************************************************************************
void
LdSensorVu::DeleteBackup( void )
{
    mConnectionUniversal->SetWriteEnable( true );

    try
    {
        uint16_t lCRCTry = 5;
        uint32_t lDataSize = 0;

        mConnectionUniversal->Write( 0x5E, 0, lDataSize, lCRCTry, 0, 0, 5000 );
        mConnectionUniversal->IsDeviceReady( 4000 );
    }
    catch( ... )
    {
        mConnectionUniversal->SetWriteEnable( false );
        throw;
    }

    mConnectionUniversal->SetWriteEnable( false );
}
