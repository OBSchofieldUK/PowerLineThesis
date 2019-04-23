// *****************************************************************************
/// \file    LtComM16.h
///
/// \brief   Structures and defines for M16 sensor communication.
///
/// \author  Patrick Boulay
///
/// \since   March 2017
//
// Copyright (c) 2017 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#pragma once

#include "comm/LtComEthernetPublic.h"

namespace LtComM16
{

    //*****************************************************************************
    //*************** Constants and Macros ****************************************
    //*****************************************************************************
#define M16_MAX_ECHOES                      96       ///< Maximum number of echoes.
#define M16_LED_INTENSITY_MAX               16       ///< Number of led intensity.
#define M16_NUMBER_CHANNELS                 16       ///< Number of Leddar channels.

#define M16_CFG_REQUEST_COMPUTE(_offset)   (LtComLeddarTechPublic::LT_COMM_CFGSRV_REQUEST_PLATFORM_SPECIFIC_BASE + _offset)
#define M16_ID_COMPUTE(_offset)                 (LtComLeddarTechPublic::LT_COMM_ID_PLATFORM_SPECIFIC_BASE + _offset)

#define M16_FILTERED_SCALE                  8192


    //*****************************************************************************
    //*************** Data Type Definition ****************************************
    //*****************************************************************************

    // Element Id definitions: (beware some IDs in this range are defined at the LtCom level.
    typedef enum _eLtCommPlatformM16ElementIdCodes
    {
        /// \brief  M16 platform specific elements ID codes.
        M16_ID_CFG_THRESHOLD_TABLE_OFFSET   = M16_ID_COMPUTE( 0x0000 ),   ///< (0x1000) {LtFixedPoint} Threshold table offset in M16_FILTERED_SCALE.
        M16_ID_CAL_CHAN_TIMEBASE_DELAY      = M16_ID_COMPUTE( 0x0014 ),   ///< (0x1014) {LtFixedPoint}[number of channels] Calibration time base delay of each segments.
        M16_ID_CFG_TRANS_IMP_GAIN           = M16_ID_COMPUTE( 0x007B ),   ///< (0x107B) {LtUInt8} FPGA parameter: transimpedance gain from M16_DEFAULT_TRANS_IMP_GAIN_MIN to M16_DEFAULT_TRANS_IMP_GAIN_MAX.
        M16_ID_CFG_LED_INTENSITY            = M16_ID_COMPUTE( 0x0080 ),   ///< (0x1080) {LtUInt8} Led intensity from M16_DEFAULT_LED_INTENSITY_MIN to M16_DEFAULT_LED_INTENSITY_MAX.
        M16_ID_CFG_SENSOR_POSITION_X        = M16_ID_COMPUTE( 0x0090 ),   ///< (0x1090) {LtFloat32} Sensor X position.
        M16_ID_CFG_SENSOR_POSITION_Y        = M16_ID_COMPUTE( 0x0091 ),   ///< (0x1091) {LtFloat32} Sensor Y position.
        M16_ID_CFG_SENSOR_POSITION_Z        = M16_ID_COMPUTE( 0x0092 ),   ///< (0x1092) {LtFloat32} Sensor Z position.
        M16_ID_CFG_SENSOR_ORIENTATION_YAW   = M16_ID_COMPUTE( 0x0093 ),   ///< (0x1093) {LtFloat32} Sensor yaw orientation.
        M16_ID_CFG_SENSOR_ORIENTATION_PITCH = M16_ID_COMPUTE( 0x0094 ),   ///< (0x1094) {LtFloat32} Sensor pitch orientation.
        M16_ID_CFG_SENSOR_ORIENTATION_ROLL  = M16_ID_COMPUTE( 0x0095 ),   ///< (0x1095) {LtFloat32} Sensor roll orientation.
        M16_ID_CFG_SERIAL_PORT_BAUDRATE     = M16_ID_COMPUTE( 0x0096 ),   ///< (0x1096) {LtUInt32}[Number of serial port] Baudrate value. See \ref eLtCommPlatformM16SerialBaudrate.
        M16_ID_CFG_SERIAL_PORT_DATA_BITS    = M16_ID_COMPUTE( 0x0097 ),   ///< (0x1097) {LtUInt8}[Number of serial port] Number of bits per data (8 or 9). See \ref eLtCommPlatformM16SerialDataBits.
        M16_ID_CFG_SERIAL_PORT_PARITY       = M16_ID_COMPUTE( 0x0098 ),   ///< (0x1098) {LtUInt8}[Number of serial port] Parity. See \ref eLtCommPlatformM16SerialParity.
        M16_ID_CFG_SERIAL_PORT_STOP_BITS    = M16_ID_COMPUTE( 0x0099 ),   ///< (0x1099) {LtUInt8}[Number of serial port] Stop bits count. See \ref eLtCommPlatformM16SerialStopBits.
        M16_ID_CFG_SERIAL_PORT_ADDRESS      = M16_ID_COMPUTE( 0x009A ),   ///< (0x109A) {LtUInt8}[Number of serial port] ModBus address (1 to 247).
        M16_ID_CFG_SERIAL_PORT_FLOW_CONTROL = M16_ID_COMPUTE( 0x009B ),   ///< (0x109B) {LtUInt8}[Number of serial port] Flow control mode. See \ref eLtCommPlatformM16SerialFlowControl.
        M16_ID_CFG_CAN_PORT_BAUDRATE        = M16_ID_COMPUTE( 0x009C ),   ///< (0x109C) {LtUInt32}[Number of CAN port] Baudrate value. See \ref eLtCommPlatformM16CanBaudrate.
        M16_ID_CFG_CAN_PORT_TX_MSG_BASE_ID  = M16_ID_COMPUTE( 0x009D ),   ///< (0x109D) {LtUInt32}[Number of CAN port] Tx message base ID.
        M16_ID_CFG_CAN_PORT_RX_MSG_BASE_ID  = M16_ID_COMPUTE( 0x009E ),   ///< (0x109E) {LtUInt32}[Number of CAN port] Rx message base ID.
        M16_ID_CFG_CAN_PORT_FRAME_FORMAT    = M16_ID_COMPUTE( 0x009F ),   ///< (0x109F) {LtUInt8}[Number of CAN port] Tx Rx frame format. See \ref eLtCommPlatformM16CanFrameFormat.

        // IDs included from 0x10A0 to 0x10A5 are reserved

        M16_ID_DISCRETE_OUTPUTS = M16_ID_COMPUTE( 0x00A6 ),   ///< (0x10A6) {LtUInt32} Bits fields discrete outputs state.

        // IDs included from 0x10A7 to 0x10A8 are reserved

        M16_ID_CFG_ACQ_OPTIONS                          = M16_ID_COMPUTE( 0x00A9 ),   ///< (0x10A9) {LtUInt16} Bits field of acquisition options. See \ref eLtCommPlatformM16AcqOptions.
        M16_ID_CFG_AUTO_ACQ_AVG_FRM                     = M16_ID_COMPUTE( 0x00AA ),   ///< (0x10AA) {LtUInt16} Number of frame to evaluate new automatic acquisition parameters from M16_DEFAULT_AUTO_AVG_FRM_MIN to M16_DEFAULT_AUTO_AVG_FRM_MAX.
        M16_ID_ACQ_CURRENT_PARAMS                       = M16_ID_COMPUTE( 0x00AB ),   ///< (0x10AB) {LtUInt32} Current acquisition parameters states.
        M16_ID_MEASUREMENT_RATE                         = M16_ID_COMPUTE( 0x00AC ),   ///< (0x10AC) {LtFixedPoint} Measurement rate in M16_MEASUREMENT_RATE_SCALE.
        M16_ID_MEASUREMENT_RATE_LIST                    = M16_ID_COMPUTE( 0x00AD ),   ///< (0x10AD) {LtFixedPoint}[] List of supported measurement rate in M16_MEASUREMENT_RATE_SCALE.
        M16_ID_BEAM_RANGE                               = M16_ID_COMPUTE( 0x00AE ),   ///< (0x10AE) {LtFixedPoint} Beam range in M16_DISTANCE_SCALE for displaying accommodation.
        M16_ID_LED_POWER                                = M16_ID_COMPUTE( 0x00AF ),   ///< (0x10AF) {LtUInt8} Led power in percent.
        M16_ID_LED_POWER_LIST                           = M16_ID_COMPUTE( 0x00B0 ),   ///< (0x10B0) {LtUInt8}[] List of supported led power in percent.
        M16_ID_LIMIT_CFG_THRESHOLD_TABLE_OFFSET         = M16_ID_COMPUTE( 0x00B1 ),   ///< (0x10B1) {LtFixedPoint}[2] Minimum and maximum threshold table offset permitted to M16_ID_CFG_THRESHOLD_TABLE_OFFSET
        ///<                            from the current configuration. Call \ref M16_CFGSRV_REQUEST_PARAMS_TO_THRES_TABLE_OFFSET_MIN
        ///<                            to get the minimum value with a different configuration.
        M16_ID_LIMIT_CFG_LED_INTENSITY                  = M16_ID_COMPUTE( 0x00B2 ),   ///< (0x10B2) {LtUInt8}[2] Minimum and maximum led intensity permitted to M16_ID_CFG_LED_INTENSITY.
        M16_ID_LIMIT_CFG_AUTO_ACQ_AVG_FRM               = M16_ID_COMPUTE( 0x00B3 ),   ///< (0x10B3) {LtUInt16}[2] Minimum and maximum number of auto acq avg frame permitted to M16_ID_CFG_AUTO_ACQ_AVG_FRM.
        M16_ID_CFG_SERIAL_PORT_MAX_ECHOES               = M16_ID_COMPUTE( 0x00B4 ),   ///< (0x10B4) {LtUInt8}[Number of serial port] Maximum number of echoes to send on the serial port.
        M16_ID_LIMIT_CFG_SERIAL_PORT_MAX_ECHOES         = M16_ID_COMPUTE( 0x00B5 ),   ///< (0x10B5) {LtUInt8} Maximum number of echoes permitted to M16_ID_CFG_SERIAL_PORT_MAX_ECHOES.
        M16_ID_DEFAULT_CFG_AUTO_ACQ_AVG_FRM             = M16_ID_COMPUTE( 0x00B6 ),   ///< (0x10B6) {LtUInt16} Default value of auto acq avg frame to set to M16_ID_CFG_AUTO_ACQ_AVG_FRM.
        M16_ID_CAL_LED_INTENSITY                        = M16_ID_COMPUTE( 0x00B7 ),   ///< (0x10B7) {LtFixedPoint}[17] Led intensity distance compensation
        M16_ID_CFG_BAYES_PRECISION                      = M16_ID_COMPUTE( 0x00B8 ),   ///< (0x10B8) {LtInt8} Bayes detector precision adjustment
        M16_ID_ACQUISITION_OPTION_MASK                  = M16_ID_COMPUTE( 0x00B9 ),   ///< (0x10B9) {LtUInt16} Acquisition option mask
        M16_ID_TRACE_TYPE                               = M16_ID_COMPUTE( 0x00BA ),   ///< (0x10BA) {LtUInt8} Select the trace type to send
        M16_ID_PREDICTED_TEMP                           = M16_ID_COMPUTE( 0x00BB ),   ///< (0x10BB) {LtInt32} Predicted temperature
        M16_ID_CFG_CAN_PORT_OPTIONS                     = M16_ID_COMPUTE( 0x00BC ),   ///< (0x10BC) {LtUInt16}[Number of CAN port] CAN port flag options: see \ref eLtCommPlatformM16CanOptions.
        M16_ID_CFG_CAN_PORT_MAILBOX_DELAY               = M16_ID_COMPUTE( 0x00BD ),   ///< (0x10BD) {LtUInt16}[Number of CAN port] Inter-mailbox(es) message delay in msec.
        M16_ID_CFG_CAN_PORT_ACQCYCLE_DELAY              = M16_ID_COMPUTE( 0x00BE ),   ///< (0x10BE) {LtUInt16}[Number of CAN port] Inter-cycle message block delay in msec.
        M16_ID_CFG_CAN_PORT_MAX_ECHOES                  = M16_ID_COMPUTE( 0x00BF ),   ///< (0x10BF) {LtUInt8}[Number of CAN port] Maximum number of echoes to send on the serial port.
        M16_ID_LIMIT_CFG_CAN_PORT_MAX_ECHOES            = M16_ID_COMPUTE( 0x00C0 ),   ///< (0x10C0) {LtUInt8} Maximum number of echoes permitted to M16_ID_CFG_CAN_PORT_MAX_ECHOES.
        M16_ID_CFG_LWECHOES_DIST_RES                    = M16_ID_COMPUTE( 0x00C1 ),   ///< (0x10C1) {LtUInt16} Lightweight echoes distance resolution.
        M16_ID_CFG_LWECHOES_CHANNEL_SELECT              = M16_ID_COMPUTE( 0x00C2 ),   ///< (0x10C2) {LtUInt16} Bits field of selected channel to send by lightweight echoes module.
        M16_ID_CAN_PORT_OPTIONS_MASK                    = M16_ID_COMPUTE( 0x00C3 ),   ///< (0x10C3) {LtUInt16} CAN port flag option mask of available options.
        M16_ID_SERIAL_PORT_BAUDRATE_OPTIONS_MASK        = M16_ID_COMPUTE( 0x00C4 ),   ///< (0x10C4) {LtUInt16}[Number of serial port] Bits field of supported baudrate.
        M16_ID_TEST_MODE                                = M16_ID_COMPUTE( 0x00C5 ),   ///< (0x10C5) {LtUInt16} Test mode.
        M16_ID_NB_CYCLE_PER_SCAN                        = M16_ID_COMPUTE( 0x00C6 ),   ///< (0x10C6) {LtUInt16} Number of cycle per scan in clock cycle.
        M16_ID_PWM_PERIOD                               = M16_ID_COMPUTE( 0x00C7 ),   ///< (0x10C7) {LtUInt16} PWM period in clock cycle.
        M16_ID_PWM_TABLE                                = M16_ID_COMPUTE( 0x00C8 ),   ///< (0x10C8) {LtUInt8}[16] PWM charge pulse in clock cycle.
        M16_ID_CFG_START_TRACE_INDEX                    = M16_ID_COMPUTE( 0x00C9 ),   ///< (0x10C9) {LtUInt32} Index where the peak detector begin.
        M16_ID_LIMIT_START_TRACE_INDEX                  = M16_ID_COMPUTE( 0x00CA ),   ///< (0x10CA) {LtUInt32}[2] Minimum and maximum index where the peak detector start to compute a pulse.
        M16_ID_DATA_LEVEL                               = M16_ID_COMPUTE( 0x00CB ),   ///< (0x10CB) {LtUInt32} See LT_COMM_DATA_LEVEL_...
    } eLtCommPlatformM16ElementIdCodes;


    typedef enum _eLtCommPlatformM16CfgSrvRequestCode
    {
        /// \brief  M16 platform specific configuration server request codes.
        M16_CFGSRV_REQUEST_MESUREMENT_RATE_TO_PARAMS = M16_CFG_REQUEST_COMPUTE( 0x0000 ),           ///< (0x1000) Convert measurement rate to accumulation and oversampling parameters.
        M16_CFGSRV_REQUEST_PARAMS_TO_MESUREMENT_RATE = M16_CFG_REQUEST_COMPUTE( 0x0001 ),           ///< (0x1001) Convert accumulation and oversampling parameters to measurement rate.
        M16_CFGSRV_REQUEST_LED_POWER_TO_PARAMS = M16_CFG_REQUEST_COMPUTE( 0x0002 ),                 ///< (0x1002) Convert led power to led intensity parameters.
        M16_CFGSRV_REQUEST_PARAMS_TO_LED_POWER = M16_CFG_REQUEST_COMPUTE( 0x0003 ),                 ///< (0x1003) Convert led intensity parameters to led power.
        M16_CFGSRV_REQUEST_PARAMS_TO_THRES_TABLE_OFFSET_MIN = M16_CFG_REQUEST_COMPUTE( 0x0004 ),    ///< (0x1004) Convert accumulation and oversampling parameters to the minimum permitted of table offset threshold.
        M16_CFGSRV_REQUEST_BASE_SAMPLE_COUNT_TO_BEAM_RANGE = M16_CFG_REQUEST_COMPUTE( 0x0005 )      ///< (0x1005) Convert base sample count to beam range.
    } eLtCommPlatforM16CfgSrvRequestCode;

    typedef enum _eLtCommDeviceOptions
    {
        /// \brief  Device option bit field definitions with mask and specific option definitions.
        LT_COMM_DEVICE_OPTION_NONE = 0x00000000,   ///< Nothing option selected.
        LT_COMM_DEVICE_OPTION_ALL = 0xFFFFFFFF,   ///< All option selected.

        // ****************************
        // * Bit field definitions    *
        // * M16 families             *
        // ****************************
        LT_COMM_DEVICE_OPTION_LFOV_0 = 0x00000001,   ///< Leddar sensor field of view, see LT_COMM_DEVICE_OPTION_xx_DEG_LFOV.
        LT_COMM_DEVICE_OPTION_DEMO_KIT = 0x00000002,   ///< Demo unit: 0=normal, 1=demo kit
        LT_COMM_DEVICE_OPTION_PIN = 0x00000004,   ///< Sensor type: 0=APD, 1=PIN.
        LT_COMM_DEVICE_OPTION_NOPANTILT = 0x00000008,   ///< Pan & tilt option: 0=activated, 1=deactivated. See LT_COMM_DEVICE_OPTION_NOPANTILT_xx.
        LT_COMM_DEVICE_OPTION_LFOV_1 = 0x00000010,   ///< Leddar sensor field of view, see LT_COMM_DEVICE_OPTION_xx_DEG_LFOV.
        LT_COMM_DEVICE_OPTION_LFOV_2 = 0x00000020,   ///< Leddar sensor field of view, see LT_COMM_DEVICE_OPTION_xx_DEG_LFOV.
        LT_COMM_DEVICE_OPTION_TRIGGER = 0x00000040,   ///< Trigger option: 0=none, 1=present. See LT_COMM_DEVICE_OPTION_TRIGGER_xx.
        LT_COMM_DEVICE_OPTION_CFOV_0 = 0x00000080,   ///< Camera field of view, see LT_COMM_DEVICE_OPTION_xx_DEG_CFOV.
        LT_COMM_DEVICE_OPTION_CFOV_1 = 0x00000100,   ///< Camera field of view, see LT_COMM_DEVICE_OPTION_xx_DEG_CFOV.
        LT_COMM_DEVICE_OPTION_SENSOR_ORIENTATION = 0x00000200,   ///< Sensor orientation: see LT_COMM_DEVICE_OPTION_SENSOR_ORIENTATION_xx.
        LT_COMM_DEVICE_OPTION_GEN2 = 0x00000400,   ///< Product generation: see LT_COMM_DEVICE_OPTION_GEN_xx.
        LT_COMM_DEVICE_OPTION_CFOV_2 = 0x00000800,   ///< Camera field of view, see LT_COMM_DEVICE_OPTION_xx_DEG_CFOV.
        LT_COMM_DEVICE_OPTION_GHO_0 = 0x00001000,   ///< M16 hardware option field, see LT_COMM_DEVICE_OPTION_GHO_xx.
        LT_COMM_DEVICE_OPTION_GHO_1 = 0x00002000,   ///< M16 hardware option field, see LT_COMM_DEVICE_OPTION_GHO_xx.
        LT_COMM_DEVICE_OPTION_GHO_2 = 0x00004000,   ///< M16 hardware option field, see LT_COMM_DEVICE_OPTION_GHO_xx.
        LT_COMM_DEVICE_OPTION_GHO_3 = 0x00008000,   ///< M16 hardware option field, see LT_COMM_DEVICE_OPTION_GHO_xx.
        LT_COMM_DEVICE_OPTION_GHO_4 = 0x00010000,   ///< M16 hardware option field, see LT_COMM_DEVICE_OPTION_GHO_xx.


        // *******************************
        // * Mask and option definitions *
        // * M16 families                *
        // *******************************
        // - Leddar sensor optic field of view mask and option definitions.
        LT_COMM_DEVICE_OPTION_LFOV_MASK = LT_COMM_DEVICE_OPTION_LFOV_0 | LT_COMM_DEVICE_OPTION_LFOV_1 | LT_COMM_DEVICE_OPTION_LFOV_2,   ///< Leddar sensor optic mask.
        LT_COMM_DEVICE_OPTION_18_DEG_LFOV = LT_COMM_DEVICE_OPTION_NONE,                                   ///< 18 degrees Leddar sensor optic.
        LT_COMM_DEVICE_OPTION_34_DEG_LFOV = LT_COMM_DEVICE_OPTION_LFOV_0,                                 ///< 34 degrees Leddar sensor optic.
        LT_COMM_DEVICE_OPTION_26_DEG_LFOV = LT_COMM_DEVICE_OPTION_LFOV_1,                                 ///< 26 degrees Leddar sensor optic.
        LT_COMM_DEVICE_OPTION_60_DEG_LFOV = LT_COMM_DEVICE_OPTION_LFOV_0 | LT_COMM_DEVICE_OPTION_LFOV_1,  ///< 60 degrees Leddar sensor optic.
        LT_COMM_DEVICE_OPTION_45_DEG_LFOV = LT_COMM_DEVICE_OPTION_LFOV_2,                                 ///< 45 degrees Leddar sensor optic.
        LT_COMM_DEVICE_OPTION_10_DEG_LFOV = LT_COMM_DEVICE_OPTION_LFOV_2 | LT_COMM_DEVICE_OPTION_LFOV_0,  ///< 10 degrees Leddar sensor optic.
        LT_COMM_DEVICE_OPTION_100_DEG_LFOV = LT_COMM_DEVICE_OPTION_LFOV_2 | LT_COMM_DEVICE_OPTION_LFOV_1,  ///< 100 degrees Leddar sensor optic (currently available on M16 platform).

        // - Camera optic field of view mask and option definitions.
        LT_COMM_DEVICE_OPTION_CFOV_MASK = LT_COMM_DEVICE_OPTION_CFOV_0 | LT_COMM_DEVICE_OPTION_CFOV_1 | LT_COMM_DEVICE_OPTION_CFOV_2,   ///< Camera optic mask.
        LT_COMM_DEVICE_OPTION_37_DEG_CFOV = LT_COMM_DEVICE_OPTION_NONE,                                   ///< 37 degrees camera optic.
        LT_COMM_DEVICE_OPTION_58_DEG_CFOV = LT_COMM_DEVICE_OPTION_CFOV_0,                                 ///< 58 degrees camera optic.
        LT_COMM_DEVICE_OPTION_17_DEG_CFOV = LT_COMM_DEVICE_OPTION_CFOV_1,                                 ///< 17 degrees camera optic.
        LT_COMM_DEVICE_OPTION_28_DEG_CFOV = LT_COMM_DEVICE_OPTION_CFOV_0 | LT_COMM_DEVICE_OPTION_CFOV_1,  ///< 28 degrees camera optic.
        LT_COMM_DEVICE_OPTION_27_DEG_CFOV = LT_COMM_DEVICE_OPTION_CFOV_2,                                 ///< 27 degrees camera optic.

        // - Pan and tilt mask and option definitions.
        LT_COMM_DEVICE_OPTION_NOPANTILT_MASK = LT_COMM_DEVICE_OPTION_NOPANTILT,                              ///< Pan and tilt mask.
        LT_COMM_DEVICE_OPTION_NOPANTILT_PRESENT = LT_COMM_DEVICE_OPTION_NONE,                                   ///< Pan and tilt present and activated.
        LT_COMM_DEVICE_OPTION_NOPANTILT_NOT_PRESENT = LT_COMM_DEVICE_OPTION_NOPANTILT,                              ///< Pan and tilt no present or not activated.

        // - Physical trigger mask and option definitions.
        LT_COMM_DEVICE_OPTION_TRIGGER_MASK = LT_COMM_DEVICE_OPTION_TRIGGER,                                ///< Physical trigger mask.
        LT_COMM_DEVICE_OPTION_TRIGGER_NOT_PRESENT = LT_COMM_DEVICE_OPTION_NONE,                                   ///< No physical trigger.
        LT_COMM_DEVICE_OPTION_TRIGGER_PRESENT = LT_COMM_DEVICE_OPTION_TRIGGER,                                ///< Physical trigger present.

        // - Sensor orientation mask and option definitions.
        LT_COMM_DEVICE_OPTION_SENSOR_ORIENTATION_MASK = LT_COMM_DEVICE_OPTION_SENSOR_ORIENTATION,                     ///< Sensor orientation mask.
        LT_COMM_DEVICE_OPTION_SENSOR_ORIENTATION_HORIZ = LT_COMM_DEVICE_OPTION_NONE,                                   ///< Horizontal sensing (Ex. dtec).
        LT_COMM_DEVICE_OPTION_SENSOR_ORIENTATION_VERT = LT_COMM_DEVICE_OPTION_SENSOR_ORIENTATION,                     ///< Vertical sensing (Ex. side-tec).

        // - Device product generation mask and option definitions.
        LT_COMM_DEVICE_OPTION_GEN_MASK = LT_COMM_DEVICE_OPTION_GEN2,                                        ///< Product generation mask.
        LT_COMM_DEVICE_OPTION_GEN_FIRST = LT_COMM_DEVICE_OPTION_NONE,                                       ///< First product generation.
        LT_COMM_DEVICE_OPTION_GEN_SECOND = LT_COMM_DEVICE_OPTION_GEN2,                                      ///< Second product generation.

        // - M16 product hardware mask and option definitions.
        LT_COMM_DEVICE_OPTION_GHO_MASK = LT_COMM_DEVICE_OPTION_GHO_0 | LT_COMM_DEVICE_OPTION_GHO_1 | LT_COMM_DEVICE_OPTION_GHO_2 | LT_COMM_DEVICE_OPTION_GHO_3 | LT_COMM_DEVICE_OPTION_GHO_4,   ///< M16 hardware option mask.
        LT_COMM_DEVICE_OPTION_GHO_NO_DISPLAY = LT_COMM_DEVICE_OPTION_GHO_0,                                 ///< IS16 without display option.
        LT_COMM_DEVICE_OPTION_GHO_CAN = LT_COMM_DEVICE_OPTION_GHO_1,                                        ///< IS16 CAN port option (serial port disabled).
        LT_COMM_DEVICE_OPTION_GHO_RAW_DATA = LT_COMM_DEVICE_OPTION_GHO_2,                                   ///< IS16 with raw data output only.
        LT_COMM_DEVICE_OPTION_GHO_LASER_PRODUCT = LT_COMM_DEVICE_OPTION_GHO_3,                              ///< M16 product contain LASER source.
        LT_COMM_DEVICE_OPTION_GHO_FAILSAFE = LT_COMM_DEVICE_OPTION_GHO_4                                    ///< M16 product contain a fail safe option for test mode.

    } eLtCommDeviceOptions;

    typedef enum _eLtCommPlatformM16AcqOptions
    {
        /// \brief  Bits field acquisition options saved in configuration.
        M16_ACQ_OPTIONS_NONE = 0x0000,           ///< No acquisition options selected.

        M16_ACQ_OPTIONS_AUTO_LED_INTENSITY = 0x0001,        ///< Automatic led intensity.
        M16_ACQ_OPTIONS_AUTO_TRANSIMP_GAIN = 0x0002,        ///< Automatic transimpedance gain. This is available only if automatic led intensity is enabled.
        M16_ACQ_OPTIONS_DEMERGE_OBJECTS = 0x0004,           ///< Enable the two object demerge algorithm.
        M16_ACQ_OPTIONS_XTALK_REMOVAL_DISABLE = 0x0008,     ///< Disable xtalk removal algorithm (logical inverse).
        M16_ACQ_OPTIONS_TEMP_COMP_DISABLE = 0x0010,         ///< Disable temperature compensation (logical inverse).

        M16_ACQ_OPTIONS_MASK = 0x001F                       ///< Supported option mask.
    } eLtCommPlatformM16AcqOptions;


    typedef enum _eLtCommPlatformM16SerialBaudrate
    {
        /// \brief  Serial port baudrate enumeration.
        M16_SERIAL_PORT_BAUDRATE_9600 = 9600,       ///< 9600 bps
        M16_SERIAL_PORT_BAUDRATE_19200 = 19200,     ///< 19200 bps
        M16_SERIAL_PORT_BAUDRATE_38400 = 38400,     ///< 38400 bps
        M16_SERIAL_PORT_BAUDRATE_57600 = 57600,     ///< 57600 bps
        M16_SERIAL_PORT_BAUDRATE_115200 = 115200,   ///< 115200 bps
        M16_SERIAL_PORT_BAUDRATE_230400 = 230400,   ///< 230400 bps
        M16_SERIAL_PORT_BAUDRATE_460800 = 460800,   ///< 460800 bps
        M16_SERIAL_PORT_BAUDRATE_921600 = 921600    ///< 921600 bps
    } eLtCommPlatformM16SerialBaudrate;

    typedef enum _eLtCommPlatformM16SerialBaudrateOptionMask
    {
        /// \brief  Serial of supported port baudrate.
        M16_SERIAL_PORT_OPTION_BAUDRATE_9600 = 0x0001,      ///< 9600 bps option is available
        M16_SERIAL_PORT_OPTION_BAUDRATE_19200 = 0x0002,     ///< 19200 bps option is available
        M16_SERIAL_PORT_OPTION_BAUDRATE_38400 = 0x0004,     ///< 38400 bps option is available
        M16_SERIAL_PORT_OPTION_BAUDRATE_57600 = 0x0008,     ///< 57600 bps option is available
        M16_SERIAL_PORT_OPTION_BAUDRATE_115200 = 0x0010,    ///< 115200 bps option is available
        M16_SERIAL_PORT_OPTION_BAUDRATE_230400 = 0x0020,    ///< 230400 bps option is available
        M16_SERIAL_PORT_OPTION_BAUDRATE_460800 = 0x0040,    ///< 460800 bps option is available
        M16_SERIAL_PORT_OPTION_BAUDRATE_921600 = 0x0080,    ///< 921600 bps option is available

        M16_SERIAL_PORT_OPTION_BAUDRATE_STANDARD = 0x001F
    } eLtCommPlatformM16SerialBaudrateOptionMask;

    typedef enum _eLtCommPlatformM16SerialDataBits
    {
        /// \brief  Serial port data bits enumeration.
        M16_SERIAL_PORT_DATA_8_BITS = 8,            ///< Data on 8 bits.
        M16_SERIAL_PORT_DATA_9_BITS = 9             ///< Data on 9 bits.
    } eLtCommPlatformM16SerialDataBits;

    typedef enum _eLtCommPlatformM16SerialParity
    {
        /// \brief  Serial port parity enumeration.
        M16_SERIAL_PORT_PARITY_NONE = 0,        ///< No parity.
        M16_SERIAL_PORT_PARITY_ODD = 1,         ///< Odd parity.
        M16_SERIAL_PORT_PARITY_EVEN = 2         ///< Even parity.
    } eLtCommPlatformM16SerialParity;

    typedef enum _eLtCommPlatformM16SerialStopBits
    {
        /// \brief  Serial port stop bits enumeration.
        M16_SERIAL_PORT_STOP_1_BIT = 1,            ///< One stop bit.
        M16_SERIAL_PORT_STOP_2_BITS = 2            ///< Two stop bits.
    } eLtCommPlatformM16SerialStopBits;

    typedef enum _eLtCommPlatformM16SerialFlowControl
    {
        /// \brief  Serial port flow control enumeration.
        M16_SERIAL_PORT_CONTROL_NONE = 0,            ///< No flow control.
        M16_SERIAL_PORT_CONTROL_CTS_RTS = 1             ///< CTS-RTS flow control.
    } eLtCommPlatformM16SerialFlowControl;

    typedef enum _eLtCommPlatformM16CanBaudrate
    {
        /// \brief  CAN port baudrate enumeration.
        M16_CAN_PORT_BAUDRATE_10KBPS = 10,      ///< 10 Kbps
        M16_CAN_PORT_BAUDRATE_20KBPS = 20,      ///< 20 Kbps
        M16_CAN_PORT_BAUDRATE_50KBPS = 50,      ///< 50 Kbps
        M16_CAN_PORT_BAUDRATE_100KBPS = 100,    ///< 100 Kbps
        M16_CAN_PORT_BAUDRATE_125KBPS = 125,    ///< 125 Kbps
        M16_CAN_PORT_BAUDRATE_250KBPS = 250,    ///< 250 Kbps
        M16_CAN_PORT_BAUDRATE_500KBPS = 500,    ///< 500 Kbps
        M16_CAN_PORT_BAUDRATE_1MBPS = 1000      ///< 1 Mbps
    } eLtCommPlatformM16CanBaudrate;

    typedef enum _eLtCommPlatformM16CanFrameFormat
    {
        /// \brief  CAN port frame format.
        M16_CAN_PORT_FRAME_FORMAT_STANDARD = 0,        ///< Standard 11 bits.
        M16_CAN_PORT_FRAME_FORMAT_EXTENDED = 1         ///< Extended 29 bits.
    } eLtCommPlatformM16CanFrameFormat;

    typedef enum _eLtCommPlatformM16CanOptions
    {
        /// \brief  Bits field of CAN options.
        M16_CAN_PORT_OPTIONS_NONE = 0x0000,                 ///< No CAN option selected.

        M16_CAN_PORT_OPTIONS_MULT_MSG_ID_FORMAT = 0x0001,   ///< Send detection on multiple message ID.
        M16_CAN_PORT_OPTIONS_MSGBOX_DELAY = 0x0002,         ///< Delay in msec between a message sent in a message box(es).
        M16_CAN_PORT_OPTIONS_ACQCYCLE_DELAY = 0x0004,       ///< Delay in msec between cycle of acquisition message frame.
        M16_CAN_PORT_OPTIONS_FLAG_INFO = 0x0008,            ///< Use detection message with flag information.

        M16_CAN_PORT_OPTIONS_MASK = 0x000F                  ///< Supported option mask.
    } eLtCommPlatformM16CanOptions;


    //*****************************************************************************
    //*************** Public Function Declarations ********************************
    //*****************************************************************************

}