// *****************************************************************************
// Module..: Leddar
//
/// \file    LdPropertyIds.h
///
/// \brief   ID list of properties.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************
#pragma once

namespace LeddarCore
{
    namespace LdPropertyIds
    {
        enum
        {
            // Sensor's properties
            ID_DEVICE_TYPE                  = 0x0009,
            ID_DEVICE_NAME                  = 0x0059,
            ID_PART_NUMBER                  = 0x0010,
            ID_SOFTWARE_PART_NUMBER         = 0x0011,
            ID_MANUFACTURER_NAME            = 0x0012,
            ID_SERIAL_NUMBER                = 0x0013,
            ID_BUILD_DATE                   = 0x0014,
            ID_FIRMWARE_VERSION_STR         = 0x0015,
            ID_BOOTLOADER_VERSION           = 0x0016,
            ID_ASIC_VERSION                 = 0x0017,
            ID_FPGA_VERSION                 = 0x0018,
            ID_GROUP_ID_NUMBER              = 0x0062,
            ID_CARRIER_FIRMWARE_VERSION     = 0x0071,
            ID_CARRIER_FIRMWARE_PART_NUMBER = 0x0093,
            ID_CARRIER_PART_NUMBER          = 0x0072,
            ID_CARRIER_SERIAL_NUMBER        = 0x0073,
            ID_CARRIER_OPTIONS              = 0x0074,
            ID_CARRIER_SOFTWARE_PART_NUMBER = 0x0075,
            ID_MAC_ADDRESS                  = 0x0078,   // Mac address in text format
            ID_MAC_ADDRESS_ARRAY            = 0x007A,   // Mac address in uint8_t[6]
            ID_IP_ADDRESS                   = 0x0090,
            ID_IP_MODE                      = 0x0106,   //DHCP Mode
            ID_DATA_SERVER_PORT             = 0x0098,
            ID_DATA_SERVER_PROTOCOL         = 0x009A,
            ID_OPTIONS                      = 0x0019,
            ID_ACQ_OPTIONS                  = 0x0092,
            ID_TEMPERATURE_SCALE            = 0x0067,
            ID_CPU_LOAD_SCALE               = 0x0068,
            ID_APD_TEMPERATURE_SCALE        = 0x006A,
            ID_CRC32                        = 0x0080,
            ID_ANGLE_OVR                    = 0x0097,
            ID_TEST_MODE                    = 0x0099,
            ID_APD_VBIAS_VOLTAGE_T0         = 0x010A,
            ID_APD_VBIAS_MULTI_FACTOR       = 0x010B,
            ID_APD_VBIAS_T0                 = 0x010C,
            ID_APD_OPTIONS                  = 0x010D,
            ID_MEMS_PHASE                   = 0x010E,
            ID_BUFFER_SIZE_TCP              = 0x010F,
            ID_BUFFER_SIZE_UDP              = 0x0110,
            ID_XTALK_OPTIC_SEG_ENABLE       = 0x0111,
            ID_XTALK_OPTIC_LINE_ENABLE      = 0x0112,
            ID_XTALK_OPTIC_ECH_SEG_ENABLE   = 0x115A,
            ID_XTALK_OPTIC_ECH_LINE_ENABLE  = 0x115B,
            ID_APD_TRACK_TEMP_COMP_ENABLE   = 0x0114,
            ID_ACC_DIST_ENABLE              = 0x0115,
            ID_ACC_DIST_POSITION            = 0x0116,
            ID_ACC_DIST_EXP                 = 0x0117,
            ID_LIMIT_ACC_DIST_POS           = 0x0118,
            ID_LIMIT_ACC_DIST_EXP           = 0x0119,
            ID_THRESHOLD_OPTIONS            = 0x011A,
            ID_AUTOMATIC_THRESHOLD_SENSI    = 0x011B,
            ID_THREHSOLD_POS_OFFSET         = 0x011C,
            ID_LIMIT_THREHSOLD_POS_OFFSET   = 0x011D,
            ID_TEMP_COMP                    = 0x011F,

            ID_VSEGMENT                     = 0x0020,
            ID_HSEGMENT                     = 0x0021,
            ID_RSEGMENT                     = 0x0070,
            ID_REF_SEG_MASK                 = 0x0069,

            ID_BASE_SAMPLE_DISTANCE         = 0x0022,
            ID_DETECTION_PER_SEGMENT        = 0x0023,
            ID_DETECTION_LENGTH             = 0x002a,
            ID_DISTANCE_SCALE               = 0x0024,
            ID_RAW_AMP_SCALE_BITS           = 0x0025,
            ID_RAW_AMP_SCALE                = 0x0026,
            ID_AMP_SCALE                    = 0x0058,
            ID_FILTERED_AMP_SCALE_BITS      = 0x0027,
            ID_FILTERED_AMP_SCALE           = 0x0084,
            ID_NB_SAMPLE_MAX                = 0x0063,

            ID_ACCUMULATION_EXP             = 0x0028,
            ID_ACCUMULATION_LIMITS          = 0x0087,
            ID_OVERSAMPLING_EXP             = 0x0029,
            ID_OVERSAMPLING_LIMITS          = 0x0088,
            ID_BASE_POINT_COUNT             = 0x0030,
            ID_BASE_POINT_COUNT_LIMITS      = 0x0086,
            ID_SEGMENT_ENABLE               = 0x0031,
            ID_REF_PULSE_RATE               = 0x0060,
            ID_PRECISION                    = 0x0032,
            ID_PRECISION_ENABLE             = 0x0033,
            ID_PRECISION_LIMITS             = 0x0107,
            ID_XTALK_ECHO_REMOVAL_ENABLE    = 0x0034,
            ID_XTALK_REMOVAL_ENABLE         = 0x0035,
            ID_SATURATION_COMP_ENABLE       = 0x0036,
            ID_OVERSHOOT_MNG_ENABLE         = 0x0037,
            ID_SENSIVITY                    = 0x0038, //Sensitivity = threshold offset
            ID_SENSIVITY_LIMITS             = 0x0085,
            ID_PULSE_RATE                   = 0x0066,
            ID_CHANGE_DELAY                 = 0x0081,
            ID_CHANGE_DELAY_LIMITS          = 0x0089,
            ID_GAIN_ENABLE                  = 0x0082,
            ID_REFRESH_RATE                 = 0x0091,
            ID_TRACE_POINT_STEP             = 0x011E, //Distance between two points in the trace
            ID_START_TRACE                  = 0x0094,
            ID_START_TRACE_LIMITS           = 0x0095,
            ID_NUMBER_TRACE_SENT            = 0x009B,
            ID_DISTANCE_RESOLUTION          = 0x0096,
            ID_ECHO_AMPLITUDE_MAX           = 0x009C,

            ID_LED_INTENSITY                = 0x0039,
            ID_LED_INTENSITY_LIST           = 0x0103,
            ID_LED_PWR_ENABLE               = 0x0040,
            ID_LED_PWR_POURCENTAGE          = 0x0041,
            ID_LED_AUTO_PWR_ENABLE          = 0x0042,
            ID_LED_AUTO_FRAME_AVG           = 0x0043,
            ID_LED_AUTO_ECHO_AVG            = 0x0044,
            ID_LED_AUTO_FRAME_AVG_LIMITS    = 0x004A,
            ID_LED_AUTO_ECHO_AVG_LIMITS     = 0x004B,
            ID_LED_USR_PWR_COUNT            = 0x0061,
            ID_PWM_LASER                    = 0x0108,

            ID_DEMERGING_ENABLE             = 0x0045,
            ID_TRACE_BUFFER_TYPE            = 0x0046,
            ID_STATIC_NOISE_REMOVAL_ENABLE  = 0x0047,
            ID_STATIC_NOISE_UPDATE_ENABLE   = 0x0100, //LeddarOne only
            ID_STATIC_NOISE_UPDATE_RATE     = 0x0101, //LeddarOne only
            ID_STATIC_NOISE_UPDATE_AVERAGE  = 0x0102, //LeddarOne only
            ID_LEARNED_TRACE_OPTIONS        = 0x0065,
            ID_ALGO_REQUESTS                = 0x006B,

            ID_THRESH_AGG_AMP               = 0x1153,
            ID_THRESH_VICTIM_DIST           = 0x1154,
            ID_THRESH_ELEC_AGG_AMP          = 0x1155,
            ID_THRESH_ECH_VICTIM_LEFT       = 0x1156,
            ID_THRESH_ECH_VICTIM_RIGHT      = 0x1157,
            ID_THRESH_GAUSS_SENSIB          = 0x1158,
            ID_THRESH_M_PEAK                = 0x1159,

            ID_TIMEBASE_DELAY               = 0x0048,
            ID_PEAK_CALIBRATION_OFFSET      = 0x0049,
            ID_INTENSITY_COMPENSATIONS      = 0x0050, //led compensation
            ID_REAL_DISTANCE_OFFSET         = 0x0064,
            ID_MAX_ECHOES_PER_CHANNEL       = 0x0079,

            ID_ORIGIN_X                     = 0x0051,
            ID_ORIGIN_Y                     = 0x0052,
            ID_ORIGIN_Z                     = 0x0053,
            ID_YAW                          = 0x0054,
            ID_PITCH                        = 0x0055,
            ID_ROLL                         = 0x0056,
            ID_FOV                          = 0x0057,
            ID_HFOV                         = 0x0076,
            ID_VFOV                         = 0x0077,

            ID_LICENSE                      = 0x0104,
            ID_LICENSE_INFO                 = 0x0105,
            ID_VOLATILE_LICENSE             = 0x0120,
            ID_VOLATILE_LICENSE_INFO        = 0x0121,

            ID_COM_SERIAL_PORT_BAUDRATE         = 0x1000,
            ID_COM_SERIAL_PORT_DATA_BITS        = 0x1001,
            ID_COM_SERIAL_PORT_PARITY           = 0x1002,
            ID_COM_SERIAL_PORT_STOP_BITS        = 0x1003,
            ID_COM_SERIAL_PORT_ADDRESS          = 0x1004,
            ID_COM_SERIAL_PORT_FLOW_CONTROL     = 0x1005,
            ID_COM_SERIAL_PORT_BAUDRATE_OPTIONS = 0x1006,
            ID_COM_SERIAL_PORT_LOGICAL_PORT     = 0x1007,
            ID_COM_SERIAL_PORT_MAX_ECHOES       = 0x1008,
            ID_COM_SERIAL_PORT_ECHOES_RES       = 0x1009,
            ID_COM_SERIAL_PORT_CURRENT_PORT     = 0x100A,
            ID_COM_CAN_PORT_BAUDRATE            = 0x100B,
            ID_COM_CAN_PORT_TX_MSG_BASE_ID      = 0x100C,
            ID_COM_CAN_PORT_RX_MSG_BASE_ID      = 0x100D,
            ID_COM_CAN_PORT_FRAME_FORMAT        = 0x100E,
            ID_COM_CAN_PORT_PORT_OPTIONS        = 0x100F,
            ID_COM_CAN_PORT_PORT_MAILBOX_DELAY  = 0x1010,
            ID_COM_CAN_PORT_PORT_ACQCYCLE_DELAY = 0x1012,
            ID_COM_CAN_PORT_PORT_MAX_ECHOES     = 0x1013,
            ID_COM_CAN_PORT_PORT_OPTIONS_MASK   = 0x1014,
            ID_COM_CAN_PORT_LOGICAL_PORT        = 0x1015,
            ID_COM_CAN_PORT_MAX_ECHOES          = 0x1016,
            ID_COM_CAN_PORT_ECHOES_RES          = 0x1017,


            // Result State's properties
            ID_RS_TIMESTAMP                     = 0x9000,
            ID_RS_SYSTEM_TEMP                   = 0x9001,
            ID_RS_PREDICT_TEMP                  = 0x9002,
            ID_RS_CPU_LOAD                      = 0x9003,
            ID_RS_DISCRETE_OUTPUTS              = 0x9004,   /// Flag to know if there is something in the detection zone (IS16 & evalkit)
            ID_RS_ACQ_CURRENT_PARAMS            = 0x9005,   /// Sent with every trace, contains led power and several flags
            ID_RS_APD_TEMP                      = 0x9006,
            ID_RS_BACKUP                        = 0x9007,   /// Calibration backup: 0=invalid backup, 1=factory backup, 2=user backup
            ID_RS_APD_GAIN                      = 0x9008,
            ID_RS_NOISE_LEVEL                   = 0x9009,
            ID_RS_ADC_RSSI                      = 0x900A,
            ID_RS_SNR                           = 0x900B
        };
    }

}

