////////////////////////////////////////////////////////////////////////////////////////////////////
/// Module..: Leddar
///
/// \file   LdSensorM16.h.
///
/// \brief  Class of the M16 sensor
///
/// \since  August 2018
///
/// Copyright (c) 2018 LeddarTech Inc. All rights reserved.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "LdSensor.h"
#include "LdProtocolLeddartechUSB.h"

namespace LeddarDevice
{
    class LdSensorM16 : public LdSensor
    {
    public:
        explicit LdSensorM16( LeddarConnection::LdConnection *aConnection );
        ~LdSensorM16();

        virtual void Connect( void ) override;

        virtual void GetConstants( void ) override;
        virtual void GetConfig( void ) override;
        virtual void SetConfig( void ) override;
        virtual void WriteConfig( void ) override;
        virtual void RestoreConfig( void ) override;
        void         GetCalib();

        virtual void SetDataMask( uint32_t aDataMask ) override;
        virtual bool GetData( void ) override;
        virtual bool GetEchoes( void ) override { return false; }; //Dont use this function, use GetData
        virtual void GetStates( void ) override {}; //Dont use this function, use GetData

        virtual void    Reset( LeddarDefines::eResetType aType, LeddarDefines::eResetOptions aOptions = LeddarDefines::RO_NO_OPTION ) override;
        void            RequestProperties( LeddarCore::LdPropertiesContainer *aProperties, std::vector<uint16_t> aDeviceIds );
        void            SetProperties( LeddarCore::LdPropertiesContainer *aProperties, std::vector<uint16_t> aDeviceIds, unsigned int aRetryNbr = 0 );

        virtual void                    RemoveLicense( const std::string &aLicense ) override;
        virtual void                    RemoveAllLicenses( void ) override;
        virtual LeddarDefines::sLicense SendLicense( const std::string &aLicense ) override;
        LeddarDefines::sLicense         SendLicense( const std::string &aLicense, bool aVolatile );
        LeddarDefines::sLicense         SendLicense( const uint8_t *aLicense, bool aVolatile = false );
        virtual std::vector<LeddarDefines::sLicense> GetLicenses( void ) override;

    protected:
        LeddarConnection::LdProtocolLeddartechUSB    *mProtocolConfig;
        LeddarConnection::LdProtocolLeddartechUSB    *mProtocolData;
        LeddarCore::LdPropertiesContainer            *mResultStatePropeties;

        std::vector< LeddarConnection::LdProtocolLeddartechUSB::LdEcho> mProtocolEchoes;

        virtual bool    ProcessStates( void );
        void            ProcessEchoes( void );

    private:
        void InitProperties( void );
        void GetListing( void );
        void GetIntensityMappings( void );
        void UpdateConstants( void );

        // Supported command
        bool mHasBeamRange;
        bool mHasThresholdMin;
        bool mHasIntensityMapping;

    };
}