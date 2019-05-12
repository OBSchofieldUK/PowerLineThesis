// *****************************************************************************
// Module..: Leddar
//
/// \file    LdSensor.h
///
/// \brief   Base class of all sensors.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#pragma once

#include "LdConnection.h"
#include "LdDefines.h"
#include "LdDevice.h"
#include "LdResultEchoes.h"
#include "LdResultStates.h"

namespace LeddarDevice
{
    enum eLicenseType
    {
        LICENSE_NONE       = 0,
        LICENSE_ADMIN      = 1,
        LICENSE_INTEGRATOR = 2,
        LICENSE_PUBLIC     = 4,
        LICENSE_TRACE      = 8
    };

    class LdSensor : public LdDevice
    {
    public:
        enum eDataMask
        {
            DM_NONE             = 0,
            DM_STATES           = 1,
            DM_ECHOES           = 2,
            DM_ALL              = DM_STATES | DM_ECHOES
        };

        ~LdSensor();
        virtual void                        GetConfig( void )                        = 0;
        virtual void                        SetConfig( void )                        = 0;
        virtual void                        WriteConfig( void ) {}
        virtual void                        RestoreConfig( void ) {}
        virtual void                        GetConstants( void )                     = 0;
        virtual bool                        GetData( void );
        virtual bool                        GetEchoes( void )                        = 0;
        virtual void                        GetStates( void )                        = 0;
        virtual void                        Reset( LeddarDefines::eResetType aType, LeddarDefines::eResetOptions aOptions = LeddarDefines::RO_NO_OPTION ) = 0;
        LeddarConnection::LdResultEchoes   *GetResultEchoes( void ) { return &mEchoes; }
        LeddarConnection::LdResultStates   *GetResultStates( void ) { return &mStates; }

        virtual void                        SetDataMask( uint32_t aDataMask ) { mDataMask = aDataMask; }

        LdSensor                           *GetCarrier( void ) { return mCarrier; }
        void                                SetCarrier( LdSensor *aCarrier );

        virtual void                        RemoveLicense( const std::string & /*aLicense*/ ) {}
        virtual void                        RemoveAllLicenses( void ) {}
        virtual LeddarDefines::sLicense     SendLicense( const std::string & /*aLicense*/ ) { return LeddarDefines::sLicense(); }
        virtual std::vector<LeddarDefines::sLicense> GetLicenses( void ) { return std::vector<LeddarDefines::sLicense> ( 0 ); }

    protected:
        LdSensor( LeddarConnection::LdConnection *aConnection, LeddarCore::LdPropertiesContainer *aProperties = nullptr );
        LeddarConnection::LdResultEchoes mEchoes;
        LeddarConnection::LdResultStates mStates;

        static uint32_t GetDataMaskAll( void ) { return DM_ALL; }
        virtual uint32_t ConvertDataMaskToLTDataMask( uint32_t aMask );
        uint32_t mDataMask;

    private:
        LdSensor *mCarrier;
    };
}
