// *****************************************************************************
// Module..: Leddar
//
/// \file    LdSensorVu.h
///
/// \brief   Base class of sensor Leddar Vu.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#pragma once

#include "LdSensor.h"
#include "LdConnectionUniversal.h"

namespace LeddarDevice
{

    class LdSensorVu : virtual public LdSensor
    {

    public:
        enum eTransfertMode
        {
            TM_FREE_RUN,
            TM_BLOCKING,
            TM_PARTIAL_BLOCKING
        };

        ~LdSensorVu( void );
        virtual bool                                GetEchoes( void ) override;
        virtual void                                GetConfig( void ) override;
        virtual void                                SetConfig( void ) override;
        virtual void                                GetConstants( void ) override;
        virtual void                                GetStates( void ) override;
        void                                        ResetToDefaultWithoutWriteEnable( int16_t aCRCTry = 0 );
        void                                        ResetToDefault( void );
        virtual void                                RemoveLicense( const std::string &aLicense ) override;
        virtual void                                RemoveAllLicenses( void ) override;
        virtual LeddarDefines::sLicense             SendLicense( const std::string &aLicense ) override;
        virtual std::vector<LeddarDefines::sLicense> GetLicenses( void ) override;
        virtual void                                Reset( LeddarDefines::eResetType aType, LeddarDefines::eResetOptions aOptions = LeddarDefines::RO_NO_OPTION ) override;

        void                                        CreateBackup( void );
        void                                        DeleteBackup( void );

        void                                        SetTransferMode( eTransfertMode aMode );
        static uint32_t                             GetBankAddress( uint8_t aBankType );

    protected:
        explicit LdSensorVu( LeddarConnection::LdConnection *aConnection );

        virtual void                               InitProperties( void );
        LeddarConnection::LdConnectionUniversal   *mConnectionUniversal;
        uint16_t                                   mChannelCount;
        bool                                       mRepair;

    private:
        bool                                       mErrorFlag;
        bool                                       mBackupFlagAvailable;
    };
}