// *****************************************************************************
// Module..: Leddar
//
/// \file    LdConnectionInfoEthernet.h
///
/// \brief   Connection information on ethernet devices.
///
/// \author  Patrick Boulay
///
/// \since   July 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#pragma once
#include "LdConnectionInfo.h"
#include "LtDefines.h"

namespace LeddarConnection
{
    class LdConnectionInfoEthernet : public LdConnectionInfo
    {
    public:
        enum eProtocolType
        {
            PT_TCP,
            PT_UDP
        };

        LdConnectionInfoEthernet( const std::string &aIP, const uint32_t aPort, const std::string &aDescription,
                                  const eConnectionType aType, const eProtocolType aProtocolType = PT_TCP,
                                  const std::string &aUsed = "UNKNOWN", uint32_t aTimeout = 1000,
                                  const std::string &aDisplayName = "" );
        ~LdConnectionInfoEthernet();

        std::string     GetIP( void ) const { return mIP; }
        uint32_t        GetPort( void ) const { return mPort;}
        std::string     GetDescription( void ) const { return mDescription; }
        uint32_t        GetTimeout( void ) const { return mTimeout; }
        void            SetTimeout( uint32_t aTimeout ) { mTimeout = aTimeout; } //To be used before connect
        std::string     GetUsed( void ) const { return mUsed; }
        eProtocolType   GetProtocoleType( void ) const { return mProtocolType; }
        uint32_t        GetDeviceType( void ) const { return mDeviceType; }
        void            SetDeviceType( uint32_t aDeviceType ) { mDeviceType = aDeviceType; }

    protected:
        std::string     mIP;
        uint32_t        mPort;
        std::string     mDescription;
        uint32_t        mTimeout;
        std::string     mUsed;
        eProtocolType   mProtocolType;
        uint32_t        mDeviceType;
    };
}
